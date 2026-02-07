/*
Copyright (C) 1997-2001 Id Software, Inc.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/
// cl.input.c  -- builds an intended movement command to send to the server

#include "client.h"
#include "common/crc.h"

#if USE_SDL
#include <SDL.h>
#endif

static cvar_t    *cl_nodelta;
static cvar_t    *cl_maxpackets;
static cvar_t    *cl_packetdup;
static cvar_t    *cl_fuzzhack;
#if USE_DEBUG
static cvar_t    *cl_showpackets;
#endif
static cvar_t    *cl_instantpacket;
static cvar_t    *cl_batchcmds;

static cvar_t    *m_filter;
static cvar_t    *m_accel;
static cvar_t    *m_autosens;

static cvar_t    *cl_upspeed;
static cvar_t    *cl_forwardspeed;
static cvar_t    *cl_sidespeed;
static cvar_t    *cl_yawspeed;
static cvar_t    *cl_pitchspeed;
static cvar_t    *cl_run;
static cvar_t    *cl_anglespeedkey;

static cvar_t    *freelook;
static cvar_t    *lookspring;
static cvar_t    *lookstrafe;
static cvar_t    *sensitivity;

static cvar_t    *m_pitch;
static cvar_t    *m_yaw;
static cvar_t    *m_forward;
static cvar_t    *m_side;

/*
===============================================================================

INPUT SUBSYSTEM

===============================================================================
*/

typedef struct {
    bool        modified;
    int         old_dx;
    int         old_dy;
} in_state_t;

static in_state_t   input;

static cvar_t    *in_enable;
static cvar_t    *in_grab;

#if USE_SDL
static cvar_t    *in_joystick;
static cvar_t    *in_joystick_auto;
static cvar_t    *joy_autobind;
static cvar_t    *joy_deadzone;
static cvar_t    *joy_move_scale;
static cvar_t    *joy_look_scale;
static cvar_t    *joy_invert_y;
static cvar_t    *joy_trigger_threshold;
static cvar_t    *joy_rumble_enable;
static cvar_t    *joy_haptic_magnitude;
static cvar_t    *joy_haptic_distance;

typedef struct {
    bool                sdl_initialized;
    SDL_GameController  *controller;
    SDL_JoystickID      instance_id;
    SDL_JoystickID      auto_lockout_id;
    SDL_Haptic          *haptic;
    bool                has_gc_rumble;
    bool                rumble_fail_warned;

    Sint16              raw_lx, raw_ly;
    Sint16              raw_rx, raw_ry;
    Sint16              raw_lt, raw_rt;

    float               lx, ly;
    float               rx, ry;

    bool                buttons[SDL_CONTROLLER_BUTTON_MAX];
    bool                trigger_l_down;
    bool                trigger_r_down;

    bool                menu_left;
    bool                menu_right;
    bool                menu_up;
    bool                menu_down;

    bool                bind_attempted;
} gamepad_t;

static gamepad_t gamepad;
#endif

static bool IN_GetCurrentGrab(void)
{
    if (cls.active != ACT_ACTIVATED)
        return false;   // main window doesn't have focus

    if (r_config.flags & QVF_FULLSCREEN)
        return true;    // full screen

    if (cls.key_dest & (KEY_MENU | KEY_CONSOLE))
        return false;   // menu or console is up

    if (sv_paused->integer)
        return false;   // game paused

    if (cls.state != ca_active)
        return false;   // not connected

    if (in_grab->integer >= 2) {
        if (cls.demo.playback && !Key_IsDown(K_SHIFT))
            return false;   // playing a demo (and not using freelook)

        if (cl.frame.ps.pmove.pm_type == PM_FREEZE)
            return false;   // spectator mode
    }

    if (in_grab->integer >= 1)
        return true;    // regular playing mode

    return false;
}

#if USE_SDL
static float apply_deadzone(float x, float dz)
{
    float ax = fabsf(x);

    if (ax <= dz) {
        return 0.0f;
    }
    if (dz >= 0.999f) {
        return 0.0f;
    }

    return (x > 0.0f ? 1.0f : -1.0f) * ((ax - dz) / (1.0f - dz));
}

static float normalize_axis(Sint16 v)
{
    if (v < 0) {
        return (float)v / 32768.0f;
    }
    return (float)v / 32767.0f;
}

static int translate_controller_button(SDL_GameControllerButton button)
{
    int b = (int)button;

    if (b < 0) {
        return -1;
    }

    if (b < 4) {
        return K_JOY1 + b;
    }

    b -= 4;
    if (b >= 0 && b < 28) {
        return K_AUX1 + b;
    }

    return -1;
}

static void virtual_key_event(unsigned key, bool *state, bool down, unsigned time)
{
    if (*state == down) {
        return;
    }

    *state = down;
    Key_Event(key, down, time);
}

static void close_controller(unsigned time)
{
    if (!gamepad.controller) {
        return;
    }

    // Release any pressed gamepad keys to avoid stuck +commands.
    for (int b = 0; b < SDL_CONTROLLER_BUTTON_MAX; b++) {
        if (!gamepad.buttons[b]) {
            continue;
        }

        int key = translate_controller_button((SDL_GameControllerButton)b);
        if (key >= 0) {
            Key_Event((unsigned)key, false, time);
        }
        gamepad.buttons[b] = false;
    }

    virtual_key_event(K_AUX27, &gamepad.trigger_l_down, false, time);
    virtual_key_event(K_AUX28, &gamepad.trigger_r_down, false, time);

    virtual_key_event(K_LEFTARROW, &gamepad.menu_left, false, time);
    virtual_key_event(K_RIGHTARROW, &gamepad.menu_right, false, time);
    virtual_key_event(K_UPARROW, &gamepad.menu_up, false, time);
    virtual_key_event(K_DOWNARROW, &gamepad.menu_down, false, time);

    if (gamepad.haptic) {
        SDL_HapticClose(gamepad.haptic);
        gamepad.haptic = NULL;
    }
    gamepad.has_gc_rumble = false;
    gamepad.rumble_fail_warned = false;

    SDL_GameControllerClose(gamepad.controller);
    gamepad.controller = NULL;
    gamepad.instance_id = -1;

    gamepad.raw_lx = gamepad.raw_ly = 0;
    gamepad.raw_rx = gamepad.raw_ry = 0;
    gamepad.raw_lt = gamepad.raw_rt = 0;

    gamepad.lx = gamepad.ly = 0.0f;
    gamepad.rx = gamepad.ry = 0.0f;
}

static void apply_default_gamepad_binds(bool force)
{
    // Baseline binds on JOY/AUX only. Non-destructive unless -force.
    static const struct {
        int key;
        const char *cmd;
    } binds[] = {
        // Triggers
        { K_AUX28, "+attack" },
        { K_AUX27, "+use" },

        // Face buttons
        { K_JOY1, "+moveup" },
        { K_JOY2, "+movedown" },
        { K_JOY3, "invuse" },
        { K_JOY4, "inven" },

        // Bumpers
        { K_AUX6, "weapprev" },
        { K_AUX7, "weapnext" },

        // D-pad
        { K_AUX8, "invuse" },
        { K_AUX9, "invdrop" },
        { K_AUX10, "invprev" },
        { K_AUX11, "invnext" },

        // Misc
        { K_AUX1, "score" },
        { K_AUX4, "centerview" },
        { K_AUX5, "+speed" },
    };

    bool any = false;

    for (int i = 0; i < q_countof(binds); i++) {
        const char *existing = Key_GetBindingForKey(binds[i].key);
        if (!force && existing[0]) {
            continue;
        }

        Key_SetBinding(binds[i].key, binds[i].cmd);
        any = true;
    }

    if (any) {
        Com_Printf("Gamepad: default binds applied (use joy_bind_defaults [-force])\n");
    }
}

static void joy_bind_defaults_f(void)
{
    bool force = false;

    for (int i = 1; i < Cmd_Argc(); i++) {
        const char *a = Cmd_Argv(i);
        if (!a || !a[0]) {
            continue;
        }
        if (!Q_stricmp(a, "force") || !Q_stricmp(a, "-force")) {
            force = true;
        }
    }

    apply_default_gamepad_binds(force);
}

static void joy_unbindall_f(void)
{
    for (int k = K_JOYFIRST; k <= K_JOYLAST; k++) {
        Key_SetBinding(k, NULL);
    }

    for (int k = K_AUXFIRST; k <= K_AUXLAST; k++) {
        Key_SetBinding(k, NULL);
    }

    Com_Printf("Gamepad: cleared JOY/AUX bindings (keyboard/mouse untouched)\n");
}

static void do_rumble(int low, int high, int ms)
{
    if (!gamepad.controller || !joy_rumble_enable || !joy_rumble_enable->integer) {
        return;
    }

    // Safety caps
    low = Q_clip(low, 0, 65535);
    high = Q_clip(high, 0, 65535);
    ms = Q_clip(ms, 0, 5000);

#if SDL_VERSION_ATLEAST(2, 0, 9)
    if (gamepad.has_gc_rumble) {
        if (SDL_GameControllerRumble(gamepad.controller, (Uint16)low, (Uint16)high, (Uint32)ms) == 0) {
            return;
        }
    }
#endif

    if (gamepad.haptic) {
        float strength = (float)(low > high ? low : high) / 65535.0f;
        strength = Q_clipf(strength, 0.0f, 1.0f);
        if (SDL_HapticRumblePlay(gamepad.haptic, strength, (Uint32)ms) == 0) {
            return;
        }
    }

    if (!gamepad.rumble_fail_warned) {
        Com_WPrintf("Gamepad rumble failed: %s\n", SDL_GetError());
        gamepad.rumble_fail_warned = true;
    }
}

static void joy_rumble_f(void)
{
    int low = 0, high = 0, ms = 200;

    if (!joy_rumble_enable || !joy_rumble_enable->integer) {
        return;
    }

    if (!gamepad.controller) {
        return;
    }

    if (Cmd_Argc() >= 2) {
        low = Q_atoi(Cmd_Argv(1));
    }
    if (Cmd_Argc() >= 3) {
        high = Q_atoi(Cmd_Argv(2));
    }
    if (Cmd_Argc() >= 4) {
        ms = Q_atoi(Cmd_Argv(3));
    }

    do_rumble(low, high, ms);
}

void IN_JoyRumbleTrigger(const char *name, const vec3_t origin, int entnum, float volume)
{
    float intensity = 0.0f;
    float low_freq = 1.0f;
    float high_freq = 1.0f;
    float dist_factor = 1.0f;
    int duration = 0;

    if (!gamepad.controller || !joy_rumble_enable || !joy_rumble_enable->integer) {
        return;
    }

    if (!name || !name[0] || volume <= 0.0f) {
        return;
    }

    float max_distance = joy_haptic_distance ? joy_haptic_distance->value : 1000.0f;
    bool from_player = entnum == cl.frame.clientNum + 1;

    if (origin && !from_player && max_distance > 0.0f) {
        float dist = Distance(origin, cl.refdef.vieworg);
        if (dist > max_distance) {
            return;
        }
        dist_factor = (max_distance - dist) / max_distance;
    }

    if (strstr(name, "weapons/")) {
        intensity = 1.0f;
        duration = 160;

        if (strstr(name, "hyprbf1") || strstr(name, "machgf")) {
            intensity = 0.5f;
        } else if (strstr(name, "chain")) {
            intensity = 0.6f;
        } else if (strstr(name, "rocklf") || strstr(name, "grenlf")) {
            intensity = 1.0f;
            duration = 250;
            low_freq = 1.5f;
            high_freq = 0.8f;
        } else if (strstr(name, "railgf1")) {
            intensity = 1.0f;
            duration = 300;
            low_freq = 2.0f;
            high_freq = 2.0f;
        } else if (strstr(name, "bfg")) {
            intensity = 1.0f;
            duration = 500;
            low_freq = 3.0f;
            high_freq = 3.0f;
        } else if (strstr(name, "shotgf")) {
            intensity = 0.8f;
            duration = 200;
        } else if (strstr(name, "blastf")) {
            intensity = 0.6f;
        }
    } else if (strstr(name, "player/") || strstr(name, "players/")) {
        if (strstr(name, "fall") || strstr(name, "pain")) {
            intensity = 1.0f;
            duration = 250;
            low_freq = 2.0f;
            high_freq = 1.5f;
        } else if (strstr(name, "death")) {
            intensity = 1.0f;
            duration = 800;
            low_freq = 3.0f;
            high_freq = 3.0f;
        } else if (strstr(name, "land")) {
            intensity = 0.3f;
            duration = 150;
        } else if (strstr(name, "jump")) {
            intensity = 0.1f;
            duration = 100;
        }
    } else if (strstr(name, "items/")) {
        intensity = 0.4f;
        duration = 150;

        if (strstr(name, "damage") || strstr(name, "protect") || strstr(name, "quad")) {
            intensity = 1.0f;
            duration = 400;
            low_freq = 2.0f;
            high_freq = 2.0f;
        } else if (strstr(name, "health") || strstr(name, "mega")) {
            intensity = strstr(name, "mega") || strstr(name, "l_health") ? 0.7f : 0.5f;
        }
    } else if ((strstr(name, "misc/") || strstr(name, "pickups/")) &&
               (strstr(name, "w_pkup") || strstr(name, "am_pkup"))) {
        intensity = 0.4f;
        duration = 120;
    } else if (strstr(name, "world/")) {
        if (strstr(name, "quake") || strstr(name, "rumble")) {
            intensity = 0.5f;
            duration = 500;
            low_freq = 1.5f;
        } else if (strstr(name, "explod")) {
            intensity = 0.8f;
            duration = 400;
            low_freq = 2.0f;
            high_freq = 1.5f;
        }
    }

    if (intensity <= 0.0f) {
        return;
    }

    float mag = joy_haptic_magnitude ? joy_haptic_magnitude->value : 1.0f;
    float vol_factor = Q_clipf(volume, 0.0f, 1.0f);

    if (mag <= 0.0f || vol_factor <= 0.0f) {
        return;
    }

    int final_low = (int)(65535.0f * intensity * low_freq * mag * dist_factor * vol_factor);
    int final_high = (int)(65535.0f * intensity * high_freq * mag * dist_factor * vol_factor);

    if (final_low == 0 && final_high == 0) {
        return;
    }

    do_rumble(final_low, final_high, duration);
}

static bool open_first_controller(bool skip_lockout)
{
    int num = SDL_NumJoysticks();

    if (gamepad.controller) {
        return true;
    }

    for (int i = 0; i < num; i++) {
        if (!SDL_IsGameController(i)) {
            continue;
        }

        if (skip_lockout) {
            SDL_JoystickID id = SDL_JoystickGetDeviceInstanceID(i);
            if (id != -1 && id == gamepad.auto_lockout_id) {
                continue;
            }
        }

        gamepad.controller = SDL_GameControllerOpen(i);
        if (!gamepad.controller) {
            continue;
        }

        SDL_Joystick *joy = SDL_GameControllerGetJoystick(gamepad.controller);
        gamepad.instance_id = joy ? SDL_JoystickInstanceID(joy) : -1;
        gamepad.haptic = NULL;
        gamepad.has_gc_rumble = false;
        gamepad.rumble_fail_warned = false;

        if (joy) {
            gamepad.haptic = SDL_HapticOpenFromJoystick(joy);
            if (gamepad.haptic) {
                if (SDL_HapticRumbleInit(gamepad.haptic) != 0) {
                    Com_Printf("Gamepad: HapticRumbleInit failed: %s\n", SDL_GetError());
                    SDL_HapticClose(gamepad.haptic);
                    gamepad.haptic = NULL;
                }
            }
        }

#if SDL_VERSION_ATLEAST(2, 0, 18)
        if (SDL_GameControllerHasRumble(gamepad.controller)) {
            gamepad.has_gc_rumble = true;
        }
#elif SDL_VERSION_ATLEAST(2, 0, 9)
        if (SDL_GameControllerRumble(gamepad.controller, 0, 0, 0) == 0) {
            gamepad.has_gc_rumble = true;
        }
#endif

        memset(gamepad.buttons, 0, sizeof(gamepad.buttons));
        gamepad.trigger_l_down = false;
        gamepad.trigger_r_down = false;
        gamepad.menu_left = false;
        gamepad.menu_right = false;
        gamepad.menu_up = false;
        gamepad.menu_down = false;

        const char *name = SDL_GameControllerName(gamepad.controller);
        Com_Printf("Gamepad: %s\n", name ? name : "Unknown");

        if (!gamepad.bind_attempted) {
            gamepad.bind_attempted = true;
            if (joy_autobind && joy_autobind->integer) {
                apply_default_gamepad_binds(false);
            }
        }

        return true;
    }

    return false;
}

static void update_controller_enabled(unsigned time)
{
    if (!in_joystick || !in_joystick_auto) {
        return;
    }

    if (in_joystick->modified) {
        in_joystick->modified = false;

        if (in_joystick->integer) {
            gamepad.auto_lockout_id = -1;
            open_first_controller(false);
        } else {
            if (gamepad.controller) {
                gamepad.auto_lockout_id = gamepad.instance_id;
            }
            close_controller(time);
        }
    }
}

static void poll_controller(unsigned time)
{
    if (!gamepad.sdl_initialized || !in_joystick || !in_joystick_auto) {
        return;
    }

    update_controller_enabled(time);

    if (gamepad.controller && !SDL_GameControllerGetAttached(gamepad.controller)) {
        Com_Printf("Gamepad disconnected.\n");
        close_controller(time);
    }

    // Open controller on demand.
    if (!gamepad.controller) {
        if (in_joystick->integer) {
            open_first_controller(false);
        } else if (in_joystick_auto->integer) {
            if (open_first_controller(true) && !in_joystick->integer) {
                Cvar_SetInteger(in_joystick, 1, FROM_CODE);
            }
        }
    }

    if (!gamepad.controller || !in_joystick->integer) {
        return;
    }

    SDL_GameControllerUpdate();

    // Axes
    gamepad.raw_lx = SDL_GameControllerGetAxis(gamepad.controller, SDL_CONTROLLER_AXIS_LEFTX);
    gamepad.raw_ly = SDL_GameControllerGetAxis(gamepad.controller, SDL_CONTROLLER_AXIS_LEFTY);
    gamepad.raw_rx = SDL_GameControllerGetAxis(gamepad.controller, SDL_CONTROLLER_AXIS_RIGHTX);
    gamepad.raw_ry = SDL_GameControllerGetAxis(gamepad.controller, SDL_CONTROLLER_AXIS_RIGHTY);
    gamepad.raw_lt = SDL_GameControllerGetAxis(gamepad.controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
    gamepad.raw_rt = SDL_GameControllerGetAxis(gamepad.controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

    float dz = joy_deadzone ? Q_clipf(joy_deadzone->value, 0.0f, 0.95f) : 0.15f;
    gamepad.lx = apply_deadzone(normalize_axis(gamepad.raw_lx), dz);
    gamepad.ly = apply_deadzone(normalize_axis(gamepad.raw_ly), dz);
    gamepad.rx = apply_deadzone(normalize_axis(gamepad.raw_rx), dz);
    gamepad.ry = apply_deadzone(normalize_axis(gamepad.raw_ry), dz);

    const bool key_wait = Key_IsWaiting();
    const bool in_menu = (cls.key_dest & KEY_MENU) != 0;

    // Triggers as digital buttons (useful for binds).
    int trig_thresh = 16384;
    if (joy_trigger_threshold) {
        trig_thresh = (int)(32767.0f * Q_clipf(joy_trigger_threshold->value, 0.001f, 1.0f) + 0.5f);
    }

    if (cls.active != ACT_ACTIVATED) {
        const int thresh = 16896;

        gamepad.menu_left = in_menu && !key_wait && gamepad.raw_lx < -thresh;
        gamepad.menu_right = in_menu && !key_wait && gamepad.raw_lx > thresh;
        gamepad.menu_up = in_menu && !key_wait && gamepad.raw_ly < -thresh;
        gamepad.menu_down = in_menu && !key_wait && gamepad.raw_ly > thresh;

        gamepad.trigger_l_down = gamepad.raw_lt > trig_thresh;
        gamepad.trigger_r_down = gamepad.raw_rt > trig_thresh;

        for (int b = 0; b < SDL_CONTROLLER_BUTTON_MAX; b++) {
            gamepad.buttons[b] = SDL_GameControllerGetButton(gamepad.controller, (SDL_GameControllerButton)b) != 0;
        }

        return;
    }

    // Virtual keys to navigate menus with left stick.
    if (in_menu && !key_wait) {
        const int thresh = 16896;

        virtual_key_event(K_LEFTARROW, &gamepad.menu_left, gamepad.raw_lx < -thresh, time);
        virtual_key_event(K_RIGHTARROW, &gamepad.menu_right, gamepad.raw_lx > thresh, time);
        virtual_key_event(K_UPARROW, &gamepad.menu_up, gamepad.raw_ly < -thresh, time);
        virtual_key_event(K_DOWNARROW, &gamepad.menu_down, gamepad.raw_ly > thresh, time);
    } else {
        virtual_key_event(K_LEFTARROW, &gamepad.menu_left, false, time);
        virtual_key_event(K_RIGHTARROW, &gamepad.menu_right, false, time);
        virtual_key_event(K_UPARROW, &gamepad.menu_up, false, time);
        virtual_key_event(K_DOWNARROW, &gamepad.menu_down, false, time);
    }

    virtual_key_event(K_AUX27, &gamepad.trigger_l_down, gamepad.raw_lt > trig_thresh, time);
    virtual_key_event(K_AUX28, &gamepad.trigger_r_down, gamepad.raw_rt > trig_thresh, time);

    // Buttons
    for (int b = 0; b < SDL_CONTROLLER_BUTTON_MAX; b++) {
        const bool down = SDL_GameControllerGetButton(gamepad.controller, (SDL_GameControllerButton)b) != 0;
        if (down == gamepad.buttons[b]) {
            continue;
        }

        gamepad.buttons[b] = down;

        SDL_GameControllerButton btn = (SDL_GameControllerButton)b;

        // In keybind wait mode, allow gamepad buttons to bind normally,
        // but map B/Back/Start to Escape to cancel binding.
        if (key_wait && in_menu) {
            if (btn == SDL_CONTROLLER_BUTTON_B || btn == SDL_CONTROLLER_BUTTON_BACK ||
                btn == SDL_CONTROLLER_BUTTON_START) {
                Key_Event(K_ESCAPE, down, time);
                continue;
            }
        }

        // In menus, map A button to Enter to select items.
        if (in_menu && !key_wait && btn == SDL_CONTROLLER_BUTTON_A) {
            Key_Event(K_ENTER, down, time);
        }

        // In menus, treat B/Back as Escape (back).
        if (in_menu) {
            if (btn == SDL_CONTROLLER_BUTTON_B || btn == SDL_CONTROLLER_BUTTON_BACK) {
                Key_Event(K_ESCAPE, down, time);
                continue;
            }
        }

        // Special-case Start as Escape by default, but still allow binds via AUX.
        if (btn == SDL_CONTROLLER_BUTTON_START) {
            Key_Event(K_ESCAPE, down, time);
        }

        int key = translate_controller_button(btn);
        if (key >= 0) {
            Key_Event((unsigned)key, down, time);
        }
    }
}

static void gamepad_init(void)
{
    in_joystick = Cvar_Get("in_joystick", "0", CVAR_ARCHIVE);
    in_joystick_auto = Cvar_Get("in_joystick_auto", "0", CVAR_ARCHIVE);
    joy_autobind = Cvar_Get("joy_autobind", "1", CVAR_ARCHIVE);
    joy_deadzone = Cvar_Get("joy_deadzone", "0.15", CVAR_ARCHIVE);
    joy_move_scale = Cvar_Get("joy_move_scale", "1.0", CVAR_ARCHIVE);
    joy_look_scale = Cvar_Get("joy_look_scale", "1800", CVAR_ARCHIVE);
    joy_invert_y = Cvar_Get("joy_invert_y", "0", CVAR_ARCHIVE);
    joy_trigger_threshold = Cvar_Get("joy_trigger_threshold", "0.55", CVAR_ARCHIVE);
    joy_rumble_enable = Cvar_Get("joy_rumble_enable", "1", CVAR_ARCHIVE);
    joy_haptic_magnitude = Cvar_Get("joy_haptic_magnitude", "1.0", CVAR_ARCHIVE);
    joy_haptic_distance = Cvar_Get("joy_haptic_distance", "1000.0", CVAR_ARCHIVE);

    if (gamepad.sdl_initialized) {
        return;
    }

    if (SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC) == -1) {
        Com_EPrintf("Couldn't initialize SDL gamecontroller: %s\n", SDL_GetError());
        return;
    }

    SDL_GameControllerEventState(SDL_IGNORE);
    SDL_JoystickEventState(SDL_IGNORE);
    gamepad.sdl_initialized = true;

    if ((in_joystick && in_joystick->integer) || (in_joystick_auto && in_joystick_auto->integer)) {
        if (open_first_controller(false) && in_joystick_auto->integer && !in_joystick->integer) {
            Cvar_SetInteger(in_joystick, 1, FROM_CODE);
        }
    }
}

static void gamepad_shutdown(unsigned time)
{
    close_controller(time);

    if (gamepad.sdl_initialized) {
        SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC);
        gamepad.sdl_initialized = false;
    }
}

static void gamepad_add_move(vec3_t move)
{
    if (!in_joystick || !in_joystick->integer) {
        return;
    }

    if (!gamepad.controller) {
        return;
    }

    if (cls.active != ACT_ACTIVATED) {
        return;
    }

    if (cls.key_dest != KEY_GAME) {
        return;
    }

    float move_scale = joy_move_scale ? joy_move_scale->value : 1.0f;

    move[1] += cl_sidespeed->value * move_scale * gamepad.lx;
    move[0] += cl_forwardspeed->value * move_scale * -gamepad.ly;
}
#endif

#if !USE_SDL
void IN_JoyRumbleTrigger(const char *name, const vec3_t origin, int entnum, float volume)
{
    (void)name;
    (void)origin;
    (void)entnum;
    (void)volume;
}
#endif

/*
============
IN_Activate
============
*/
void IN_Activate(void)
{
    if (vid && vid->grab_mouse) {
        vid->grab_mouse(IN_GetCurrentGrab());
    }
}

/*
============
IN_Restart_f
============
*/
static void IN_Restart_f(void)
{
    IN_Shutdown();
    IN_Init();
}

/*
============
IN_Frame
============
*/
void IN_Frame(void)
{
#if USE_SDL
    unsigned time = Sys_Milliseconds();
#endif

    if (input.modified) {
        IN_Restart_f();
    }

#if USE_SDL
    poll_controller(time);
#endif
}

/*
================
IN_WarpMouse
================
*/
void IN_WarpMouse(int x, int y)
{
    if (vid && vid->warp_mouse) {
        vid->warp_mouse(x, y);
    }
}

/*
============
IN_Shutdown
============
*/
void IN_Shutdown(void)
{
    if (in_grab) {
        in_grab->changed = NULL;
    }

    if (vid && vid->shutdown_mouse) {
        vid->shutdown_mouse();
    }

#if USE_SDL
    gamepad_shutdown(Sys_Milliseconds());
#endif

    memset(&input, 0, sizeof(input));
}

static void in_changed_hard(cvar_t *self)
{
    input.modified = true;
}

static void in_changed_soft(cvar_t *self)
{
    IN_Activate();
}

/*
============
IN_Init
============
*/
void IN_Init(void)
{
#if USE_SDL
    gamepad_init();
#endif

    in_enable = Cvar_Get("in_enable", "1", 0);
    in_enable->changed = in_changed_hard;
    if (!in_enable->integer) {
        Com_Printf("Mouse input disabled.\n");
        return;
    }

    if (!vid || !vid->init_mouse || !vid->init_mouse()) {
        Cvar_Set("in_enable", "0");
        return;
    }

    in_grab = Cvar_Get("in_grab", "1", 0);
    in_grab->changed = in_changed_soft;

    IN_Activate();
}


/*
===============================================================================

KEY BUTTONS

Continuous button event tracking is complicated by the fact that two different
input sources (say, mouse button 1 and the control key) can both press the
same button, but the button should only be released when both of the
pressing key have been released.

When a key event issues a button command (+forward, +attack, etc), it appends
its key number as a parameter to the command so it can be matched up with
the release.

state bit 0 is the current state of the key
state bit 1 is edge triggered on the up to down transition
state bit 2 is edge triggered on the down to up transition


Key_Event (int key, bool down, unsigned time);

  +mlook src time

===============================================================================
*/

typedef struct {
    int         down[2];        // key nums holding it down
    unsigned    downtime;        // msec timestamp
    unsigned    msec;            // msec down this frame
    int         state;
} kbutton_t;

static kbutton_t    in_klook;
static kbutton_t    in_left, in_right, in_forward, in_back;
static kbutton_t    in_lookup, in_lookdown, in_moveleft, in_moveright;
static kbutton_t    in_strafe, in_speed, in_use, in_attack;
static kbutton_t    in_up, in_down;

static int          in_impulse;
static bool         in_mlooking;

static void KeyDown(kbutton_t *b)
{
    int k;
    char *c;

    c = Cmd_Argv(1);
    if (c[0])
        k = Q_atoi(c);
    else
        k = -1;        // typed manually at the console for continuous down

    if (k == b->down[0] || k == b->down[1])
        return;        // repeating key

    if (!b->down[0])
        b->down[0] = k;
    else if (!b->down[1])
        b->down[1] = k;
    else {
        Com_WPrintf("Three keys down for a button!\n");
        return;
    }

    if (b->state & 1)
        return;        // still down

    // save timestamp
    c = Cmd_Argv(2);
    b->downtime = Q_atoi(c);
    if (!b->downtime) {
        b->downtime = com_eventTime - 100;
    }

    b->state |= 1 + 2;    // down + impulse down
}

static void KeyUp(kbutton_t *b)
{
    int k;
    char *c;
    unsigned uptime;

    c = Cmd_Argv(1);
    if (c[0])
        k = Q_atoi(c);
    else {
        // typed manually at the console, assume for unsticking, so clear all
        b->down[0] = b->down[1] = 0;
        b->state = 0;    // impulse up
        return;
    }

    if (b->down[0] == k)
        b->down[0] = 0;
    else if (b->down[1] == k)
        b->down[1] = 0;
    else
        return;        // key up without corresponding down (menu pass through)
    if (b->down[0] || b->down[1])
        return;        // some other key is still holding it down

    if (!(b->state & 1))
        return;        // still up (this should not happen)

    // save timestamp
    c = Cmd_Argv(2);
    uptime = Q_atoi(c);
    if (!uptime) {
        b->msec += 10;
    } else if (uptime > b->downtime) {
        b->msec += uptime - b->downtime;
    }

    b->state &= ~1;        // now up
}

static void KeyClear(kbutton_t *b)
{
    b->msec = 0;
    b->state &= ~2;        // clear impulses
    if (b->state & 1) {
        b->downtime = com_eventTime; // still down
    }
}

static void IN_KLookDown(void) { KeyDown(&in_klook); }
static void IN_KLookUp(void) { KeyUp(&in_klook); }
static void IN_UpDown(void) { KeyDown(&in_up); }
static void IN_UpUp(void) { KeyUp(&in_up); }
static void IN_DownDown(void) { KeyDown(&in_down); }
static void IN_DownUp(void) { KeyUp(&in_down); }
static void IN_LeftDown(void) { KeyDown(&in_left); }
static void IN_LeftUp(void) { KeyUp(&in_left); }
static void IN_RightDown(void) { KeyDown(&in_right); }
static void IN_RightUp(void) { KeyUp(&in_right); }
static void IN_ForwardDown(void) { KeyDown(&in_forward); }
static void IN_ForwardUp(void) { KeyUp(&in_forward); }
static void IN_BackDown(void) { KeyDown(&in_back); }
static void IN_BackUp(void) { KeyUp(&in_back); }
static void IN_LookupDown(void) { KeyDown(&in_lookup); }
static void IN_LookupUp(void) { KeyUp(&in_lookup); }
static void IN_LookdownDown(void) { KeyDown(&in_lookdown); }
static void IN_LookdownUp(void) { KeyUp(&in_lookdown); }
static void IN_MoveleftDown(void) { KeyDown(&in_moveleft); }
static void IN_MoveleftUp(void) { KeyUp(&in_moveleft); }
static void IN_MoverightDown(void) { KeyDown(&in_moveright); }
static void IN_MoverightUp(void) { KeyUp(&in_moveright); }
static void IN_SpeedDown(void) { KeyDown(&in_speed); }
static void IN_SpeedUp(void) { KeyUp(&in_speed); }
static void IN_StrafeDown(void) { KeyDown(&in_strafe); }
static void IN_StrafeUp(void) { KeyUp(&in_strafe); }

static void IN_AttackDown(void)
{
    KeyDown(&in_attack);

    if (cl_instantpacket->integer && cls.state == ca_active && !cls.demo.playback) {
        cl.sendPacketNow = true;
    }
}

static void IN_AttackUp(void)
{
    KeyUp(&in_attack);
}

static void IN_UseDown(void)
{
    KeyDown(&in_use);

    if (cl_instantpacket->integer && cls.state == ca_active && !cls.demo.playback) {
        cl.sendPacketNow = true;
    }
}

static void IN_UseUp(void)
{
    KeyUp(&in_use);
}

static void IN_Impulse(void)
{
    in_impulse = Q_atoi(Cmd_Argv(1));
}

static void IN_CenterView(void)
{
    cl.viewangles[PITCH] = -SHORT2ANGLE(cl.frame.ps.pmove.delta_angles[PITCH]);
}

static void IN_MLookDown(void)
{
    in_mlooking = true;
}

static void IN_MLookUp(void)
{
    in_mlooking = false;

    if (!freelook->integer && lookspring->integer)
        IN_CenterView();
}

/*
===============
CL_KeyState

Returns the fraction of the frame that the key was down
===============
*/
static float CL_KeyState(const kbutton_t *key)
{
    unsigned msec = key->msec;

    if (key->state & 1) {
        // still down
        if (com_eventTime > key->downtime) {
            msec += com_eventTime - key->downtime;
        }
    }

    // special case for instant packet
    if (!cl.cmd.msec) {
        return (float)(key->state & 1);
    }

    return Q_clipf((float)msec / cl.cmd.msec, 0, 1);
}

//==========================================================================

static float autosens_x;
static float autosens_y;

static void CL_ApplyMouseMotion(float mx, float my)
{
    float speed;

    if (!mx && !my) {
        return;
    }

    Cvar_ClampValue(m_accel, 0, 1);

    speed = sqrtf(mx * mx + my * my);
    speed = sensitivity->value + speed * m_accel->value;

    mx *= speed;
    my *= speed;

    if (m_autosens->integer) {
        mx *= cl.fov_x * autosens_x;
        my *= cl.fov_y * autosens_y;
    }

// add mouse X/Y movement
    if ((in_strafe.state & 1) || (lookstrafe->integer && !in_mlooking)) {
        cl.mousemove[1] += m_side->value * mx;
    } else {
        cl.viewangles[YAW] -= m_yaw->value * mx;
    }

    if ((in_mlooking || freelook->integer) && !(in_strafe.state & 1)) {
        cl.viewangles[PITCH] += m_pitch->value * my;
    } else {
        cl.mousemove[0] -= m_forward->value * my;
    }
}

/*
================
CL_MouseMove
================
*/
static void CL_MouseMove(void)
{
    int dx, dy;
    float mx, my;

    if (!vid || !vid->get_mouse_motion) {
        return;
    }
    if (cls.key_dest & (KEY_MENU | KEY_CONSOLE)) {
        return;
    }
    if (!vid->get_mouse_motion(&dx, &dy)) {
        return;
    }

    if (m_filter->integer) {
        mx = (dx + input.old_dx) * 0.5f;
        my = (dy + input.old_dy) * 0.5f;
    } else {
        mx = dx;
        my = dy;
    }

    input.old_dx = dx;
    input.old_dy = dy;

    CL_ApplyMouseMotion(mx, my);
}

#if USE_SDL
static void CL_GamepadLook(int msec)
{
    float dt, look_scale;
    float mx, my;
    float rx, ry;

    if (!in_joystick || !in_joystick->integer) {
        return;
    }

    if (!gamepad.controller) {
        return;
    }

    if (cls.active != ACT_ACTIVATED) {
        return;
    }

    if (cls.state != ca_active) {
        return;
    }

    if (cls.key_dest != KEY_GAME) {
        return;
    }

    dt = msec * 0.001f;
    if (dt <= 0.0f || dt > 0.25f) {
        dt = 0.016f;
    }

    look_scale = joy_look_scale ? joy_look_scale->value : 1800.0f;

    rx = gamepad.rx;
    ry = gamepad.ry;
    if (joy_invert_y && joy_invert_y->integer) {
        ry = -ry;
    }

    mx = rx * look_scale * dt;
    my = ry * look_scale * dt;

    CL_ApplyMouseMotion(mx, my);
}
#endif


/*
================
CL_AdjustAngles

Moves the local angle positions
================
*/
static void CL_AdjustAngles(int msec)
{
    float speed;

    if (in_speed.state & 1)
        speed = msec * cl_anglespeedkey->value * 0.001f;
    else
        speed = msec * 0.001f;

    if (!(in_strafe.state & 1)) {
        cl.viewangles[YAW] -= speed * cl_yawspeed->value * CL_KeyState(&in_right);
        cl.viewangles[YAW] += speed * cl_yawspeed->value * CL_KeyState(&in_left);
    }
    if (in_klook.state & 1) {
        cl.viewangles[PITCH] -= speed * cl_pitchspeed->value * CL_KeyState(&in_forward);
        cl.viewangles[PITCH] += speed * cl_pitchspeed->value * CL_KeyState(&in_back);
    }

    cl.viewangles[PITCH] -= speed * cl_pitchspeed->value * CL_KeyState(&in_lookup);
    cl.viewangles[PITCH] += speed * cl_pitchspeed->value * CL_KeyState(&in_lookdown);
}

/*
================
CL_BaseMove

Build the intended movement vector
================
*/
static void CL_BaseMove(vec3_t move)
{
    if (in_strafe.state & 1) {
        move[1] += cl_sidespeed->value * CL_KeyState(&in_right);
        move[1] -= cl_sidespeed->value * CL_KeyState(&in_left);
    }

    move[1] += cl_sidespeed->value * CL_KeyState(&in_moveright);
    move[1] -= cl_sidespeed->value * CL_KeyState(&in_moveleft);

    move[2] += cl_upspeed->value * CL_KeyState(&in_up);
    move[2] -= cl_upspeed->value * CL_KeyState(&in_down);

    if (!(in_klook.state & 1)) {
        move[0] += cl_forwardspeed->value * CL_KeyState(&in_forward);
        move[0] -= cl_forwardspeed->value * CL_KeyState(&in_back);
    }

#if USE_SDL
    gamepad_add_move(move);
#endif

// adjust for speed key / running
    if ((in_speed.state & 1) ^ cl_run->integer) {
        VectorScale(move, 2, move);
    }
}

static void CL_ClampSpeed(vec3_t move)
{
    const float speed = 400;    // default (maximum) running speed

    move[0] = Q_clipf(move[0], -speed, speed);
    move[1] = Q_clipf(move[1], -speed, speed);
    move[2] = Q_clipf(move[2], -speed, speed);
}

static void CL_ClampPitch(void)
{
    float pitch, angle;

    pitch = SHORT2ANGLE(cl.frame.ps.pmove.delta_angles[PITCH]);
    angle = cl.viewangles[PITCH] + pitch;

    if (angle < -180)
        angle += 360; // wrapped
    if (angle > 180)
        angle -= 360; // wrapped

    angle = Q_clipf(angle, -89, 89);
    cl.viewangles[PITCH] = angle - pitch;
}

/*
=================
CL_UpdateCmd

Updates msec, angles and builds interpolated movement vector for local prediction.
Doesn't touch command forward/side/upmove, these are filled by CL_FinalizeCmd.
=================
*/
void CL_UpdateCmd(int msec)
{
    VectorClear(cl.localmove);

    if (sv_paused->integer) {
        return;
    }

    // add to milliseconds of time to apply the move
    cl.cmd.msec += msec;

    // adjust viewangles
    CL_AdjustAngles(msec);

    // get basic movement from keyboard
    CL_BaseMove(cl.localmove);

    // allow mice to add to the move
    CL_MouseMove();

#if USE_SDL
    CL_GamepadLook(msec);
#endif

    // add accumulated mouse forward/side movement
    cl.localmove[0] += cl.mousemove[0];
    cl.localmove[1] += cl.mousemove[1];

    // clamp to server defined max speed
    CL_ClampSpeed(cl.localmove);

    CL_ClampPitch();

    cl.cmd.angles[0] = ANGLE2SHORT(cl.viewangles[0]);
    cl.cmd.angles[1] = ANGLE2SHORT(cl.viewangles[1]);
    cl.cmd.angles[2] = ANGLE2SHORT(cl.viewangles[2]);
}

static void m_autosens_changed(cvar_t *self)
{
    float fov;

    if (self->value > 90.0f && self->value <= 179.0f)
        fov = self->value;
    else
        fov = 90.0f;

    autosens_x = 1.0f / fov;
    autosens_y = 1.0f / V_CalcFov(fov, 4, 3);
}

static const cmdreg_t c_input[] = {
    { "centerview", IN_CenterView },
    { "+moveup", IN_UpDown },
    { "-moveup", IN_UpUp },
    { "+movedown", IN_DownDown },
    { "-movedown", IN_DownUp },
    { "+left", IN_LeftDown },
    { "-left", IN_LeftUp },
    { "+right", IN_RightDown },
    { "-right", IN_RightUp },
    { "+forward", IN_ForwardDown },
    { "-forward", IN_ForwardUp },
    { "+back", IN_BackDown },
    { "-back", IN_BackUp },
    { "+lookup", IN_LookupDown },
    { "-lookup", IN_LookupUp },
    { "+lookdown", IN_LookdownDown },
    { "-lookdown", IN_LookdownUp },
    { "+strafe", IN_StrafeDown },
    { "-strafe", IN_StrafeUp },
    { "+moveleft", IN_MoveleftDown },
    { "-moveleft", IN_MoveleftUp },
    { "+moveright", IN_MoverightDown },
    { "-moveright", IN_MoverightUp },
    { "+speed", IN_SpeedDown },
    { "-speed", IN_SpeedUp },
    { "+attack", IN_AttackDown },
    { "-attack", IN_AttackUp },
    { "+use", IN_UseDown },
    { "-use", IN_UseUp },
    { "impulse", IN_Impulse },
    { "+klook", IN_KLookDown },
    { "-klook", IN_KLookUp },
    { "+mlook", IN_MLookDown },
    { "-mlook", IN_MLookUp },
    { "in_restart", IN_Restart_f },
#if USE_SDL
    { "joy_bind_defaults", joy_bind_defaults_f },
    { "joy_rumble", joy_rumble_f },
    { "joy_unbindall", joy_unbindall_f },
#endif
    { NULL }
};

/*
============
CL_RegisterInput
============
*/
void CL_RegisterInput(void)
{
    Cmd_Register(c_input);

    cl_nodelta = Cvar_Get("cl_nodelta", "0", 0);
    cl_maxpackets = Cvar_Get("cl_maxpackets", "30", 0);
    cl_fuzzhack = Cvar_Get("cl_fuzzhack", "0", 0);
    cl_packetdup = Cvar_Get("cl_packetdup", "1", 0);
#if USE_DEBUG
    cl_showpackets = Cvar_Get("cl_showpackets", "0", 0);
#endif
    cl_instantpacket = Cvar_Get("cl_instantpacket", "1", 0);
    cl_batchcmds = Cvar_Get("cl_batchcmds", "1", 0);

    cl_upspeed = Cvar_Get("cl_upspeed", "200", 0);
    cl_forwardspeed = Cvar_Get("cl_forwardspeed", "200", 0);
    cl_sidespeed = Cvar_Get("cl_sidespeed", "200", 0);
    cl_yawspeed = Cvar_Get("cl_yawspeed", "140", 0);
    cl_pitchspeed = Cvar_Get("cl_pitchspeed", "150", CVAR_CHEAT);
    cl_anglespeedkey = Cvar_Get("cl_anglespeedkey", "1.5", CVAR_CHEAT);
    cl_run = Cvar_Get("cl_run", "1", CVAR_ARCHIVE);

    freelook = Cvar_Get("freelook", "1", CVAR_ARCHIVE);
    lookspring = Cvar_Get("lookspring", "0", CVAR_ARCHIVE);
    lookstrafe = Cvar_Get("lookstrafe", "0", CVAR_ARCHIVE);
    sensitivity = Cvar_Get("sensitivity", "3", CVAR_ARCHIVE);

    m_pitch = Cvar_Get("m_pitch", "0.022", CVAR_ARCHIVE);
    m_yaw = Cvar_Get("m_yaw", "0.022", 0);
    m_forward = Cvar_Get("m_forward", "1", 0);
    m_side = Cvar_Get("m_side", "1", 0);
    m_filter = Cvar_Get("m_filter", "0", 0);
    m_accel = Cvar_Get("m_accel", "0", 0);
    m_autosens = Cvar_Get("m_autosens", "0", 0);
    m_autosens->changed = m_autosens_changed;
    m_autosens_changed(m_autosens);
}

/*
=================
CL_FinalizeCmd

Builds the actual movement vector for sending to server. Assumes that msec
and angles are already set for this frame by CL_UpdateCmd.
=================
*/
void CL_FinalizeCmd(void)
{
    vec3_t move;

    // command buffer ticks in sync with cl_maxfps
    Cbuf_Frame(&cmd_buffer);
    Cbuf_Frame(&cl_cmdbuf);

    if (cls.state != ca_active) {
        goto clear; // not talking to a server
    }

    if (sv_paused->integer) {
        goto clear;
    }

//
// figure button bits
//
    if (in_attack.state & 3)
        cl.cmd.buttons |= BUTTON_ATTACK;
    if (in_use.state & 3)
        cl.cmd.buttons |= BUTTON_USE;

    if (cls.key_dest == KEY_GAME && Key_AnyKeyDown()) {
        cl.cmd.buttons |= BUTTON_ANY;
    }

    if (cl.cmd.msec > 250) {
        cl.cmd.msec = 100;        // time was unreasonable
    }

    // rebuild the movement vector
    VectorClear(move);

    // get basic movement from keyboard
    CL_BaseMove(move);

    // add mouse forward/side movement
    move[0] += cl.mousemove[0];
    move[1] += cl.mousemove[1];

    // clamp to server defined max speed
    CL_ClampSpeed(move);

    // store the movement vector
    cl.cmd.forwardmove = move[0];
    cl.cmd.sidemove = move[1];
    cl.cmd.upmove = move[2];

    cl.cmd.impulse = in_impulse;

    // save this command off for prediction
    cl.cmdNumber++;
    cl.cmds[cl.cmdNumber & CMD_MASK] = cl.cmd;

clear:
    // clear pending cmd
    memset(&cl.cmd, 0, sizeof(cl.cmd));

    // clear all states
    cl.mousemove[0] = 0;
    cl.mousemove[1] = 0;

    in_attack.state &= ~2;
    in_use.state &= ~2;

    KeyClear(&in_right);
    KeyClear(&in_left);

    KeyClear(&in_moveright);
    KeyClear(&in_moveleft);

    KeyClear(&in_up);
    KeyClear(&in_down);

    KeyClear(&in_forward);
    KeyClear(&in_back);

    KeyClear(&in_lookup);
    KeyClear(&in_lookdown);

    in_impulse = 0;
}

static inline bool ready_to_send(void)
{
    unsigned msec;

    if (cl.sendPacketNow) {
        return true;
    }
    if (cls.netchan.message.cursize || cls.netchan.reliable_ack_pending) {
        return true;
    }
    if (!cl_maxpackets->integer) {
        return true;
    }

    if (cl_maxpackets->integer < 10) {
        Cvar_Set("cl_maxpackets", "10");
    }

    msec = 1000 / cl_maxpackets->integer;
    if (msec) {
        msec = 100 / (100 / msec);
    }
    if (cls.realtime - cl.lastTransmitTime < msec) {
        return false;
    }

    return true;
}

static inline bool ready_to_send_hacked(void)
{
    if (!cl_fuzzhack->integer) {
        return true; // packet drop hack disabled
    }

    if (cl.cmdNumber - cl.lastTransmitCmdNumberReal > 2) {
        return true; // can't drop more than 2 cmds
    }

    return ready_to_send();
}

/*
=================
CL_SendDefaultCmd
=================
*/
static void CL_SendDefaultCmd(void)
{
    int cursize q_unused;
    uint32_t checksumIndex;
    usercmd_t *cmd, *oldcmd;
    client_history_t *history;
    int version;

    // archive this packet
    history = &cl.history[cls.netchan.outgoing_sequence & CMD_MASK];
    history->cmdNumber = cl.cmdNumber;
    history->sent = cls.realtime;    // for ping calculation
    history->rcvd = 0;

    cl.lastTransmitCmdNumber = cl.cmdNumber;

    // see if we are ready to send this packet
    if (!ready_to_send_hacked()) {
        cls.netchan.outgoing_sequence++; // just drop the packet
        return;
    }

    cl.lastTransmitTime = cls.realtime;
    cl.lastTransmitCmdNumberReal = cl.cmdNumber;

    // begin a client move command
    MSG_WriteByte(clc_move);

    // save the position for a checksum byte
    checksumIndex = 0;
    version = 0;
    if (cls.serverProtocol <= PROTOCOL_VERSION_DEFAULT) {
        checksumIndex = msg_write.cursize;
        SZ_GetSpace(&msg_write, 1);
    } else if (cls.serverProtocol == PROTOCOL_VERSION_R1Q2) {
        version = cls.protocolVersion;
    }

    // let the server know what the last frame we
    // got was, so the next message can be delta compressed
    if (cl_nodelta->integer || !cl.frame.valid /*|| cls.demowaiting*/) {
        MSG_WriteLong(-1);   // no compression
    } else {
        MSG_WriteLong(cl.frame.number);
    }

    // send this and the previous cmds in the message, so
    // if the last packet was dropped, it can be recovered
    cmd = &cl.cmds[(cl.cmdNumber - 2) & CMD_MASK];
    MSG_WriteDeltaUsercmd(NULL, cmd, version);
    MSG_WriteByte(cl.lightlevel);
    oldcmd = cmd;

    cmd = &cl.cmds[(cl.cmdNumber - 1) & CMD_MASK];
    MSG_WriteDeltaUsercmd(oldcmd, cmd, version);
    MSG_WriteByte(cl.lightlevel);
    oldcmd = cmd;

    cmd = &cl.cmds[cl.cmdNumber & CMD_MASK];
    MSG_WriteDeltaUsercmd(oldcmd, cmd, version);
    MSG_WriteByte(cl.lightlevel);

    if (cls.serverProtocol <= PROTOCOL_VERSION_DEFAULT) {
        // calculate a checksum over the move commands
        msg_write.data[checksumIndex] = COM_BlockSequenceCRCByte(
            msg_write.data + checksumIndex + 1,
            msg_write.cursize - checksumIndex - 1,
            cls.netchan.outgoing_sequence);
    }

    P_FRAMES++;

    //
    // deliver the message
    //
    cursize = Netchan_Transmit(&cls.netchan, msg_write.cursize, msg_write.data, 1);
#if USE_DEBUG
    if (cl_showpackets->integer) {
        Com_Printf("%i ", cursize);
    }
#endif

    SZ_Clear(&msg_write);
}

/*
=================
CL_SendBatchedCmd
=================
*/
static void CL_SendBatchedCmd(void)
{
    int i, j, seq, numCmds, numDups;
    q_unused int totalCmds, totalMsec, cursize, bits;
    usercmd_t *cmd, *oldcmd;
    client_history_t *history, *oldest;
    byte *patch;

    // see if we are ready to send this packet
    if (!ready_to_send()) {
        return;
    }

    // archive this packet
    seq = cls.netchan.outgoing_sequence;
    history = &cl.history[seq & CMD_MASK];
    history->cmdNumber = cl.cmdNumber;
    history->sent = cls.realtime;    // for ping calculation
    history->rcvd = 0;

    cl.lastTransmitTime = cls.realtime;
    cl.lastTransmitCmdNumber = cl.cmdNumber;
    cl.lastTransmitCmdNumberReal = cl.cmdNumber;

    MSG_BeginWriting();

    // begin a client move command
    patch = SZ_GetSpace(&msg_write, 1);

    // let the server know what the last frame we
    // got was, so the next message can be delta compressed
    if (cl_nodelta->integer || !cl.frame.valid /*|| cls.demowaiting*/) {
        *patch = clc_move_nodelta; // no compression
    } else {
        *patch = clc_move_batched;
        MSG_WriteLong(cl.frame.number);
    }

    Cvar_ClampInteger(cl_packetdup, 0, MAX_PACKET_FRAMES - 1);
    numDups = cl_packetdup->integer;

    *patch |= numDups << SVCMD_BITS;

    // send lightlevel
    MSG_WriteByte(cl.lightlevel);

    // send this and the previous cmds in the message, so
    // if the last packet was dropped, it can be recovered
    oldcmd = NULL;
    totalCmds = 0;
    totalMsec = 0;
    for (i = seq - numDups; i <= seq; i++) {
        oldest = &cl.history[(i - 1) & CMD_MASK];
        history = &cl.history[i & CMD_MASK];

        numCmds = history->cmdNumber - oldest->cmdNumber;
        if (numCmds >= MAX_PACKET_USERCMDS) {
            Com_WPrintf("%s: MAX_PACKET_USERCMDS exceeded\n", __func__);
            MSG_BeginWriting();
            break;
        }
        totalCmds += numCmds;
        MSG_WriteBits(numCmds, 5);
        for (j = oldest->cmdNumber + 1; j <= history->cmdNumber; j++) {
            cmd = &cl.cmds[j & CMD_MASK];
            totalMsec += cmd->msec;
            bits = MSG_WriteDeltaUsercmd_Enhanced(oldcmd, cmd);
#if USE_DEBUG
            if (cl_showpackets->integer == 3) {
                MSG_ShowDeltaUsercmdBits_Enhanced(bits);
            }
#endif
            oldcmd = cmd;
        }
    }

    MSG_FlushBits();

    P_FRAMES++;

    //
    // deliver the message
    //
    cursize = Netchan_Transmit(&cls.netchan, msg_write.cursize, msg_write.data, 1);
#if USE_DEBUG
    if (cl_showpackets->integer == 1) {
        Com_Printf("%i(%i) ", cursize, totalCmds);
    } else if (cl_showpackets->integer == 2) {
        Com_Printf("%i(%i) ", cursize, totalMsec);
    } else if (cl_showpackets->integer == 3) {
        Com_Printf(" | ");
    }
#endif

    SZ_Clear(&msg_write);
}

static void CL_SendKeepAlive(void)
{
    client_history_t *history;
    int cursize q_unused;

    // archive this packet
    history = &cl.history[cls.netchan.outgoing_sequence & CMD_MASK];
    history->cmdNumber = cl.cmdNumber;
    history->sent = cls.realtime;    // for ping calculation
    history->rcvd = 0;

    cl.lastTransmitTime = cls.realtime;
    cl.lastTransmitCmdNumber = cl.cmdNumber;
    cl.lastTransmitCmdNumberReal = cl.cmdNumber;

    cursize = Netchan_Transmit(&cls.netchan, 0, NULL, 1);
#if USE_DEBUG
    if (cl_showpackets->integer) {
        Com_Printf("%i ", cursize);
    }
#endif
}

static void CL_SendUserinfo(void)
{
    char userinfo[MAX_INFO_STRING];
    cvar_t *var;
    int i;

    if (cls.userinfo_modified == MAX_PACKET_USERINFOS) {
        size_t len = Cvar_BitInfo(userinfo, CVAR_USERINFO);
        Com_DDPrintf("%s: %u: full update\n", __func__, com_framenum);
        MSG_WriteByte(clc_userinfo);
        MSG_WriteData(userinfo, len + 1);
        MSG_FlushTo(&cls.netchan.message);
    } else if (cls.serverProtocol == PROTOCOL_VERSION_Q2PRO) {
        Com_DDPrintf("%s: %u: %d updates\n", __func__, com_framenum,
                     cls.userinfo_modified);
        for (i = 0; i < cls.userinfo_modified; i++) {
            var = cls.userinfo_updates[i];
            MSG_WriteByte(clc_userinfo_delta);
            MSG_WriteString(var->name);
            if (var->flags & CVAR_USERINFO) {
                MSG_WriteString(var->string);
            } else {
                // no longer in userinfo
                MSG_WriteString(NULL);
            }
        }
        MSG_FlushTo(&cls.netchan.message);
    } else {
        Com_WPrintf("%s: update count is %d, should never happen.\n",
                    __func__, cls.userinfo_modified);
    }
}

static void CL_SendReliable(void)
{
    if (Netchan_SeqTooBig(&cls.netchan)) {
        Com_Error(ERR_DROP, "Outgoing sequence too big");
    }

    if (cls.userinfo_modified) {
        CL_SendUserinfo();
        cls.userinfo_modified = 0;
    }

    if (cls.netchan.message.overflowed) {
        SZ_Clear(&cls.netchan.message);
        Com_Error(ERR_DROP, "Reliable message overflowed");
    }
}

void CL_SendCmd(void)
{
    if (cls.state < ca_connected) {
        return; // not talking to a server
    }

    // generate usercmds while playing a demo, but do not send them
    if (cls.demo.playback) {
        return;
    }

    if (cls.state != ca_active || sv_paused->integer) {
        // send a userinfo update if needed
        CL_SendReliable();

        // just keepalive or update reliable
        if (Netchan_ShouldUpdate(&cls.netchan)) {
            CL_SendKeepAlive();
        }

        cl.sendPacketNow = false;
        return;
    }

    // are there any new usercmds to send after all?
    if (cl.lastTransmitCmdNumber == cl.cmdNumber) {
        return; // nothing to send
    }

    // send a userinfo update if needed
    CL_SendReliable();

    if (cls.serverProtocol == PROTOCOL_VERSION_Q2PRO && cl_batchcmds->integer) {
        CL_SendBatchedCmd();
    } else {
        CL_SendDefaultCmd();
    }

    cl.sendPacketNow = false;
}
