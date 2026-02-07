Q2PRO
=====

Q2PRO is an enhanced Quake 2 client and server for Windows and Linux. Supported
features include:

* unified OpenGL renderer with support for wide range of OpenGL versions
* enhanced console with persistent command history and autocompletion
* rendering / physics / packet rate separation
* syncing to GPU for reduced input lag with vsync on
* ZIP packfiles (.pkz)
* JPEG/PNG textures and screenshots
* MD3 and MD5 (re-release) models
* Ogg Vorbis music and Ogg Theora cinematics
* compatibility with re-release assets
* fast and secure HTTP downloads
* multichannel sound using OpenAL
* SDL2 gamepad support (analog movement/look, menu navigation, binds, rumble)
* stereo WAV files support
* seeking in demos, recording from demos, server side multiview demos
* live game broadcasting capabilities
* network protocol extensions for larger maps
* eliminates frame overflows (even for legacy clients)
* won't crash if game data is corrupted

Gamepad Quick Start
-------------------

1. Build with SDL2 enabled (`-Dsdl2=enabled`).
2. On Windows, place `SDL2.dll` next to `q2pro.exe`.
3. In the in-game console:

       set in_joystick_auto 1
       set joy_autobind 1
       joy_bind_defaults
       in_restart

4. Optional rumble test:

       joy_rumble 32767 32767 200

See `doc/client.asciidoc` for full gamepad cvars/commands and bind keys.

Recent Changes
--------------

* Added SDL2 GameController input path for modern gamepads.
* Added analog movement/look and menu navigation with controller input.
* Added trigger-to-digital key support (`AUX27`/`AUX28`) and default bind helper commands.
* Added rumble support (`joy_rumble`, gameplay rumble controls).
* Added SDL2-focused CI/release workflows that package client, dedicated server and required runtime files.

Windows, Linux and FreeBSD binaries are published in GitHub Releases for tagged builds.
Other platforms are advised to build from source.
See BUILDING.md file for instructions.

For information on using and configuring Q2PRO, refer to client and server
manuals available in doc/ subdirectory.
