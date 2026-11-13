# libfpv in a web browser

A game whose source code is available can carry its libfpv mod onto the
web: the game, the mod and libfpv are compiled together into one
WebAssembly module, served as a few static files. The player opens a link
and flies. There is nothing to download, install or extract, and no plugin.
The radio, keyboard, overlay, OSD, sound and saved settings all work in
the page.

This guide is for the developer of such a mod. The reference is
`examples/standalone`, which builds into a page with
`emcmake cmake -S . -B build/web && cmake --build build/web`.

## How it works, and why it is fast

| | Desktop | Web |
|---|---|---|
| Code | native `libfpv.so`/`.dll` | WebAssembly, compiled from the same C/C++ with `-O3` and linked statically into the game's module |
| Simulation | own 1 kHz thread | the same 1 kHz fixed-step loop, run inside `fpv_drone_tick` and `fpv_overlay_render` on the page's thread |
| Radios and gamepads | embedded SDL3 | the browser's Gamepad API |
| Settings, EEPROMs, radio mappings | files in `data_dir` | files in `data_dir`, kept in the browser's IndexedDB |
| C++ exceptions | native | WebAssembly exception instructions (`-fwasm-exceptions`) |
| Betaflight Configurator | `ws://localhost:5761` | not available (see [Limits](#limits)) |

WebAssembly runs the flight controller and the physics at native speed:
the test suite takes as long on Node as natively, and one simulated
millisecond costs about 23 µs either way on the same desktop CPU. The
browser build has no threads, for these reasons:

- **It works everywhere.** WebAssembly threads need `SharedArrayBuffer`,
  which browsers only give to pages served with two special headers
  (COOP and COEP). GitHub Pages, itch.io and most simple hosts cannot send
  them. A threadless page runs from any static host, and in every current
  browser.
- **It costs little.** One simulated millisecond (the physics plus eight
  Betaflight PID loops) takes about 25 µs, so at 60 frames per second the
  simulation uses about 0.4 ms of each 16.7 ms frame.
- **Nothing is lost.** Browsers only update gamepads once per frame and only
  on the page's main thread, so a separate thread could not read the radio
  any faster.

libfpv still steps Betaflight and the physics at 1 kHz, in fixed steps
whatever the frame rate. Each frame it runs the steps that real time has
reached, and draws one step in the past with interpolation, exactly as on
the desktop. If a frame takes too long it never catches up more than 50 ms,
and when the tab is hidden the drone pauses (auto-pause).

## Requirements

- **Emscripten** 4.0 or newer (tested with 6.0.12):
  ```sh
  git clone https://github.com/emscripten-core/emsdk.git && cd emsdk
  ./emsdk install latest && ./emsdk activate latest && source ./emsdk_env.sh
  ```
- **The game must already run in a browser**, which usually means an
  Emscripten port of it: its main loop returns to the browser after each
  frame (`emscripten_set_main_loop`, SDL's main callbacks, or Asyncify), and
  it draws with WebGL 2 (OpenGL ES 3.0). Many open-source game ports have a
  web target already. libfpv adds nothing to this list.
- **Players' browsers**: any current Chrome, Edge, Firefox or Safari, with
  WebGL 2 and WebAssembly exceptions (Chrome 95, Firefox 100, Safari 15.2 or
  newer).

## Building libfpv

```sh
source /path/to/emsdk/emsdk_env.sh
emcmake cmake -S . -B build/web
cmake --build build/web -j
```

| Output in `build/web` | What it is |
|---|---|
| `libfpv.a` | the library: one object with libfpv, Betaflight and Dear ImGui |
| `web/` | `examples/standalone` as a page: `index.html`, `index.js`, `index.wasm` |
| `tests/fpv_tests.js` | the test suite, run with `node fpv_tests.js` |

`libfpv.a` holds a single object, so it links like any static library: no
`--whole-archive`, and every Betaflight parameter group comes with it.
Inside it, Betaflight, Dear ImGui and fmt are local, as they are hidden in
the desktop library, so they never clash with the game's own symbols (a C
game's `init()` or `delay()`, or a game that has its own Dear ImGui). Only
the `fpv_*` API is visible.

## Porting a mod

### 1. Link libfpv

With CMake, link `libfpv::fpv` as on the desktop. The flags below come with
it.

Otherwise (a Makefile, for example), add libfpv's header folder to the
include path, and add these to the game's final link:

```
path/to/libfpv/build/web/libfpv.a
-fwasm-exceptions
-lidbfs.js
--js-library path/to/libfpv/src/web/libfpv_web.js
-sDEFAULT_TO_CXX          (only when the game links with emcc, not em++)
```

- `-fwasm-exceptions`: libfpv uses C++ exceptions while it parses JSON.
  Without this flag a damaged settings file would abort the game instead
  of being ignored. The game's own code does not have to be compiled with
  it.
- `-lidbfs.js` and `libfpv_web.js`: the saved settings (step 3) and the
  overlay's clipboard.
- `-sDEFAULT_TO_CXX`: libfpv is C++, so the link needs the C++ runtime. A
  C game (a decompilation, for example) usually links with `emcc`, which
  leaves it out. The desktop library carries its own; a static one cannot.

Also recommended for the game's link: `-O3`, `-sALLOW_MEMORY_GROWTH=1`
(collision meshes of large levels), and `-sMIN_WEBGL_VERSION=2`.

### 2. Draw with WebGL 2

Draw the OSD and the overlay with `docs/renderers/fpv_render_gl3.h`, and
define `FPV_GL3_GLES` before including it: OpenGL ES 3.0 and WebGL 2 have
no base-vertex draws and need the `#version 300 es` shaders. The same
define makes `docs/renderers/fpv_video_gl3.h` (the lens and the video feed,
INTEGRATION.md "The video feed") work in WebGL 2; a game that renders with
WebGL 1 or GLES 2 shaders uses `fpv_video_gl2.h` with `FPV_VIDEO_GL2_GLES`. The lens distortion
shader in INTEGRATION.md works as it is after a `#version 300 es` /
`precision highp float;` header.

### 3. Use the browser's data folder

```c
cfg.data_dir = FPV_WEB_DATA_DIR;   /* "/libfpv" */
```

Before `main()` runs, libfpv's runtime mounts this folder from the
browser's IndexedDB and loads it. Every file libfpv saves there afterwards
(settings, a quad's Betaflight EEPROM, radio mappings) is written back at
once, with no save call from the game. To keep libfpv's files elsewhere, set
`Module.libfpvDataDir` in the page before the game's script loads and pass
the same path. libfpv logs a warning when `data_dir` is not inside the kept
folder, because the settings would then be lost on reload.

### 4. Call libfpv every frame

`FPV_TIMING_THREAD` (the default) works unchanged: on the web it runs the
simulation inside `fpv_drone_tick` and `fpv_overlay_render`. Call them every
frame on the page's main thread, which is where an Emscripten port runs
its frame anyway. A game that stops calling them (a pause menu, a loading
screen) pauses the drone, just like the desktop auto-pause.
`FPV_TIMING_MANUAL` works as on the desktop.

### 5. Sound, keys and the mouse

Nothing changes: forward the same events as on the desktop.

- **Sound**: call `fpv_audio_read` from the game's audio callback. With
  SDL's browser audio this callback runs on the main thread, which is fine.
  Browsers only start sound after the player clicked or pressed a key; SDL
  takes care of that.
- **Keys**: F8 (overlay), the keyboard fallback (W/S, A/D, the arrow keys,
  Enter, T) and the game's own keys all reach the page. Browsers keep a few
  keys for themselves: F11 (the browser's own fullscreen, which also works
  for the game when its canvas fills the page), F5, F12, Ctrl+W, Ctrl+T and
  Ctrl+N. Esc leaves fullscreen and mouse capture, so do not make it quit.
- **Pasting** a `diff all` into the overlay: Ctrl+V in a text field, or the
  Paste button in F8 > Import & CLI. The browser asks the player once
  whether the page may read the clipboard. Copy buttons write to the
  system clipboard.

### 6. Radios and gamepads

libfpv reads them through the browser's Gamepad API, with the same radio
detection and setup wizard as on the desktop. A radio in USB joystick mode
(EdgeTX, OpenTX, ELRS handsets) is a generic HID joystick to the browser,
just as it is to the operating system. Two browser rules matter to
players, so put them on the page:

- A browser lists a device only after a button was pressed or a stick
  moved on it while the page was open. "Move a stick on your radio" is
  enough.
- The page reads devices once per frame (60 to 144 times a second) instead
  of 1000. Betaflight and the physics still run at 1 kHz.

A radio's mapping is saved per browser and device, so the wizard runs once.

## Deploying

The build leaves a folder of static files; for `examples/standalone` that is
`build/web/web/`. Upload it to any web host. There is no server code and
nothing else to install for players:

- **GitHub Pages**: see the workflow in PUBLISHING.md, [Web releases](PUBLISHING.md#web-releases).
- **itch.io**: zip the folder's contents (with `index.html` at the top),
  create an HTML project and tick "This file will be played in the browser".
- **Netlify, Cloudflare Pages, a plain nginx**: copy the folder.

To have the page start from your own Betaflight setup instead of libfpv's,
configure with `-DLIBFPV_STANDALONE_BF_CONFIG=<diff all file>` (see
`assets/configs`). The file is embedded in the page and every drone's
configuration starts from it (`fpv_set_default_betaflight_diff`), also
after a reset in F8 > Drone; players' later changes are kept.

The host must serve `.wasm` files as `application/wasm` (all of the above
do) so the browser compiles the module while it downloads. Hosts compress
the files automatically. To try a build locally:

```sh
python3 -m http.server -d build/web/web 8080   # then open http://localhost:8080
```

The page has to come from a web server: browsers do not run WebAssembly
from a `file://` URL.

## Limits

- **Betaflight Configurator cannot connect.** It talks to the quad over a
  WebSocket that the game has to accept, and a web page cannot accept
  connections. `fpv_configurator_get_info` reports it disabled, and the
  overlay's Configurator tab tells the player where to turn instead: every
  setting the overlay shows is Betaflight's own, the Betaflight CLI is in
  F8 > Import & CLI, and a real quad's `diff all` can be pasted there.
- **Saved settings belong to one browser.** Clearing the site's data (or a
  private window) starts from defaults.
- **Players pick the radio up by moving it**, as described above.

## Testing

```sh
node build/web/tests/fpv_tests.js
```

runs the whole test suite on WebAssembly (use Node 18 or newer; emsdk ships
one in `emsdk/node/`). Every test that passes on the desktop passes here:
flight physics, Betaflight imports, GPS, OSD and the real-time loop.
