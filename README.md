<p><img src="docs/img/libfpv-logo.svg" alt="libfpv" width=250></p>

# FPV Drone in Any Game

<p><a href="https://github.com/hammerill/sm64ex-fpv"><img src="https://raw.githubusercontent.com/hammerill/sm64ex-fpv/nightly/sm64-fpv.gif" alt="SM64 FPV" width=512></a></p>

*(example with [sm64ex-fpv](https://github.com/hammerill/sm64ex-fpv))*

<p><a href="https://github.com/hammerill/gtaiv-fpv"><img src="https://raw.githubusercontent.com/hammerill/gtaiv-fpv/main/gtaiv-fpv.gif" alt="GTA IV FPV" width=512></a></p>

*(another one with [gtaiv-fpv](https://github.com/hammerill/gtaiv-fpv))*

<p><a href="https://github.com/hammerill/gtasa-fpv"><img src="https://raw.githubusercontent.com/hammerill/gtasa-fpv/main/gtasa-fpv.gif" alt="GTA SA FPV" width=512></a></p>

*(yet another with [gtasa-fpv](https://github.com/hammerill/gtasa-fpv))*

<p><a href="https://github.com/hammerill/reVC-mac-fpv"><img src="https://raw.githubusercontent.com/hammerill/reVC-mac-fpv/main/revc-mac-fpv.gif" alt="reVC FPV" width=512></a></p>

*([reVC-mac-fpv](https://github.com/hammerill/reVC-mac-fpv))*

A drop-in FPV quadcopter for any game. libfpv runs real Betaflight firmware
(through [SimITL](https://github.com/AJ92/SimITL)) on its own 1 kHz
simulation thread, reads the player's radio or gamepad, and gives the game
everything it needs to draw one rendered frame: the drone's pose, the FPV,
chase and line-of-sight cameras, motor RPMs, impacts, a Betaflight OSD, a
settings overlay and motor sound.

The game supplies collision geometry and draws the result. libfpv has no
renderer of its own, and its API is plain C, so it can be loaded from any
engine or mod loader.

## Features

- **Betaflight in the loop.** The firmware is real Betaflight 2025.12, so
  rates, modes, filters and arming behave as they do on a real quad. Betaflight
  Configurator can connect over WebSocket (`ws://localhost:5761`). The Betaflight
  CLI is available in-process, and `diff all` from a real quad can be imported.
- **Physics presets.** Frame, motor, propeller, battery, VTX and camera are
  set in JSON presets (`assets/presets/`), with a separate EEPROM for each quad.
- **GPS.** A simulated GPS and barometer drive Betaflight's speed, home
  arrow and distance, altitude, vario and distance-flown elements. The game
  can set where home is with `fpv_gps_set_home`.
- **Collisions.** The game provides static meshes, moving objects or a
  raycast callback, and libfpv reports impacts and prop damage.
- **Wind.** The game gives its weather's wind, and the drone flies in it:
  stronger higher up, gusty, calm in the lee of buildings and indoors,
  with helicopter downwash and blast waves. Betaflight has to hold the quad
  against it. Players can turn it off.
- **Input.** Radios and gamepads are read through an embedded, hidden SDL3,
  with a radio setup wizard (arm, angle, beeper, turtle, air mode and OSD
  disable switches).
  Keyboard and gamepad fallbacks are included.
- **Presentation.** The OSD uses Betaflight's MAX7456 fonts (analog or HD
  look). A Dear ImGui overlay opens on F8, and synthesized motor sound comes
  through `fpv_audio_read`. Both the OSD and the overlay are returned as
  textured triangles, so one small renderer draws both.
- **Camera.** Uptilt and FOV are adjustable, and there is a fisheye lens
  (on by default). Analog video noise grows with range and with obstacles in
  the way.
- **Web builds.** A game with source code available can ship its mod as a
  web page. The game and libfpv compile to WebAssembly, and players fly from
  a link with nothing to install. Radio, keys, overlay, OSD, sound and saved
  settings all work in the browser ([docs/WEB.md](docs/WEB.md)).

## Building

Requirements: CMake 3.24 or newer, a C11/C++17 compiler (GCC or Clang), and
the usual X11/Wayland and audio development headers for SDL3 on Linux. On
macOS the Xcode command line tools are enough.

```sh
git clone --recursive https://github.com/hammerill/libfpv.git
cd libfpv
cmake -S . -B build/linux
cmake --build build/linux -j
ctest --test-dir build/linux
```

If you cloned without `--recursive`, run `git submodule update --init`
first. The SimITL and Betaflight patches in `patches/` are applied
automatically at configure time.

The default build type is `Release`. Outputs in the build directory:

| File | What it is |
|---|---|
| `libfpv.so` (`libfpv.dylib` on macOS, `libfpv.dll` + `libfpv.def` on Windows) | the library |
| `examples/fpv_standalone` | a small SDL3 + OpenGL 3.3 game that uses every libfpv hook |
| `examples/fpv_minimal` | the smallest headless integration |
| `tests/fpv_tests` | the test suite |

### Options

| Option | Default | |
|---|---|---|
| `LIBFPV_WITH_SDL` | `ON` | Read radios and gamepads with the embedded SDL3 |
| `LIBFPV_WITH_OVERLAY` | `ON` | Build the Dear ImGui overlay |
| `LIBFPV_BUILD_TESTS` | `ON` | Build the test suite |
| `LIBFPV_BUILD_EXAMPLES` | `ON` | Build the examples |

### Windows

Windows builds are cross-compiled from Linux with MinGW-w64 (install the
`-posix` variant). All runtimes are linked statically, so `libfpv.dll`
depends only on Windows system DLLs:

```sh
cmake -S . -B build/win -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/mingw-w64-x86_64.cmake
cmake --build build/win -j
```

For 32-bit games (such as GTA IV), use `cmake/toolchains/mingw-w64-i686.cmake`
with `-B build/win32` instead.

MSVC hosts can make an import library with `lib /def:libfpv.def /machine:x64`
or load the DLL with `LoadLibrary`.

### Web (WebAssembly)

Browser builds use [Emscripten](https://emscripten.org) (4.0 or newer):

```sh
source /path/to/emsdk/emsdk_env.sh
emcmake cmake -S . -B build/web
cmake --build build/web -j
node build/web/tests/fpv_tests.js
```

This makes `libfpv.a` for the game's WebAssembly module, and
`build/web/web/`, the standalone example as a page (`index.html`,
`index.js`, `index.wasm`) that any static web host can serve. See
[docs/WEB.md](docs/WEB.md) for porting a mod and for what differs in a
browser.

### Updating

`git pull` alone is usually not enough. It does not move the submodules to
their new commits, and `cmake --build` does not apply patches that were added
to `patches/`; only a configure does. After each pull, run:

```sh
git pull
git submodule update --init
cmake -S . -B build/linux    # once for each build directory you use
cmake --build build/linux -j
```

The patched submodules are marked `ignore = dirty`, so `git status` will not
tell you they are out of date. If the build fails with undeclared Betaflight
or SimITL symbols after a pull, a patch is probably missing. Reset the
submodules (this discards any local changes in them) and configure again:

```sh
git submodule foreach 'git checkout -- . && git clean -fd'
git submodule update --init
cmake -S . -B build/linux
```

### macOS

macOS builds natively with Apple Clang (tested on Apple Silicon):

```sh
cmake -S . -B build/mac
cmake --build build/mac -j
ctest --test-dir build/mac
```

Hidden symbols keep the embedded SDL3's C functions private, but
Objective-C classes share one global namespace. If the game also links
SDL3, the Objective-C runtime warns that classes such as `SDL3Window` are
implemented twice, and which copy is used is undefined. `fpv_standalone`
shows these warnings because it links its own SDL3 on purpose. To avoid the
duplicate classes, build with `-DLIBFPV_WITH_SDL=OFF` and pass input in
with `fpv_input_set_channels`.

### Sanitizers

```sh
cmake -S . -B build/asan -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer" \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer"
```

## Trying it out

```sh
./build/linux/examples/fpv_standalone
```

| Key | Action |
|---|---|
| F8 | libfpv overlay (preset, radio setup, rates, camera, Betaflight CLI); Esc closes it |
| V | Camera: FPV / chase / line of sight |
| R | Repair and respawn |
| F11 or Alt+Enter | Fullscreen |
| Esc | Quit |

Without a radio, use the keyboard: Enter arms, W/S move the throttle, A/D
is yaw (starting slow and rising exponentially to full while held, how fast
is set in F8 > Controls) and the arrow keys pitch and roll (self-level).
With "Automatic hover throttle" on (F8 > Controls; by default it is on
while no radio is connected), W climbs (the
throttle rises exponentially to full; its speed is set there too), S cuts
the throttle, and no key (or both) hovers at a throttle learned in flight
for each drone. T toggles turtle mode: to
flip a crashed drone back over, turn it on, arm, hold an arrow key until
it lands upright, then turn turtle off to fly away.
Settings, EEPROMs and the
selected preset are stored in `./data` (change it with
`--data <dir>`).

The web build of the same example runs in a browser:

```sh
python3 -m http.server -d build/web/web 8080   # then open http://localhost:8080
```

## Integrating into a game

```c
fpv_global_config cfg;
fpv_global_config_defaults(&cfg);
cfg.data_dir = "<writable folder>";
cfg.coords = fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1.0f);
fpv_global_init(&cfg);                        // once

fpv_static_surfaces_load(&level_mesh, 1);     // world collision
fpv_drone *d;
fpv_drone_create(NULL, &d);                   // spawn

// every frame
fpv_drone_tick(d, dt, &frame);
// place the game camera at frame.camera, draw the drone at frame.position /
// frame.rotation, then draw fpv_osd_draw_data() and fpv_overlay_render()

fpv_global_terminate();                       // on unload / exit
```

- `include/libfpv.h`: the whole API, documented inline (threading, memory
  ownership, ABI rules).
- `docs/INTEGRATION.md`: integration details, such as the fisheye
  post-process shader, and lessons from real integrations (GTA IV).
- `docs/renderers/fpv_render_gl3.h`: a single-header OpenGL 3.3 / ES 3.0
  renderer for the OSD and the overlay.
- `examples/standalone/main.cpp`: a complete integration, with a comment at
  the top listing every hook in order.
- `docs/WEB.md`: compiling a game and its mod into a web page.
- `docs/PUBLISHING.md`: releasing a finished mod so players only download
  one ZIP or open one link (automatic builds, the install page, licensing).
- `docs/DECISIONS.md`: why things are the way they are.

## Repository layout

```
include/      public C API (libfpv.h)
src/          library: sim/, config/, input/, world/, osd/, overlay/, audio/, web/
assets/       presets and OSD fonts (embedded into the library at build time)
patches/      patch series applied to the SimITL and Betaflight submodules
third_party/  SimITL, Betaflight, libwebsockets, fmt, SDL, Dear ImGui, nlohmann/json
examples/     fpv_minimal and fpv_standalone
tests/        test suite and real-quad `diff all` fixtures
cmake/        core build, file embedding, toolchains, symbol export lists
tools/        patch series helper, WebAssembly symbol localizer
```

## License

GPL-3.0-or-later. libfpv embeds Betaflight, so any mod or program that links
it must be distributed under GPL-3.0-compatible terms. See [LICENSE](LICENSE)
for the full text.
