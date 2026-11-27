# libfpv design decisions

This is the log of decisions not fixed by the original brief. Each entry
gives the decision, why it was made and what it would take to reverse it.
Newest entries go at the bottom of their section.

## Dependencies and repository

### D1. Flattened HTTPS submodules, SimITL pinned, patches as a series

SimITL's own submodules use `git@github.com:` URLs and nest Betaflight,
libwebsockets and fmt inside SimITL. libfpv adds each of them as a top-level
submodule under `third_party/` with an HTTPS URL, pinned to exactly the commit
SimITL `c881d55` (2026-09-07) references:

| Submodule | Commit / tag |
|---|---|
| `third_party/SimITL` | `c881d55` |
| `third_party/betaflight` (AJ92 fork, 2025.12.0 pre-release) | `7bd98d8` |
| `third_party/libwebsockets` | `f69835e` |
| `third_party/fmt` | `9869971` |
| `third_party/SDL` | `release-3.4.18` |
| `third_party/imgui` | `v1.92.9` |
| `third_party/nlohmann/json.hpp` (vendored single header) | `v3.12.0` |

libfpv does not use SimITL's CMake files; `cmake/SimITLCore.cmake`
reproduces SimITL's Betaflight source list against the flattened paths. It
builds only a static, position-independent `simitl_core` library with hidden
symbols.

Changes to SimITL and Betaflight are kept as a patch series in
`patches/simitl/` and `patches/betaflight/`. CMake applies them at configure
time, and a patch that already reverse-applies is skipped. Patched
submodules are marked `ignore = dirty`. We use a patch series rather than a
GitHub fork so the changes stay reviewable in this repository and can be
offered upstream one at a time.

Betaflight upstream is at 2026.6.2 while SimITL's fork tracks 2025.12.0. The
brief fixes SimITL as the flight core, so libfpv stays on SimITL's fork.
Moving to a newer Betaflight means rebasing that fork, which is out of scope
for v1.

### D2. SimITL patches

* `0001-lifecycle-logging-failsafe.patch`
  * Root cause of "stopping corrupts memory": `wsStop()` destroyed the
    libwebsockets contexts while the WebSocket thread was still inside
    `lws_service()`. The new order is: signal, `lws_cancel_service()`,
    join, then destroy. Contexts can be re-created (`wsRestart`), so the
    Configurator service can be stopped and restarted without
    re-initialising Betaflight.
  * Root cause of "game exit crashes": the `Sim` singleton owns a joinable
    `std::thread` and its destructor neither joined nor detached it, so
    process exit ran `std::terminate` (reproduced: "terminate called without
    an active exception"). The destructor now detaches. It never joins,
    because on Windows the thread is already killed by `ExitProcess` and
    joining under the loader lock can deadlock.
  * The WebSocket base port is now configurable (`wsSetBasePort`; UART1 is
    base + 1, so the default stays at 5761). `lws_service` timeout was
    lowered from 1000 ms to 50 ms so shutdown is prompt.
  * Failsafe: `rxRcFrameStatus` can report `RX_FRAME_PENDING`, which makes
    Betaflight's own RX-loss and failsafe logic run when the radio
    disappears.
  * SimITL's `fmt::print` and Betaflight's `printf`/`fprintf` are routed to
    libfpv's log callback. Betaflight C files are compiled with a
    force-included header, `src/server/core/bf_log_redirect.h`. A game's stdout is
    never written to.
* `0002-exact-contact-velocity.patch`: `StateInput::contact == 2` makes
  SimITL take the host's linear and angular velocity exactly. The existing
  value 1 blends 75 %/50 %. libfpv resolves collisions with impulses and
  needs the result applied exactly, otherwise the drone sinks into the
  ground or tunnels.
* `0004-reversed-motors.patch`: `StateInput::motorsReversed` spins the
  props backwards for turtle mode (D21): thrust is negated and scaled by
  0.5 (an inverted airfoil), and the reaction torque flips sign.
* `0007-gps-from-host.patch`: SimITL's `updateGps` wrote `gpsSol`
  directly every step, which Betaflight's virtual GPS provider overwrote
  with its own (empty) solution at 10 Hz. It is no longer called; libfpv
  feeds the virtual GPS instead (D22).
* `0008-monotonic-clock-across-reboot.patch`: `systemInit` reset the clock
  to 0 on every in-process reboot (`systemReset`), while Betaflight's
  statics survive it. Timestamps they hold then lay in the future: the
  virtual GPS stayed silent for as long as the uptime before the reboot.
* `0011-rotor-drag-advance-ratio-thrust.patch`: thrust ∝ rpm² that falls
  off with the advance ratio, and rotor drag (D25).
* `0013-battery-draw-sag-duct-thrust.patch`: the motors' no-load current
  is drawn from the pack, the electronics draw on top of the motors,
  optional pack resistance for the sag, and duct thrust (D27).
* `0014-no-configurator-thread-on-the-web.patch`: in browser builds,
  `startServices` starts no WebSocket thread (D28).
* `0015-declare-init-for-reset.patch`: `target.c` calls Betaflight's
  `init()` from `systemReset` without a declaration, so C made it
  `int init()`. Native linkers ignore the difference. wasm-ld replaces a
  call through a mismatched signature with a trap, and every in-process
  reboot crashed (D28).
* `0017-wind-relative-airspeed.patch`: `StateInput::windVelocity` and
  `rotorWind[4]`, the air's velocity at the centre of mass and at each
  motor hub. Every aerodynamic term works on the airspeed, the motion on
  the ground velocity (D30).
* `0019-osd-rows-from-video-system.patch`: the fake OSD displayport has 13
  rows when `vcd_video_system` is NTSC, 16 otherwise, as the OSD chip does;
  Betaflight's `osdInit` then moves elements below the last row up to it
  (D15).
* `0021-esc-rpm-telemetry.patch`: the simulated ESCs report each motor's
  rpm to Betaflight as bidirectional DShot does, so dynamic idle and the
  RPM filter work (D38). The SITL target compiles `USE_DSHOT` and its
  telemetry. Patch `0029` adds the DShot ESCs.
* `0022-esc-start-from-standstill.patch`: an ESC starting a stopped motor
  drives nothing for 0.12 s first (D38).
* `0023-reversed-props-no-duct-thrust.patch`: props spun backwards (turtle
  mode) give 30 % of their forward thrust (was 50 %) and no duct lip
  thrust (D21, D38).
* `0024-rotor-turbulence.patch`: `StateInit::rotorTurbulence`, unsteady
  thrust per rotor (D38).
* `0025-osd-profiles.patch`: the simITL target defines
  `USE_OSD_PROFILES` (`common_pre.h` only does for targets that define
  `USE_OSD` before it), so Betaflight has three OSD profiles like a real
  quad, and the CLI's `osd_profile_2_name`/`_3_name` point at
  Betaflight patch `0019`'s fields (D15).
* `0029-dshot-motors.patch`: DShot ESCs for the simITL target. The rpm
  telemetry of `0021` only comes back with bidirectional DShot
  (`dshot_bidir = ON`), as from real ESCs (D44).

### D37. Client, server and shared code

The source tree has three parts, built as separate CMake targets with
their own include roots:

| Part | Holds | Built as |
| --- | --- | --- |
| `src/shared` | libfpv's own code both sides use: presets (`config/`), collision (`world/`), the video and RC link models, the wall clock, logging, and `simapi/` (the `Server` interface and the snapshot types) | `fpv_shared` |
| `src/server` | the simulation server: `SimServer`, the flight model, the runner, the warhead and the Betaflight glue in `core/`; `main.cpp` is `libfpv-server` | `fpv_server`, plus `simitl_core` |
| `src/client` | the C API, the per-frame presentation, the overlay, OSD, audio and input | the `fpv` library itself |

`src/web/libfpv_web.js` stays where it was: non-CMake web builds pass it to
`--js-library` by that path.

**The `Server` interface** (`simapi/server.h`) is the only way the client
reaches the simulation, Betaflight or SimITL. Every argument and result is
a plain value, with no callbacks and no pointers into the other side, so
each call can be carried as a message. Betaflight's configuration logic
(one EEPROM per quad, the first-run setup, the EEPROM upgrades, syncing the
video system, the warm-up, reboots after an import) lives in `SimServer`.

* **Channels:** the simulation pulls them each step through `ChannelSource`
  (`input/channels.h`), which the client's `InputManager` implements.
  Devices are still started and polled on the simulation thread.
* **The collision world:** the client keeps its own `CollisionWorld` for its
  rays (chase camera, wind probes, hearing) and mirrors every change to the
  server, object ids included (`createObject(..., id)`). A server in the
  client's process shares the client's world and ignores the mirror calls.
* **Presets:** `DronePreset::physics` is libfpv's own `PhysicsParams`
  (`config/physics.h`). The server turns it into SimITL's `StateInit`
  (`toStateInit`, `flight.cpp`), so the preset code needs no SimITL header.
* **The pilot's position** is kept on both sides (`Lib::pilotPos`), and
  Betaflight's arming flag names are fetched once at init.

**The boundary is checked.** The client and the shared code have no
Betaflight, SimITL or `src/server` include roots. `tools/check-boundary.cmake`
(the `license_boundary` test) also rejects any include of them through
`third_party/`, and any client or shared file without the dual license
line below.

**Licenses.** The client and shared code (`src/client`, `src/shared`,
`src/web`), `include/libfpv.h`, `docs/renderers` and the examples are
`GPL-3.0-or-later OR MIT` (`LICENSE-MIT`); their contributors agreed to it.
The server, Betaflight and SimITL stay GPL-3.0-or-later only, so a default
build, one library holding all of it, is GPL as before.

**Split builds** (`LIBFPV_SPLIT_PROCESS`, off by default, not for the web)
run the server in `libfpv-server` (`src/server/main.cpp`). The library
then links only the client and the shared code (`--no-undefined` proves
nothing in it reaches the server), and `RemoteServer`
(`client/remote_server.cpp`) carries every `Server` call there.

* **Starting.** The first `Server` call (in `fpv_global_init` at the
  latest, earlier if the game loads its world first) listens on 127.0.0.1 at a port the system picks and
  starts `libfpv-server --port <p> --token <t>` from the library's folder
  (`LIBFPV_SERVER` overrides). It opens two connections back, each with a
  hello carrying the token and `wire::protocolId()`: the version, the
  kamikaze option and the sizes of the structs that go as they are, so a
  server from another build is refused instead of misread. A missing or
  refused server makes `start()`, and with it `fpv_global_init`, fail with
  the reason. One server per process: like the in-process one, it keeps
  Betaflight booted across terminate and init.
* **The control connection** (`simapi/wire.h`) carries the client's calls
  in order. Calls without a result (most of them: spawn, world changes,
  wind, heartbeat, `fpv_step`) are sent and not waited for. Calls with one
  (snapshots, impacts, the CLI, `start`, `stop`) wait for their reply.
  libfpv-server runs them one at a time on one thread, so a result always
  reflects every call sent before it, as with direct calls. Plain structs
  go as their bytes; the ones with strings or vectors (`DronePreset`,
  `ServerConfig`, `WindState`, `HoverFeedback`) have their fields listed in
  `wire.cpp`, which a new field must be added to.
* **The input connection** goes the other way. `RemoteChannelSource` in
  libfpv-server forwards each `ChannelSource` call, and a thread in the
  client answers them from the `InputManager`: each step's channels (with
  the previous step's hover feedback), starting and stopping the devices,
  the radio-lost latch. The devices are thus started, polled and stopped
  on that one thread, as on the simulation thread in-process, and the
  keyboard hover, failsafe and manual stepping behave exactly the same. It
  costs a round trip per step (about 0.07 ms on loopback under WSL): inside
  the 1 ms real-time step, but `fpv_step` gets that much slower per step,
  and the test suite takes about 4x as long. Without the game, a step gets
  idle channels with the radio lost (failsafe).
* **The wall clock.** The client sends its `wallEpoch()` (a
  `steady_clock` count, system-wide) in the hello reply, so the snapshots'
  wall times interpolate against the client's clock unchanged.
* **The collision world.** libfpv-server's `SimServer` has its own
  `CollisionWorld`, kept up to date by the mirror calls the client always
  makes.
* **Logs.** libfpv-server's lines (Betaflight's included) go over the
  input connection and out through the game's log callback, at the level
  the game asked for.
* **Ending.** When the connections close (the game exits, crashes or is
  killed), libfpv-server saves Betaflight's settings if it was running and
  exits. At a normal exit the client asks it to first, as the in-process
  build does. If libfpv-server dies, the client logs it once and keeps the
  last state; the simulation is gone until the game restarts.
* **Assets.** The OSD fonts (Betaflight Configurator's, GPL-3.0) are not
  compiled in. They are copied to `libfpv-fonts/` next to the library, with
  their README and the GPL text, and read from there at run time
  (`LIBFPV_FONT_DIR` overrides the folder). The same for the shipped presets
  (`assets/presets`, some exported from SimITL's own): `libfpv-presets/`,
  overridden by `LIBFPV_SHIPPED_PRESET_DIR`. libfpv copies them into the
  player's preset folder as before; with the folder missing, it logs an
  error and flies its built-in default quad.
* **Tested** by running the whole suite against libfpv-server: the split
  build's `fpv_tests` goes through it, natively and on Windows.

**Why:** libfpv embeds Betaflight and SimITL (GPL-3.0). A build that runs the
simulation in its own process (libfpv-server) and keeps the client free of
GPL code lets a mod's game-side part be under other terms. Default builds
keep one library in the game's process with direct calls, and behave as
before. The structure is the same in every build, so the split build cannot
drift from the default one.

**Nothing GPL-derived in the client:** the overlay's OSD element names
(`overlay/osd_element_names.h`) are libfpv's own short descriptions, not
the Betaflight Configurator's, and the GPL assets (fonts, shipped presets)
ship as files in split builds.

**Reversing it** means moving the files back under `src/` and calling the
flight model and `bfb_*` from the API again. Nothing outside libfpv depends
on the split: the public API and the exported symbols are unchanged.

## Simulation

### D3. SimITL coordinate frame

Verified from `physics.cpp`, `bf.cpp` and the presets: X right, Y up,
Z forward, left-handed (Unity's convention), SI units, gravity −Y. Motor 1
(rear right in Betaflight's Quad X) is at (+x, −z), and motor 2 (front
right) is at (+x, +z). SimITL integrates velocities and orientation, but
**not position**. The host must feed the rotation back each update, because
`StateInput` overwrites it.

### D4. Betaflight configuration goes through Betaflight's own CLI, in-process

`src/server/core/bf_bridge.c` implements an in-memory `serialPort_t` and drives
`cliEnter`/`cliProcess` on the simulation thread. Preset defaults, overlay
edits and `diff all` imports are therefore just CLI commands. They go
through the same parameter-group code as the Configurator (which uses MSP
over the WebSocket), so there is one source of truth and no copy of any
Betaflight setting inside libfpv. `save` is always issued as
`save noreboot`. When the Configurator owns the CLI (`cliMode`), libfpv
reports "busy" instead of interleaving.

### D5. Default EEPROM settings

Measured: a fresh EEPROM does not arm. There is no ARM mode, there is a 5 s
power-on grace (`BOOTGRACE`), and gyro calibration (`CALIB`) takes about
4 s of sim time. libfpv's defaults set `pwr_on_arm_grace = 0`, DShot600
with bidirectional DShot (D44), and `aux 0 0 0 1700 2100 0 0` (ARM on
AUX1). The import puts the motor settings right after `defaults nosave`: a
`profile` line computes the mixer's output range for the protocol of that
moment.

A pilot can still pick PWM. With PWM and synced updates, Betaflight's
config validation raises `pid_process_denom` so the PID loop runs no faster
than the ESCs' 480 Hz, and the quad wobbles at about 20 Hz. On PWM,
`ensureEssentials` turns `use_unsynced_pwm` back on if it is off, with
`pid_process_denom` back to its default. DShot is always synced and has no
such limit.

### D6. Betaflight patches

* `0001-cli-array-set-null-deref.patch`: `cliSet` parsed array settings
  with `valPtr = strchr(valPtr, ',') + 1`. When a diff gives fewer values
  than the array length (common in diffs from older firmware), this turns
  NULL into `(char *)1` and the next iteration reads address 1.
  Found by importing the 4.0.6 corpus diff under AddressSanitizer.
* `0002-sitl-crash-flip.patch`: crash flip (turtle) mode is compiled only
  with `USE_DSHOT`, which SITL builds leave out. The patch enables the
  mode box, arming in crash flip and its beeper/disarm handling for SITL.
  The motor reversal a DShot command would do is applied by the simulator
  (D21).
* `0004-virtual-gps-keeps-feature.patch`: config validation turned
  `FEATURE_GPS` off on every save and boot when no serial port has the GPS
  function, exempting only the MSP provider. The virtual provider needs no
  port either.
* `0005-altitude-elapsed-time.patch`: the altitude estimate's filters and
  the vario's derivative assumed the ALTITUDE task runs at its nominal
  100 Hz. In the simulator it runs at about 33 Hz (D22), so altitude lagged
  and the vario read 3× too high. Both now use the measured interval.
* `0007-wasm-pg-registry.patch`: WebAssembly has no linker scripts. The
  parameter group registry and reset data use sections named like C
  identifiers there, so that wasm-ld provides `__start_`/`__stop_` symbols
  for them (D28).
* `0018-sitl-rpm-telemetry-analog-esc.patch`, for the SITL's DShot
  telemetry over PWM motors (D38): `initDshotTelemetry` also starts from
  the `useDshotTelemetry` flag the simulator sets; dynamic idle's zero
  drive is `mincommand` on analog protocols, not DShot's 48; the DShot
  beacon only runs on DShot (it stamped a beacon time anyway, and that
  held arming back for good); crash flip keeps working on SITL with
  `USE_DSHOT`; and in simulator builds a parameter group missing from the
  EEPROM starts from its defaults instead of making Betaflight reset the
  whole EEPROM. The RPM filter's group is new to the SITL, and without
  this every player's EEPROM, tunes and switches included, would be reset
  on upgrade. `dshot.c` skips its interrupt guard in a simulator.
* `0019-osd-profiles-keep-eeprom-layout.patch`: with three OSD profiles
  (SimITL patch `0025`, D15) `osdConfig_t` keeps only profile 1's name in
  place; the names of profiles 2 and 3 are `profileMore`, at the end of
  the struct, read through `OSD_PROFILE_NAME_BUF()`. Betaflight's own
  layout grows the name array in the middle, which would shift every
  later OSD setting (alarms, displayport device, canvas) of an EEPROM
  saved with one profile; appended, `pgLoad` keeps them all and the new
  names start empty.
* `0025-sitl-dshot-enable-time.patch`: in simulator builds the motors'
  enable time is at least 1 ms. 0 means "not enabled", and the simulator's
  clock reads 0 at its first boot, so DShot commands never started and
  `BOOTGRACE` never cleared: a quad on DShot did not arm (D44).

### D7. One EEPROM per quad preset

Each preset has its own Betaflight EEPROM at
`<data_dir>/eeprom/<preset>.eeprom`, like one flight controller per real
quad. Switching presets saves pending changes and then re-reads the other
file (`initEEPROM` → `ensureEEPROMStructureIsValid` → `readEEPROM`). A
missing file is first reset to Betaflight defaults, so it never inherits
the previous quad's memory image. Then libfpv's first-run script and the
preset's own tune (`<preset>.diff.txt` next to the JSON, if present) are
applied. `diff all` imports go to the active preset's EEPROM.

### D8. Props follow `yaw_motors_reversed`

Measured: with SimITL's preset motor directions (−1, 1, 1, −1) and
Betaflight's default "props in" mixer, the yaw loop has positive feedback.
The drone spins up to 18 rad/s and runaway-takeoff prevention disarms it.
SimITL's own tools work because pr0p's EEPROMs set
`yaw_motors_reversed = ON`. libfpv does not force that setting, because an
imported real "props in" diff would then spin the wrong way. Instead the
simulated prop directions follow Betaflight's `yaw_motors_reversed`, read
every 64 steps so Configurator edits apply live. It flies stably both ways
(tested).

### D9. Collision shape

The collision shape is a sphere at the centre of mass plus one under each
motor bell, with level bottoms. With only a body sphere the drone rested
on a single point and ground friction could not stop yaw spin (measured:
12.8 rad/s forever). Prop strikes are tested on the propeller discs
separately and damage the props (`durability` from the prop JSON);
`FPV_MATERIAL_KILLS_PROPS` destroys them on contact. A prop touching
something also stops within a few revolutions and its ESC loses sync: the
motor stays stalled, without thrust, until the ESC starts it again
(about 0.1 s, D38).

Prop tips and duct rims (turtle flips and perches). Open props collide at
their tips: a ring of 12 small spheres (4 mm) at the prop's radius. A quad
coming over in a turtle flip, or landing tilted, touches down on them
first, as far out as they reach, and that far-out hit takes the spin out of
it. On motor feet alone (a 5" lands on them 6 cm out) it kept a third of
its spin and rolled on over. A ducted quad's ducts are rings, not balls: a
core sphere plus a ring of 8 small spheres at the duct's bottom and its
top, so it stands on flat rims and tips over an edge. These small spheres
are detail on larger parts (`Sphere::fine`): the collision substeps stay
sized by the large ones, and an Air65 II costs about 40 % more simulation
time than with its 10 spheres.

A second sphere sits on top of each motor (bell and prop nut, 6 mm above
the prop disc). Without them an upside-down drone balanced on the body
sphere: it rocked until a prop tip touched the ground, so props spun up in
turtle mode were destroyed by strikes, and a turtle flip pivoted on the
centre and spun far too easily. Now it rests on the motor tops with props
clear and pivots on an arm, like a real quad. Contacts on motor spheres
never bounce (restitution 0): arms and props soak up the hit. With the
material's restitution a quad landing on an arm after a flip cartwheeled
on over it.

The last sphere is the canopy, around the FPV camera. The quad JSON's
optional `"collision"` object sets the shape, in metres from the centre of
mass (+Y up, +Z forward):

```json
"collision": {
    "bodyRadius": 0.02,
    "motorRadius": 0.017,
    "canopy": { "offset": { "x": 0.0, "y": 0.016, "z": 0.014 }, "radius": 0.021 }
}
```

**Every drone model must keep its camera inside its collision shape:** the
lens (`camera.offset`) at least 13 mm (the 10 mm near plane plus 3 mm)
inside it in every direction. This applies to every new preset. Give a
real canopy where the quad has one, so it lands on its top like the real
one, and treat the "canopy did not keep the camera" warning as a bug in
the preset. The `camera_clearance` test drops every built-in preset on its
top, nose and side and fails if a camera gets closer to the ground than
its near plane.

Why: the camera sits ahead of and above the centre, outside
the body sphere on most frames: upside down the Fluid75's lens rested on
the ground (0 mm), the Air65 II's 4 mm and the 3" Slayer's 9.8 mm above
it, all inside the 10 mm near plane, so the ground was clipped away and the
FPV view looked through it. `ensureCameraClearance()` makes every preset
keep its lens `kCameraClearance` (near plane + 3 mm) inside the canopy:
without one it is a sphere of that radius on the lens, and a canopy too
small for its camera grows, with a warning. One sphere is enough for the
guarantee: a surface closer than that to the lens is inside the sphere,
so it is pushed out like any contact. The canopy is part of the body
(motor -1): it bounces with the material's restitution and its hits
damage all props. The Air65 II's own canopy stands above its ducts, so it
rests on its top tilted on the canopy and two ducts, as the real whoop
does; its tune's `small_angle = 180` then lets Betaflight arm it there,
which the turtle test allows.

### D10. Warm-up, so a fresh drone can arm immediately

Betaflight needs about 3.7 s of simulated time after boot for gyro, acc
and baro calibration, and RX is suspended briefly whenever an EEPROM is
read. `fpv_global_init`, EEPROM switches and the reboot after an import
step the parked simulation until `CALIBRATING`, `RXLOSS`, `FAILSAFE` and
`BOOTGRACE` clear. This costs about 40 ms of wall time.

### D11. Timing model: dedicated simulation thread at 1 kHz

Measured: one 1 ms SimITL step (8 Betaflight PID loops) costs about 10 µs
on a desktop CPU, so a 1 kHz thread uses about 1 % of a core. Collision
substeps keep every sphere moving less than half its radius, so a 30 m/s
drone takes about 4 substeps per step.

* `FPV_TIMING_THREAD` (default): a libfpv thread steps the simulation at
  `sim_rate_hz` and polls the radio at the same rate. Pacing uses a
  high-resolution waitable timer on Windows and `sleep_until` elsewhere. A
  fixed-step accumulator gives frame-rate independence and is capped at
  50 ms, so a host hitch cannot cause a spiral. Each step carries the wall
  time it represents. `fpv_drone_tick` renders one step (1 ms) in the past
  and interpolates (pos lerp, rot slerp), so there is no visible jitter at
  any host frame rate and no extrapolation error.
* `FPV_TIMING_MANUAL`: the host calls `fpv_step` itself. This is
  deterministic and used by the tests and lockstep engines.
* Auto-pause: when the host stops calling `fpv_drone_tick` for 0.5 s (a
  pause menu, loading screen or debugger), the simulation stops, unless the
  drone is disarmed on the ground. In that case Betaflight keeps running so
  the Configurator stays responsive.
* All Betaflight access (CLI, EEPROM, presets) is queued to the simulation
  thread. The public API is serialised by one recursive mutex.

The alternative, stepping inside the host's tick, would tie radio sampling
and PID timing to the game's frame rate (16 ms at 60 FPS) and break the
"radio polled at sim rate" requirement.

### D12. libfpv does not retune SimITL's aerodynamics (superseded by D25)

Measured with PropEater 5" (6S, 1900 kV): hover at 14 % throttle, and a
vertical punch-out reaches about 275 km/h after 2 s (thrust-to-weight
about 15:1 from the prop JSON). That is high but within what real 6S
racing quads do, and the model is SimITL's by design. Tests check
plausibility ranges, not exact values. D25 replaced this: pilots found the
quads floaty, with a soft throttle.

### D13. `diff all` import rules

* The import starts from `defaults nosave`, then applies the diff, clears
  all 20 mode slots, applies libfpv's base settings, re-adds the modes and
  saves without reboot. If Betaflight then reports
  `REBOOT_REQUIRED`, libfpv reboots it in-process (Betaflight's own
  `systemReset` → `init()`, the same path as the Configurator's
  "Save and Reboot") and warms it up again.
* Dropped as hardware-specific, each with its reason in the report:
  resources, timers, DMA, serial ports, board identity, LED strip,
  VTX/vtxtable, beacon, receiver settings, sensor hardware and
  calibration, OSD chip, battery/current sensor scaling, and `map` and
  `rxrange` (libfpv's radio wizard owns channel order and endpoints).
* `profile`/`rateprofile` indices this Betaflight does not have (newer
  firmware has 6 rate profiles, the embedded one has 4) are skipped
  together with every `set` that follows them, so those settings cannot
  land in the wrong profile.
* ARM is moved to AUX1 with range 1700–2100, the wizard's "switch on". The
  other mode channels are packed into AUX2–AUX4 in order of appearance. A
  fifth distinct switch channel is dropped and reported.
* Settings Betaflight rejects (renamed since older firmware) are listed
  under "Rejected by Betaflight". They are not an error.
* Corpus: six real diffs from Betaflight 4.0.6 to 4.5.1
  (`tests/diffs/README.md`). Every one imports, arms and flies in the test
  suite.

### D25. Realistic thrust and rotor drag (patch `0011`)

Pilots of real 5" quads and tiny whoops reported that libfpv felt floaty.
With the throttle cut, the quad fell too slowly. Full throttle had the
opposite problem: it climbed endlessly. Reading SimITL's model against the
literature found three causes:

* Static thrust was `b·rpm + a·rpm²`. The linear term made idle thrust
  about 4× too high: 26 % of the weight on PropEater at Betaflight's 5.5 %
  idle. Real props follow rpm², about 7 %.
* Thrust fell with climb speed through a fixed quadratic that did not
  depend on rpm. It only reached zero at 142 m/s, so a punch reached
  240 km/h within a second.
* Nothing modelled rotor (induced) drag. That drag is linear in speed and
  is the main drag of a quadrotor at low and medium speed. NeuroBEM
  (Bauersfeld et al. 2021) calls it "the dominant source of drag".
  Without it, a quad that levels out at 40 km/h coasted for about 7 s
  (computed) before reaching 20 km/h.

The patch adds `propPitch`, `propDiameter` and `rotorDragFactor` to
`StateInit`. When a pitch is known, the model changes as follows:

* Static thrust is `Tmax·(rpm/propMaxRpm)²`, with `Tmax` =
  `propThrustFactor.z`.
* Thrust is scaled by `1 − (v/v0)²`, where `v` is the axial inflow
  (climbing along the thrust axis) and `v0` = 0.9 × pitch speed. That is
  the shape of measured small-prop CT(J) curves. Descending keeps static
  thrust.
* Prop torque keeps 25 % of its static value at zero thrust, so an
  unloaded prop speeds up the way a real one does.
* Rotor drag per motor is `F = −c·ω·v_inplane`. It acts at the motor, so
  flying forward also pitches the nose up a little. `c` is
  `rotorDragFactor` times the momentum drag of the air one disc pulls
  through at hover: `c = k·sqrt(ρ·A·Tmax/2)/ω_max`.
  * For open props `k = 0.6` (the quad JSON's default) matches Faessler et
    al. 2018: 0.45 s⁻¹ mass-normalised at hover on a 5"–6" racer, against
    a measured 0.39–0.54.
  * Ducted whoops use `k = 1`.

The pitch comes from `pitchInch` in the prop JSON, or from the name
("5.1x3x3"); `diameterInch` works the same way. A prop with no known pitch
keeps SimITL's curve, and `rotorDragFactor: 0` turns rotor drag off. Every
shipped prop name carries its pitch, so every shipped preset uses the new
model.

`flight_realism` measures each preset. The reference bands come from real
quads of the class:

| Measurement | Freestyler 5" (6S, 580 g) | Air65 II (1S, 26 g) | Nomade 7" (6S Li-ion, 940 g) |
|---|---|---|---|
| Hover throttle (stick) | 22 % | 33 % | 27 % |
| Drop 1 s after cutting throttle (free fall: 4.9 m) | 4.2 m | 3.8 m | 4.4 m |
| Climb speed 0.25 s into a full punch | 70 km/h | 23 km/h | 45 km/h |
| Climb speed 1 s into a full punch | 137 km/h | 49 km/h | 99 km/h |
| Top speed, level | 141 km/h | 56 km/h | 108 km/h |
| Coast from 40 to 20 km/h after levelling out | 1.3 s | 0.5 s | 1.2 s |

(Air65 II figures after D38, which calibrated it against a real Air65 II's
blackbox logs: its thrust, motors, pack and idle. The parts figures below
were its first build.)

The Freestyler and Air65 II presets were built from real parts.
Freestyler is the default and is a typical 5" freestyle quad: 2207
1950 kV motors, 5.1x3x3 props, a 6S 1100 pack and an HD air unit. The
Air65 II uses BetaFPV's published figures: 17.8 g dry, 0702 25000 kV
motors, Gemfan 1219S props, a LAVA II 1S 320 mAh LiHV pack (8.2 g) and
about 34 g of thrust per motor. Its `.diff.txt` carries the filters, PIDs and
rates from a pilot's own flying Air65 II (Betaflight 2026.6), plus
`vbat_max_cell_voltage = 440`; without that, Betaflight reports a full LiHV
cell as "NOT PRESENT". Only the tune is taken from that diff. The quad keeps
libfpv's defaults: air mode on, libfpv's OSD and switches.

The Nomade is a classic analog 7" long range build from typical parts, not
one product: a 315 mm frame (270 g with the stack, GPS, camera and VTX),
2806.5 1300 kV motors, 7x3.5x3 props (about 2.1 kgf each at 6S), a 6S1P
21700 Li-ion pack (4200 mAh, 440 g), a Caddx Ratel 2 at 20 degrees and a
Unify Pro32 HV at 1 W with 9 W of electronics. About 9:1 on a fresh pack;
`battery_endurance` measures 5.5 A at hover (over 30 minutes of hover on
the pack) and 64 A flat out. The pack uses the SimITL sag (`maxVoltageSag`
5 V) rather than an internal resistance: with `internalResistance` at a
Li-ion's ~0.12 ohm the sag fed back into the 6S current until the voltage
collapsed. The voltage curve is still a LiPo's, so the OSD reads a LiPo's
voltages; the energy and the weight are the Li-ion's. Its
`vtxAntennaShadowDb` is 10 dB (the big frame and pack, D34). No
`.diff.txt`: Betaflight's defaults fly it.

Zero throttle (patch `0012`). Pilots found the Air65 II rolling hard toward
belly-first as soon as they chopped the throttle. SimITL puts the frame's
centre of pressure a fixed 2 cm above the centre of mass. On a 5" that is
about a fifth of the arm. On a 65 mm whoop it is most of the quad, and the
torque from it at 30 km/h was larger than all four motors at full throttle
could answer. The offset is now 0.1 × the centre-to-motor distance (11 mm
on the Freestyler, 3 mm on the Air65 II). `idle_attitude` drops both from
rest at 45°, 90° and 135° with zero throttle. With air mode, both hold
within about 4° over 3 s. Without air mode, Betaflight resets the I-term at
zero throttle and halves the PID authority, and both quads still fall
belly-first, as real quads do.

Switching quads reboots Betaflight. Reading a quad's EEPROM only reloads
its PID and rate profiles. Features, gyro filters and the dynamic notch are
set up at boot. Before this change the next quad kept the previous quad's
features (air mode included) and filters until a reboot. The same applied
to a fresh quad's first-run tune, which is saved without a reboot.
`fpv_import_betaflight_diff` also reboots when a feature changed, because
Betaflight does not flag feature changes as needing a reboot.

Still open:

* Vortex ring state. SimITL's propwash noise stands in for it.
* Thrust gain in a fast descent.
* Validation against a logged flight. NeuroBEM's raw Betaflight logs with
  motion-capture positions are the reference to fit next.

### D27. Battery draw and whoop punch (patch `0013`)

A pilot flew the Air65 II hard for 2:30 and still read 3.8 V at hover; the
real quad would have landed by then. It also took too long to reach its
top climb speed in a punch, where the real one gets there almost at once.

Battery. The Air65 II drew 1.2 A at hover against about 3 A on a real
65 mm 1S whoop; full throttle (14.5 A) was about right. Three causes:

* The motor current came from the output torque only. The no-load
  current `motorI0`, which the model already spends as friction torque,
  was never drawn from the pack. It now is (the Freestyler's hover went
  from 2.5 A to 4.1 A at 25 V, about 5.7 g/W against a real 4–5).
* SimITL took 5 W as a floor for the whole quad, not on top of the
  motors, so at hover it hid the whoop's motors entirely. The
  electronics now draw on top of the motors: `electronicsPower` in the
  quad JSON, 5 W by default (an HD air unit), 2 W on the Air65 II
  (analog VTX at 25 mW, camera, AIO board).
* Sag was `maxVoltageSag × throttle²`, 0.04 V at hover on 1S. A pack with
  `internalResistance` (ohms, pack plus plug and leads) now sags by
  `I × R × (1 + used²)`, the last factor being SimITL's growth as the
  pack empties. The LAVA II uses 35 mΩ: 0.1 V at hover, 0.65 V on a punch.

The 0702 now has `motorI0` 1.3 A and `motorR` 0.33 Ω (motor, ESC and
wiring). A 0702's bench no-load current is lower; the extra lumps the
iron and ESC losses that make a whoop far less efficient at hover than at
full throttle, which a prop with power ∝ rpm³ cannot do on its own. R
keeps full-throttle rpm, and so the top speed, where it was.

`battery_endurance` flies a hard pack as a repeating 4 s cycle (1 s full
throttle, 1 s at 60 %, 1 s at zero, 1 s hover) until the hover cell
voltage drops below 3.5 V:

| | Freestyler 5" | Air65 II |
|---|---|---|
| Hover draw | 4.1 A | 2.0 A |
| Full-throttle draw | 77 A | 18 A |
| Hard pack lands after | 124 s | 140 s (247 of 320 mAh) |

(Air65 II figures after D38. Its logs show 2.45 A at hover on a 3.7 V
cell, about 2.1 A on a fresh one, and replace the 3 A estimate above, the
1.3 A no-load current and the 0.33 ohm.)

Punch. BetaFPV's bench figures (32 g at 23000 kV and 35.4 g at 27000 kV
on the 1219S) give about 34 g per motor and a 5.2:1 thrust to weight.
Even without drag that needs 0.45 s to reach a 60 km/h climb, so the
punch could not be made quicker by drag or motor response alone. Those
figures are for a bare prop on a stand. A whoop's duct adds static thrust
through the suction on its lip, and loses it as the quad climbs.
`ductThrustGain` in the quad JSON is that extra static thrust (0.3 on the
Air65 II, the low end of measured whoop ducts). It fades linearly to zero
at half the zero-thrust inflow speed, about the top climb speed, so the
top speed stays the same. From a steady hover, 90 % of the climb speed now
comes after about 0.5 s instead of 0.7 s (34 km/h at 0.25 s instead of
27), and hover throttle drops from 32 % to 29 %. `flight_realism` now lets
the quad settle for 1 s after the teleport before it punches; the wobble
from the teleport had been costing about 10 % of the thrust.

D38 replaced the bench figures with the real quad's: about 6.5 g per prop
at hover (27000 rpm) and 24 g at 53000 rpm, about 4:1 with the duct. Its
punch builds to 23 km/h in 0.25 s.

### D30. Wind (patch `0017`, `src/shared/world/wind.*`)

Games have weather, and a quad in wind is a different thing to fly: it
leans into the wind to hold position, drifts in angle mode, gets bumped
by gusts, goes calm in the lee of a building and gets rough at its
corners. Pushing the drone with a force would get none of that right.
Real wind changes the *airspeed*, and everything aerodynamic follows from
it: drag, rotor drag, the props' inflow and prop wash. Betaflight then
has to fight the result through its own PID loop, as on a real quad.

**The flight model (patch `0017`).** `StateInput` carries the air's
velocity at the centre of mass and at each of the four motor hubs (world
frame). SimITL subtracts it wherever it used the ground velocity for
aerodynamics:

* each prop's axial inflow (the thrust and torque falloff of D25), prop
  wash and airspeed cooling use its own hub's air;
* frame drag, the Munk and centre-of-pressure moments and the flutter
  use the air at the centre of mass;
* rotor drag uses each hub's air in its prop plane.

Integration stays in the ground frame. Still air (all zero) is
bit-for-bit the old model. The hubs get their own samples so that a gust
that hits one side of the quad changes that side's thrust, and the
torque is what the gyro feels.

**What the host gives.** Only what a game knows: the weather's mean wind
at 10 m (`fpv_wind_set`), moving-air sources it simulates
(`fpv_wind_set_sources`: rotors and jets) and explosions
(`fpv_wind_add_blast`). Everything local is libfpv's job, so every game
gets it the same way.

**Local wind (`WindProbe`, host thread, 10 Hz).** Rays through the same
collision as the drone (static surfaces, objects and the host callback,
whichever is nearer), about 50 per probe:

* Height. The log profile `u(z) = u10·ln(z/z0)/ln(10/z0)` with
  z0 = 0.3 m, clamped to 0.3–2.2×. z is the height over the *lowest*
  ground within 25 m (five down rays): over a 50 m roof it is 50 m, not
  the 2 m to the roof, so a rooftop gets the wind of its height. On the
  ground in the open that is the 0.3× floor (2.4 m/s for an 8 m/s
  weather wind); at Maze Bank height, about 1.9×. Within 3 m of a roof
  that stands well above the ground it is halved at the surface (the air
  that separated at the roof's edge has not come back down), so a quad
  sitting on a skyscraper roof is not blown straight off it.
* Shelter upwind. Five headings ±30° around upwind; at each, horizontal
  rays at the drone's height and 3, 10, 25, 60 and 140 m above it. The
  nearest wall at the drone's height is the obstacle; the highest rung
  still hitting within 25 m of it gives its top. With x = distance /
  obstacle height, the mean wind kept is `1 − cover·deficit(x)`:
  deficit 0.85 from x = 1 (a little more, reversing the flow, right
  behind the wall), falling as `exp(−(x−1)/5)`, so it is mostly back by
  ten heights. `cover` (0..1) is how far below the top the drone is.
  Turbulence rises by up to 0.8× in the wake, most at about one height
  behind it (where the flow reattaches), less in the cavity. The
  numbers are round values from wind tunnel and field studies of
  building wakes (wake length about 10–15 H, recirculation about 1–2 H).
* Windward walls. Three rays downwind, 15 m: a wall ahead slows the
  wind (up to 60 %) and, if it is tall, turns part of it upwards.
* Enclosure. A roof within 60 m and walls within 30 m in at least half of
  eight directions scale the weather down to 5 % (halls, tunnels, car
  parks). Hosts that know about interiors pass `FPV_WIND_ENCLOSED`.
* All of it is low-passed with a 0.6 s time constant, and re-probed at
  once when the wind turns more than 15°. A new or reset drone starts
  without the old drone's memory.

**Gusts (sim thread).** Perlin noise in five octaves at fixed sizes, 60 m
down to 1.2 m, weighted like the von Kármán spectrum around the length
scale (0.6 × height + 5 m, up to 80 m). The size of each octave is
fixed and only its weight follows the length scale, so a changing
length scale never jumps the field. The field is carried along by the
wind (frozen turbulence) through a drift integrated every step, again so
that a changing mean wind never jumps it. It is carried by the *free*
wind, not by the calm air of a wake, otherwise gusts behind a building
would stand still. Each eddy also changes shape over its own turnover
time. The intensity is `1/ln(z/z0)` (0.1–0.35) unless the host gives
one. Across the wind the gusts are 0.8× as strong, vertically 0.55×.

**Sources and blasts.** A rotor wake follows momentum theory: it
contracts to 0.71 R and twice the induced speed one radius below the
disc, then spreads like a turbulent column. Where it lands (a ray from
0.6 R below the hub, past the aircraft's own body) it turns into a wall
jet along the ground. A jet spreads at about 6° and slows past a
potential core of six radii. Both carry gusts of about a quarter of
their speed. A blast is a front running out at 100 m/s (slower than a
real shock, so the push lasts long enough to feel), 40 m/s at the
explosion's radius and weakening as (R/r)^1.5, gone within about a
second.

**Cost.** About 50 host rays per probe at 10 Hz, plus one per rotor
source every frame. In the simulation, five wind samples per step (about
75 noise evaluations): microseconds. The player can turn wind off or
scale the weather's part (F8 > Fly > Wind), saved in `settings.json`. It
is on by default; with wind off, sources and blasts do nothing either.

**Not modelled.** Flow around individual corners (speed-up, corner
vortices), channelling along streets, thermals, and the drone's own prop
wash reflecting off walls. A real CFD solve around the drone (a local
lattice-Boltzmann grid on a heightfield from column rays) would give
corners and streets; the probes were chosen for being cheap and stable,
and they give most of the feel. To reverse: zero the wind inputs
(`fpv_wind_set(NULL)`, or the player's switch); the flight model is then
the D25 one exactly.

### D32. Simulation options: no battery drain, no damage

Players can turn off two parts of the simulation in F8 > Drone (saved in
`settings.json`, `fpv_sim_set_options` for the game): the battery's drain
and the damage to props and motors. Both are on by default.

- **No battery drain** puts back the charge each step drew
  (`Flight::keepBattery`), so the pack freezes at the level it had when
  drain was turned off: a half-flown pack stays half-flown, and the
  mAh counter holds its reading. A respawn or repair (SimITL's Repair
  command, run outside the step) still fits a full pack, which then stays
  full. Topping up to full every step was the first version, but it made
  turning drain off mid-flight refill the pack, which players read as
  a free charge. The sag is computed inside the step from its current
  (D27), so the OSD voltage still dips in a punch and comes back at hover.
- **No damage** ignores every source of prop damage in libfpv (strikes,
  `FPV_MATERIAL_KILLS_PROPS`, `fpv_drone_damage_props`) and keeps the
  windings 1 C under SimITL's burn-out temperature before and after each
  step, clearing the burned-out flag (`Flight::keepMotors`). The motors
  still warm up, so the OSD's ESC temperature reads true. A kamikaze
  warhead still destroys the drone: that is its own feature, not damage.
- `fpv_drone_damage_props` is how the game damages props from what libfpv
  cannot see (fire, a water volume, bullets, see INTEGRATION.md "Damage").
  It goes through the same switch, so the game never has to check it.

Reversing: drop the two checks; nothing else depends on them.

### D38. The Air65 II against its own blackbox logs

The Air65 II preset was built from parts figures (D25, D27). Pilots of the
real quad found libfpv's less reactive and lively than flying it indoors.
It is now calibrated against a real Air65 II on the preset's own tune
(the `.diff.txt` is that quad's): 24 armed flights in a room, about
8 minutes at 810 Hz (gyro, setpoints, motor commands, eRPM, current,
voltage), and the goggles' DVR. Each finding was checked by replaying the
logged sticks into libfpv (manual timing, the log's pack voltage) and
comparing with the log.

**Thrust.** The accelerometer against the motors' rpm gives
8.5e-11 N per rpm² per prop, duct included. That is 6.5 g at the hover's
27000 rpm and 24 g at the 53000 rpm it reaches flat out, about 4:1. The
preset had 2.2 times that: it hovered at 18000 rpm with 7.4:1. Its
`propThrustFactor.z` went from 0.33 to 0.151; `ductThrustGain` stays 0.3.

**Motors.** rpm against motor command × voltage, and the pack current,
fit kV 25000 with `motorR` 0.226 ohm, `motorI0` 0.25 A and
`propTorqueFactor` 0.0065. The old 1.3 A no-load current was more than
5.5 % idle could drive: the props stood still at idle, where the real ones
turn at 3000-4000 rpm. eRPM follows the command about 25 ms behind going
up and 35 ms going down, after about 2.5 ms; `propInertia` 2.2e-8 gives
about 30 ms with SimITL's ESC delay and filter.

**Dynamic idle (rpm telemetry).** The real quad runs bidirectional DShot
with dynamic idle (`dyn_idle_min_rpm = 30`) and the RPM filter. With
dynamic idle on, Betaflight starts the motor range at zero drive instead of
`motor_idle` and holds the minimum rpm instead. Without rpm telemetry the
simulator kept the 5.5 % offset under every throttle: +0.036 motor command
throughout, about 10 % more thrust at hover. A replayed indoor flight
climbed 150-280 m. Now SimITL feeds every motor's rpm to Betaflight as
DShot telemetry (SimITL `0021`, Betaflight `0018`), and the replay stays at
room height, with motor commands and rpm within a few percent of the log.
Quads whose tune has no dynamic idle only gain the RPM filter.

**Inertia.** Angular acceleration against the rpm differences at flips
gives roll 8.5e-6 and pitch 1.2e-5 kg m²; a parts estimate gives 7.5e-6
and 9e-6. Both axes were 1e-5. `invInertia` is now 118000 (roll, z) and
83000 (pitch, x).

**Pack.** Voltage against current in five logs: 45 mOhm, not 35.

**ESC start.** Every motor starting from standstill (arming, each push of a
turtle flip) stays at 0 rpm for about 0.1 s, then spins up. SimITL `0022`
drives nothing for 0.12 s below 300 rpm. Real dynamic idle keeps motors
running in flight, and so does the simulator's.

**Turtle.** The logged flips: full power held (`crashflip_rate` 0, in the
Air65 II's tune, D21), about 60000 rpm backwards, about
14500 deg/s². Reversed props now give 30 % of their forward thrust, not
50 %, and no duct lip thrust (`0023`).

**Turbulence.** Indoors the real quad is bumped all the time: its rate
error in 3-30 Hz, as a median over half-second windows, is about twice
the clean simulator's. Its own wash comes back round in a room. Each rotor
now gets unsteady thrust, `rotorTurbulence` in the quad JSON (the share of
thrust, standard deviation; 0 by default, 0.015 on the Air65 II), as a
Gauss-Markov process with a 60 ms correlation time (`0024`). That matches
roll and pitch; 15 ms put the power above 10 Hz and overshot there.

**Perched yaw.** Sitting on the floor at zero throttle, full yaw turns the
real quad at about 500 deg/s after 0.2 s. The simulator does that on a
floor with friction 0.3-0.4; on 0.6 it turns slower. The default material
friction is now 0.3, a smooth floor; rough ground is 0.6 where the game
sets it (INTEGRATION.md).

Not changed:

* Air mode. The real quad flies with `feature -AIRMODE` (its logs show the
  non-air-mode mixer); the preset keeps libfpv's air mode on (D25). Without
  it, zero throttle is looser, and full yaw on the floor drives the motors
  half as hard: about 0.45 against 0.78. With air mode the simulated Air65
  II hops off the floor in that yaw, as a real one with air mode would.
* Heave. Thrust against vertical speed in the logs falls about 5 % at
  ±1.25 m/s; the model loses about 3 % climbing and nothing descending.
* Yaw buffeting stays below the log's. The unsteady thrust doesn't change
  the motors' reaction torque.

The `air65_blackbox` test checks the hover rpm and drive, the idle rpm and
the ESC start against the logs.

### D39. Perching on moving surfaces

A quad sitting on a surface object (a car roof, a lift, a deck) bounced
and fell off in almost every case. The `perch_moving` tests now put a
disarmed quad on a deck moved like a car (20, 50 and 130 km/h, speeding
up and braking, a 25 m bend, a bumpy road), a lift, the standalone's
platform, a turntable and a rocking boat deck. The host's frames are 12-24
ms with 40 ms hitches. `perch_moving_threaded` runs the same in real time.
Four causes, four fixes:

* **The path between moves.** A surface went from where it was to the new
  pose over the last move's interval, then stood still until the next
  one. Frames are uneven, so it kept stopping dead and jumping, and its
  velocity (distance over interval) was off by the jitter. At 36 m/s, 3 ms
  is 11 cm. It also always lagged one frame behind. Now each object keeps
  its last moves. The surface follows the least squares line through the
  last 60 ms of them (pose and turn), moved on at its velocity. Each new
  move bends the path over to its new line within 50 ms, as a cubic
  Hermite with no jump in velocity. Contacts get that path's own velocity.
  Without new moves it goes on for four of the host's intervals after the
  last one came (0.1-0.3 s), then stops; a gap of 0.5 s or a jump of 1 m
  (0.5 rad) off the path starts afresh.
  Positions are kept from an anchor near the object: rounding the path to
  float steps at 2.5 km made a slow deck carry the quad a third too far.
* **When a move is for.** The simulation's clock, read when the move
  comes, is a few ms early or late against the game's (and in whole
  `sim_rate_hz` steps). Smoothing that noise out needed a long fit and a
  slow blend, and the surface lagged the drawn car by 50-75 ms (2 cm on a
  bumpy road). Moves now wait for the host's own clock. Threaded, they wait
  for the next `fpv_drone_tick`: the frame's time is the sum of its
  `frame_dt`, mapped to the simulation's clock at the rate the simulation
  runs against it (the last second) and pulled to it over 0.5 s. Manual,
  they wait for the next `fpv_step`, stamped with the sum of the steps.
  `fpv_step` carries the remainder of a whole step to the next call, so
  the simulation's clock keeps to the host's. A host that moves its
  surfaces after it ticks or steps gives poses for the time already
  reached, not the coming one. Each move carries both times. On uneven
  frames only the right one lines the moves up on a smooth path (the
  residual of a parabola through the last 12), so one world-wide tally
  picks it. Moving after the tick costs one frame of latency, nothing else.
* **Substeps.** All the substeps of a step met the surfaces where they
  are at the step's end. Each now meets them at its own time.
* **Resting contact.** Penetration was pushed out all the way, so a
  sphere sat exactly on the surface and every other substep found no
  contact. In those, the pull down a slope or the deck's acceleration
  went unchecked. The quad crept down a 15 degree slope that friction 0.3
  holds, and off a roof that held it. The push now leaves 0.5 mm
  (`kContactSlop`, as Box2D's slop) and the contact persists.

What decides whether a quad stays on is then physics: friction × weight
against the deck's acceleration and the drag of the air it moves through.
On the tests' car paint (friction 0.5) the 580 g 5" stays at 50 km/h,
speeding up and braking at 2 m/s². At 130 km/h its frame drag (about 9 N)
blows it off. The 29 g Air65 II holds at 20 km/h; at 40-50 km/h its drag
beats its grip. That agrees with the research. Borowczyk et al. (2017)
landed a quad on a car roof at up to 50 km/h. The quadrotor perching
work (the Hitchhiker gripper, 2022; Sherbrooke's friction landing gear)
needs suction cups, grippers or reverse thrust for faster or steeper
surfaces. No air moves with the car: the quad feels the full airspeed,
not the slower air of a roof's boundary layer.

**Reversing it** means going back to interpolating each move over its
interval in `CollisionWorld::moveObject` and applying moves at once in
`fpv_surface_object_move`.

### D41. Perching on what the host's raycast hits

D39 covered surface objects. A game whose world is its own line test (GTA
SA, through `fpv_world_set_raycast_callback`) gives libfpv only ray hits:
each became a plane that stood still until the next tick. On a car at
20 km/h the quad stayed where the roof had been and fell off within a
second; on a parked car it sat still in some spots and bounced in others.
Two causes:

* **Planes didn't move.** A hit had no velocity and no frame: the plane
  froze between ticks and friction had nothing to carry the quad with.
  `fpv_world_set_raycast_callback_ex` lets a hit say where the thing it
  hit is (`FPV_RAYCAST_HIT_POSE`, keyed by `object_id`). Each such thing
  becomes a *mover*, an object with no triangles that follows the same
  D39 path through the poses, timed for the tick's frame
  (`Lib::frameStamp`). Its planes are kept in its frame and ride with it;
  contacts get its velocity and spin. A mover with no pose for 2 s is
  dropped. The `perch_moving_raycast` test runs every `perch_moving` deck
  through the callback only; without the pose a 20 km/h car loses the quad
  at once, with it the results match the surface object's.
* **Planes stood where the surface isn't.** A plane is the surface's
  tangent at one hit point, extended 0.75 m. On a curved roof that is
  centimetres off a few decimetres away: in GTA SA the centre's sideways
  rays met a Greenwood's raised roof rim 0.3 m out, with normals tilted up
  to 27°, and those planes stood 4 cm above the roof under the arms. The
  quad tipped onto them, fell, and kicked up again (2-8 % of frames off
  the roof in a turn, 6-9 cm up). As the car turned, the world-fixed rays
  swept the rim and the planes changed every tick. Two fixes:
  * *Cuts.* A ray that went into a plane's solid side on its way to
    something further proves the plane wrong there. Each plane keeps up to
    8 such crossings (`Contact::cuts`, or where a ray starts behind it: a
    sphere is there) and stands only where it is nearer its own hit point
    than any of them. That's a Voronoi split between evidence for and
    against, with nothing to tune. A flat surface gets no cuts (every ray
    ends on the plane, within 1 cm), and neither does a concave one (its
    tangents are under it). A convex one is cut back to around each hit.
  * *Probes from the spheres.* Near a surface (a hit within the drone's
    size + 15 cm), each sphere but the body's (whose rays are the centre's)
    casts six short rays (down, up and four level ways, plus its own down
    when tilted) of its radius + 12 cm. That's about 54 more rays a tick,
    1-7 µs each in GTA SA. Every support point then has a plane measured
    right under it, and the rays' free stretches give the cuts.

In GTA SA with the perch autotest (gtasa-fpv), the same 20 km/h turn at
15°/s went from 59-92 mm of lift, 11-33 cm of slip and 2-8 % of frames off
the roof to 0 mm, 3.8 cm and none. At 40 km/h the lift went from 126 mm to
1 mm. Parked, on five car models and near the roof's edges, the quad doesn't
move. The tests' decks (flat, a channel, a 1.5 m dome) never made phantom
planes, so they only guard against regressions; the evidence is the
game's.

**Reversing it** means dropping the probe block and the cut loop in
`refreshRaycastProxies` (frame.cpp) and the `standsAt` checks in
`CollisionWorld`; the movers are independent of them.

### D44. DShot ESCs and the RPM filter

The simulated ESCs take DShot (SimITL `0029`). A DShot throttle of 48 to
2047 drives the simulator like PWM's 1000 to 2000. DShot commands stop the
motor while they are sent and go out at Betaflight's pace, as the platform
drivers send them. The ESCs do not act on the commands themselves: turtle
mode still reverses the motors from `isCrashFlipModeActive()` (D21), and
the beeper switch's beeps are libfpv's own (D31).

**Telemetry.** The ESCs report their rpm (SimITL `0021`, D38) only with
bidirectional DShot (`dshot_bidir = ON`), as real ESCs do. With
`dshot_bidir = OFF` or PWM, Betaflight gets no rpm: no RPM filter, no
dynamic idle, and `dshot_telemetry_info` says it is off.

**Defaults.** The first-run setup, every preset and every import set
DShot600 with bidirectional DShot, so the RPM filter and dynamic idle work
without setup, as before. An EEPROM made by an older libfpv (PWM was
forced) gets them once, with the EEPROM upgrade `dshot-bidir-6`. After
that the pilot's choice stays: `ensureEssentials` accepts DShot with or
without telemetry, and PWM. Other protocols (OneShot, MultiShot, brushed)
go back to DShot600. A change of the protocol family or of `dshot_bidir`
reboots Betaflight at the next `ensureEssentials` or import
(`bfb_features_pending`), because the motor driver starts at boot.

**Flight.** On `main` the motors ran on PWM with the rpm telemetry always
on. On DShot with bidirectional DShot the same quads fly the same:
`flight_realism` gives the freestyler-5 126 km/h (125 on PWM), the
`air65_blackbox` idle 4470 rpm (4422).

**Proof that the filter filters.** `fpv_betaflight_debug` (the
`Server::debugValues` call) returns Betaflight's `debug[]`, as the blackbox
records it. The `rpm_filter` test hovers the freestyler-5 for 2 s:

* `debug_mode = RPM_FILTER`: each motor's notch frequency, 163 Hz, against
  the simulated 162 rpm/60 (within 5 %).
* `debug_mode = GYRO_FILTERED`, roll and pitch: 0.00 deg/s RMS with the
  RPM filter, 1.0 to 1.3 deg/s with `rpm_filter_harmonics = 0`. SimITL adds gyro
  noise at 1, 2 and 3 times the motor frequency, and the notches take it
  out. The debug values are whole deg/s, so the filtered gyro rounds to 0.
* `dshot_bidir = OFF`: no telemetry, and the quad still arms and hovers.

**Top speed.** Since the rpm telemetry (D38) the freestyler-5 tops out at
125 to 126 km/h. The gyro at full
throttle, 80 degrees of tilt (GYRO_FILTERED, every 2 ms over 4 s):

| | Top speed | Roll, RMS | Roll |
|---|---|---|---|
| RPM filter on, 2 ms ESC delay | 126 km/h | 54 deg/s | ±68 deg/s at 20 Hz |
| `rpm_filter_harmonics = 0` | 142 km/h | 7 deg/s | noise only |
| `dshot_bidir = OFF` | 142 km/h | 7 deg/s | noise only |
| RPM filter on, 0.3 ms ESC delay | 144 km/h | 0.1 deg/s | none |

So the cap is a roll oscillation at 20 Hz: the notches' phase lag on top
of SimITL's 2 ms ESC delay. Dynamic idle plays no part (the second row
keeps it). With a 0.3 ms ESC delay (proposed in hammerill/libfpv#16) the
RPM filter runs and the oscillation is gone.

## Presentation

### D14. Overlay: libfpv is a "legacy" ImGui backend

Dear ImGui 1.92 supports dynamic font atlases (`RendererHasTextures`), but
that would push texture create/update/destroy requests onto every host
renderer. libfpv does not set the flag. It bakes ImGui's scalable default
font once at 32 px into one RGBA atlas (`FPV_TEXTURE_OVERLAY_FONT`) and
draws it at `16 px × ui_scale`. The host uploads at most two static
textures (overlay font, OSD font) and draws plain indexed triangles, so its
renderer snippet stays about 60 lines. The overlay draws its own mouse
cursor (`io.MouseDrawCursor`), because games usually hide the OS cursor.
No `imgui.ini` is written. ImGui is compiled inside libfpv with hidden
symbols, so it cannot clash with a game's own ImGui.

Overlay values for Betaflight settings are read and written with CLI
`get`/`set` on the simulation thread (`BfSettingsCache`) and refreshed
every second while the overlay is open. Configurator edits therefore show
up, and overlay edits show up in the Configurator after its next read.
Sliders write live and `save` on release.

### D15. OSD

Betaflight Configurator's MAX7456 fonts (`default.mcm` for the analog look,
`bold.mcm` for the HD look) are embedded and parsed at runtime into one
384×288 atlas. `fpv_osd_draw_data` returns quads in the same draw-data
format as the overlay, so a host needs only one renderer.
`fpv_osd_render_rgba` rasterises on the CPU for hosts that prefer a
texture. SimITL's OSD canvas is the 30×16 analog grid. "HD look" uses the
bold font in a 16:9 viewport; it does not add an HD (53×20) canvas.

**NTSC shows 13 rows (2026-11).** A real OSD chip fits 13 rows of 18-line
glyphs in NTSC's 240 field lines, and Betaflight's displayport says so:
elements below row 12 move up to it at boot. libfpv used to draw 16 rows on
every camera, so an NTSC picture got its glyphs cut or squeezed (D34).
Betaflight's video system now follows the quad's camera: `vcd_video_system`
is NTSC for an analog NTSC camera, PAL otherwise (digital keeps the 16-row
canvas), set by the first-run configuration, a diff import (which starts
from `defaults`) and a check whenever a quad is activated or its camera
changes, which reboots Betaflight when it changes (SimITL patch `0019`
gives the displayport its rows). The first-run layout has an NTSC
version (the bottom elements two rows up, the middle ones one), and a
layout still exactly libfpv's default for the other system is swapped for
it; a player's own layout is left to Betaflight, which moves what falls
below row 12 onto it, as a real flight controller does. The F8 OSD editor
and `fpv_osd_draw_data` (the same grid as the video feed draws it) follow
`osdVisibleRows()`, and so does the kamikaze build's centred OSD.
`frame.osd` stays 30×16: rows 13 to 15 are blank on NTSC.

**Three OSD profiles (2026-11).** SimITL's target built Betaflight with
one OSD profile, so a real quad's diff with elements in profiles 2 and 3
lost them and `osd_profile` did not exist. It now has three (SimITL patch
`0025`), with the EEPROM layout kept (Betaflight patch `0019`): an
element shows in the profiles whose visibility bits (11 to 13) it has,
and the active one is `osd_profile`, switched by the CMS, the
Configurator, an OSD profile adjustment on a switch or the F8 OSD tab's
profile buttons. The tab edits the active profile, so its live preview
is what the pilot sees. libfpv's CLI `save` does not reboot, so
`bfb_cli_run` applies a changed `osd_profile` itself
(`changeOsdProfileIndex`), as the reboot does on a real quad. libfpv's
first-run layout stays in profile 1.

*Only where the camera preset asks (2026-11).* Making every NTSC camera
13 rows was wrong for most quads: the Freestyler and the Nomade (Caddx
Ratel 2) lost their bigger 16-row layout. NTSC's 13 rows are now a camera
preset's `"osdRows": 13` (only the BetaFPV C03, the Air65 II's, whose DVR
D34 measures); every other camera keeps 16 rows and Betaflight's video
system PAL, NTSC picture or not (`presetNtsc()`). The feed tells the
renderers in `fpv_video_feed.osd_rows`; a renderer reading a libfpv older
than it keeps 13 rows on NTSC.

### D15b. RC packets at a link's rate; a fast Configurator (Betaflight patch `0011`, SimITL patch `0018`)

SimITL's receiver used to answer "new frame" on every scheduler pass.
Betaflight runs one non-realtime task per pass after the gyro and PID, so
its event-driven RX task took about 6000 slots a second (three states per
frame) and starved everything else. Measured per simulated second:
OSD 12 calls (0 to 3 finished frames, a full redraw every 0.3 to 0.5 s),
SERIAL (MSP) 33 instead of 100, ATTITUDE 50 instead of 100.

- RC packets now arrive at a packet rate, 500 Hz by default (ELRS
  freestyle; `SimITL::BF::setRcPacketRate`). Each packet is reported once
  and Betaflight keeps the last sticks between packets, like a real
  receiver, which adds 0 to 2 ms of realistic RC latency. Every task now
  runs at its rate, and the OSD redraws at `osd_framerate_hz` (Betaflight's
  default 12 Hz), like a real quad's.
- A lost packet (radio lost, or the link simulation's loss hook,
  `setRcPacketLossHook`) is reported as `RX_FRAME_DROPPED`. Betaflight
  patch `0011` keeps a pass with no packet from counting as a lost one, so
  Betaflight's own link quality is that of the packets.
- The Configurator sends one MSP request, waits for the reply, then sends
  the next, so the round trip sets how fast it navigates. It was 20 ms
  (measured with the RX fix already in): the websocket thread slept 10 ms
  between `lws_service()` calls, and a request waited for the next run of
  the SERIAL task (100 Hz). Now `wsWrite()` wakes the websocket thread with
  `lws_cancel_service()` once per reply (the thread sleeps only while no
  server runs), and simulator builds run the SERIAL task at
  `serial_update_rate_hz` or 1 kHz, whichever is higher. The setting and
  everyone's EEPROMs stay as they are. A round trip is now 1 ms, at about
  4 % of one core for the whole simulation.
- With the RX task at a real packet rate, Betaflight's stick commands work
  as on a real quad. Before, the RX task's interval rounded to 0 ms, so
  the 50 ms hold never added up. Disarmed in ANGLE mode, full throttle plus
  pitch or roll trims the accelerometer and saves; throttle low with yaw
  left, pitch down and roll right saves; and so on.

### D16. Audio

`fpv_audio_read` is called from the host's audio thread. It never takes
the API lock: parameters (RPM, load from current, prop damage, pan from
motor position) are copied under a small mutex once per host frame and
glided over 15 ms. Each motor mixes a band-limited blade-pass saw
(harmonics above Nyquist dropped), the motor's electrical whine (7 pole
pairs), RPM-tracking low-passed noise scaled by load, and a
once-per-revolution wobble from prop damage. Betaflight's beeper is a
2.7 kHz tone. The output is soft-clipped with `tanh`. The beeps and the
hearing from the pilot are D31.

### D17. Motor state is cleared on spawn and despawn

SimITL's `Reset`/`Repair` commands keep the motor model (RPM, ESC filters,
2 ms delay line). A drone deleted at hover throttle came back with
spinning props, popped up 5 cm and tripped ANGLE (found by test ordering).
`Flight::stopMotors` clears that state.

### D18. Shipped presets use Betaflight's stock tune

SimITL's per-quad `.bin` EEPROMs load fine (version 178), but differ from
defaults only in `feature -GPS -TELEMETRY`: there are no modes and no
tune. So libfpv ships no per-preset tune. The `<preset>.diff.txt`
mechanism (D7) is there for preset authors.

### D31. Hearing the drone from the pilot, and two beepers

**Beeps.** Betaflight's beeper output is split by its reason. While the
pilot's BEEPER mode switch is on (`bfb_beeper_switch`) the beeps come from
the motors, as DShot beacon beeps do on a real quad (a 1.76 kHz tone through
the windings), and follow the motor volume. Every other beep (arming, low
battery, failsafe) is the 2.7 kHz piezo buzzer, with its own volume. So
muting the beeps never silences the lost-drone finder.

**Where it is heard.** The player picks Auto, the drone or the pilot (F8 >
Camera & sound). Auto hears it onboard in the FPV and chase views and from
the pilot in the LOS and pilot views; the game reports its view with
`fpv_set_view` (FPV_VIEW_PILOT is the F key, D33). Onboard is the sound as
before. The two are always both computed and cross-faded over 150 ms, so
switching views does not click.

**From the pilot.** The ear is the pilot's head (1.6 m above
`pilot_position`, turning to follow the drone) or the game's listener
(`fpv_audio_set_listener`, its camera in the pilot view). The model, from
the literature on outdoor sound propagation:

- *Travel time and Doppler.* The drone's mono signal goes through a delay
  line (2 s, 690 m). The read point is the emission time,
  `tau = r / (c + dr/dt)` with c = 343 m/s, so its change gives the moving
  source's shift `f c / (c - v)` exactly (checked: 1760 Hz at 40 m/s gives
  1993 / 1576 Hz approaching / receding, theory 1992 / 1576). The host sets
  tau once a frame, a staircase: the read point follows it through three
  one-pole stages of 40 ms, per sample. Following each step within one
  audio block instead (the first version) swung the pitch by 15 % at
  25 fps, heard as a wobbly drone; now the ripple is under 0.3 %
  (`fpv_synth_tests`: a drone receding at 20 m/s, hearing set at 25, 30 and
  60 fps). The lag, about 0.12 s, only delays the shift. It moves at most
  half a sample per sample; a jump of 17 m or more between two updates is a
  teleport and is taken at once.
- *Spreading.* 1/r (6 dB per doubling), with the onboard level equal to the
  level 2 m away, so a fly-by of the pilot matches the onboard sound.
- *Air absorption.* About proportional to the frequency squared (ISO 9613-1
  at 20 C, 70 % humidity: about 5, 10, 33 and 117 dB/km at 1, 2, 4 and
  8 kHz), so the highs go first: a one-pole low-pass at the frequency where
  the loss is 3 dB, `sqrt(3 / (2e-9 r))` (12 kHz at 10 m, 4 kHz at 100 m,
  1.7 kHz at 500 m).
- *Walls.* Rays from the ear to the drone, 10 times a second, through the
  drone's collision world and the game's raycast; hits within 0.5 m of the
  last one are the same wall (both faces), up to 4. Sound gets past an
  obstacle two ways, both losing the highs first: through it (the mass law,
  TL = 20 log10(f m) - 47 dB, so a wall passes the bass) and around its
  edges (Maekawa's barrier diffraction, 10 log10(3 + 20 N) dB with the
  Fresnel number N = 2 delta / lambda, capped at about 25 dB in practice).
  libfpv does not trace the edges; it takes the result: 12 log2(1 + n) dB
  for n walls (12, 19, 24, 28 dB) and a 12 dB/octave low-pass at
  20000 / (1 + 11.5 n^1.5) Hz (1.6 kHz, 620 Hz, 340 Hz, 220 Hz). A 5"
  quad's blade-pass tone is around 1 kHz, so behind one wall it is muffled,
  behind two only its rumble is left.
- *The ground.* The reflection off the ground under the ear (an image
  source), at half the level: a copy a little later, which comb-filters the
  direct sound and sweeps as the drone climbs or comes closer, the familiar
  "phasing" of a fly-by. Only below 1.5 kHz (a one-pole low-pass on the
  reflection): grass, dirt and asphalt are rough at the highs' wavelengths,
  so their reflection does not add up coherently. Unfiltered, the whole
  sound was comb-filtered into a flanger. Its delay is smoothed like the
  direct sound's, and so is the slap echo's.
- *The game's own sounds.* `FPV_DRONE_HEARD_FROM_PILOT` in `frame.flags`
  says the drone is heard from the pilot, so a game can hear everything
  else from there too (its listener at `frame.los_camera`). Then the whole
  scene sounds as it does to a real pilot whose drone is far away, which
  the drone alone from the pilot, over the game's sounds at the drone,
  does not.
- *Echoes and reverberation.* Six horizontal rays and one up, 40 m, around
  the ear (or the drone, when heard onboard). The share of directions with a
  wall gives a slap echo (there and back to their mean distance, weaker
  farther away); a roof makes it a room. The reverberation is a Schroeder /
  Freeverb network (four damped combs, two all-passes) with RT60 0.3 s plus
  0.04 s per metre of mean room size indoors, 0.6 s outdoors. Its send falls
  with the square root of the distance while the direct sound falls with
  the distance, because a room's diffuse sound hardly depends on where the
  source is: far drones sound wetter, the main distance cue indoors.
  Onboard, a quarter of the send: a room around the drone is audible, but
  the props are close.

- *The two ears.* The output is stereo (`fpv_audio_configure(rate, 2)`).
  The drone's direction in the ear's frame (the listener's rotation, or
  the pilot's head) gives its side and whether it is ahead or behind.
  The pilot's head, with no listener, turns after the drone with a 0.3 s
  lag (a first-order follow of its yaw, in simulated time), as a real
  pilot's head does: a hovering drone is in the middle, a fast pass is
  heard crossing from one side to the other before the head catches up,
  and when it flies straight overhead the head turns round. Three cues,
  all on the direct sound (with its ground reflection and echo):
  - an equal-power pan, narrowed to 80 % so the far ear keeps -13 dB, as a
    real head lets the lows round to it;
  - the interaural time difference, Woodworth's `(a / c)(theta + sin theta)`
    with a head radius of 8.75 cm: up to 0.66 ms, the far ear reading a
    short delay line (fractional, linearly interpolated). The delay passes
    from one ear to the other through zero, so a crossing drone never
    jumps;
  - head shadow: the far ear loses up to half of what is above 2 kHz (a
    one-pole high-shelf cut, `x - k (x - lowpass(x))`). Behind the head
    both ears lose 40 % of the highs and the level 2 dB (the pinnae face
    forward): the front/back cue, which equal-power panning alone cannot
    give.

  The side and the front glide over 60 ms per audio block and are
  interpolated per sample within it, so moving them does not zipper
  (`fpv_synth_tests` checks the largest sample step while the drone jumps
  round the head stays the tone's own). Within 1 m of the ear the side
  fades out, so a listener at the drone (its own camera) hears it centred,
  like onboard. Onboard the left motors are a little left and the right ones
  a little right (30 % of the way, was 70 %: the arms' rpm differences
  pulled the whole drone a few dB to one side), so a damaged prop is heard
  on its side and the drone stays centred. With one channel there are no
  sides (the ears' delay would only comb-filter the sum); the behind cue
  stays. The reverberation's two outputs are two orthogonal sums of its
  combs, each through its own pair of all-passes (Freeverb's stereo spread,
  23 samples): one chain tapped after one and after two all-passes made the
  room 3 to 4 dB louder on the right.

Probing costs nothing until the host pulls audio (`fpv_audio_read`).
Directivity (the rotors are louder below), the pinnae's elevation cues and
HRTFs are left out. Sources: ISO 9613-1:1993; Z. Maekawa, "Noise reduction by
screens" (1965); the mass law of sound insulation; and quadcopter noise
measurements (blade-pass harmonics dominate up to about 6 kHz, broadband
above).

Reversing: `hearsFromPilot()` returning false is the old sound.

### D33. The pilot: the F key, the stand-in and the piloting pose

Games with a character get a standard F key (INTEGRATION.md "The pilot"):
the view goes back to the character while the drone stays in the world,
radio included, and F7 goes back to that drone instead of launching a new
one. libfpv treats that view like LOS (FPV_VIEW_PILOT): it is heard from the
pilot, and `fpv_drone_set_pilot_position` follows a character who walks.

The stand-in (the character carried with a far drone, a copy left at the
piloting spot) and the piloting pose are game code, but the player's choice
of them is libfpv's, so it is the same in every game and saved with the
other settings: `fpv_pilot_set_support` says what the game does and its
default distances, the overlay offers only those (F8 > Fly > Pilot, on by
default), and `fpv_pilot_get_config` reads the choice. Turning the stand-in
off keeps the real character at the spot, at the cost of the world loading
less far from it.

The pose needs to know that the pilot is flying. `FPV_DRONE_RADIO_ACTIVE` is
set while armed or for 3 s after any channel moved by more than 0.05 of its
range: noisy AUX pots and switches jitter by about 0.01-0.02, and the
threshold is the same for every game.

The pose is for the drone's views only, not the pilot view. Posing the
character again there whenever it stood still (as first advised) fought the
game: its own idle moves broke the pose, and the game had to suppress them.
In the pilot view the character is the game's, as if there were no drone,
and the radio in the player's hands is enough.

### D34. The video feed: a link budget, line-by-line fading, the goggles' raster

The old effect was one number (noise from range and one occlusion ray)
painted as uniform per-pixel static, which looks like no real feed. Real
analog FPV is FM video, and its look follows from that. Above the FM
threshold (CNR ~10-13 dB) the picture is clean. Below it come impulse
clicks ("sparklies"), then colour speckle and loss of the colour burst,
then hits on the sync pulses, then lost vertical sync and snow.
ImmersionRC measured first snow at about -94 dBm and lost sync at about
-106 dBm on a good receiver. libfpv measures CNR against a -104 dBm floor,
so those two points fall at +10 and -2 dB.

**Simulation thread (`Flight::updateVideo`).** The link budget:

- the VTX's power and a 7 dB system loss (polarisation, connectors,
  detuning), which puts a 25 mW VTX's first sparkles near 1.4 km in the
  open with the antenna in clear view, a 400 mW one's past 4 km;
- free-space loss at 5.8 GHz;
- the two-ray ground loss past `4 h1 h2 / λ` (0.7 of 40 dB/decade, rough
  ground): flying low and far is what kills range;
- a dipole VTX antenna up and back on the frame (its null along the axis,
  so flips that point it at the pilot dip), shaded toward the front by the
  frame, the stack and the battery (the frame shadow, below);
- a damaged VTX's loss: up to 20 dB, from hard hits (over 5 m/s) and
  `fpv_drone_damage_props` (fire, blasts), until the drone is repaired, and
  only with damage on. The impact jolt (the feed's) comes with every hit,
  damage on or off;
- the goggles' patch plus omni (weak right overhead).

**The frame shadow.** The VTX antenna is mounted at the back of the frame,
up and tilted back, which is what build guides recommend: carbon fibre and
a LiPo block 5.8 GHz, so the antenna is put where most of the sky behind
the quad is clear. The price is the other direction: toward the front the
signal goes through the plates, the stack, the camera and the battery.
Pilots know the pattern: the feed holds on the way out (the pilot behind
the quad, its nose tilted down, the antenna nearly vertical) and breaks up
on the way back, nose toward the goggles, especially when the quad comes
back low and level. `txAntennaGainDb` takes the shadow off the dipole's
gain: all of it straight ahead, about a quarter side-on, none behind, and
none for directions well above the top plate, where the antenna's tip
looks over the frame. Its size is the quad's `vtxAntennaShadowDb`, by
default from the arm length (about 4 dB on a whoop, 8 on a 5", 10 on a 7"
with its bigger pack). Vendor guides quote up to 20 dB through carbon; that
figure is unverified, so libfpv stays at about half of it. The frame and
battery used to be an all-round part of the system loss (10 dB, now 7):
the average over directions is about the same, but long range now plays
like the real thing. The way out is easy, and the way back needs height
(the pilot then sees the antenna over the top plate) or margin.

Obstruction uses rays spread over a 20-tick cycle. The direct ray is cast
from both ends, which gives the obstacle's depth (12 dB plus 0.6 dB/m,
capped at 45 dB). Four paths bend through the edge of the first Fresnel
zone, so the signal fades as the drone slides behind a corner instead of
switching off. Blockage also lowers the Rician K factor: behind a wall only
scattered paths remain (Rayleigh, deep fades). The thread reports the mean
received power, K and the Doppler `|v| / λ`.

Those rays only see libfpv's own collision world (static surfaces and
objects). A game whose world is reachable only through the raycast
callback (GTA San Andreas: its own collision, streamed) would leave every
building out of the link, a clean picture through a city at 800 m. So
`fpv_drone_tick` casts the same seven rays through the callback on the host
thread, one per call (the pilot's end starts 0.6 m out, past his own body
or car), and hands the loss and K to the simulation, which keeps whichever
blocks more.

**Host thread (`VideoFeed`).** It runs once per field (59.94 NTSC, 50 PAL,
the mode's frame rate for digital) on the host's clock. It generates
the fast fading line by line: Jakes' sum of eight sinusoids per antenna
with a direct path of K times their power, and selection diversity with
hysteresis. At 20 m/s a fade lasts ~40 lines, which gives the horizontal
bands of noise real feeds have, sweeping faster the faster the drone flies.
Slow log-normal shadowing (2.5 dB, 0.4 s) comes on top. Each line's CNR
maps to:

- grain from the FM improvement (video SNR ≈ CNR + 26 dB);
- a click density from Rice's rate `B / (2√3) · erfc(√CNR)` (B = 18 MHz
  IF);
- colour gain and hue jitter;
- a horizontal sync kick that the line PLL pulls back over the next ~10
  lines (the classic torn edge);
- the antenna switch's glitch, visible only on weak lines.

Per field it decides the colour killer (with hysteresis), vertical lock
and the roll, impact jolts, brownouts (cell voltage below the VTX's
`minCellVoltage`: the VTX resets for 0.4-1 s), the ESC's 48 kHz PWM
(`interference`: 2.5 cycles a line, stepping 0.05 a line, so diagonal
stripes that crawl) and the motors' electrical hum bars, both scaled by
throttle and the VTX's `powerNoise`. The result goes to the host as a
2 × lines RGBA8 table in the frame (no texture API: a frame is a
per-drone, thread-safe copy) plus a few scalars. Fades, sync and clicks
are all decided here, so every renderer shows the same feed, and the
shader only draws a table.

Digital links use a margin above `sensitivityDbm`. HDZero breaks into
macroblocks around 0 dB and holds below -8 dB. DJI and Walksnail
(`freezes`) lose bitrate first (mush), then stutter and hold frames.

**Rendering (`fpv_video_gl3.h`).** Two passes. The signal is drawn at the
feed's raster: an analog field is 720 × 240/288, the 720 samples a goggle's
decoder takes, and luma is limited to the camera's TVL through a 5 MHz
baseband, about 250 cycles across, so every "1200TVL" camera looks about
400 TVL through the link, as on real goggles. It is redrawn only on a new
field, so it runs at 60 Hz whatever the game's rate, and the previous one
is the history for digital freezes and stale blocks. The goggles then
scale it bilinearly into the camera's aspect with black bars, bobbing the
interlaced fields half a line. On analog the OSD is drawn into the signal,
because the flight controller's OSD chip writes into the video before the
VTX, so it tears, rolls and drowns in snow with the picture. Pixelation
with nearest filtering was rejected for the picture: goggles interpolate,
and blocky pixels read as a retro game, not a video feed. The scaler keeps
each step between two samples to about a screen pixel ("sharp bilinear")
only where the OSD is, so its pixels stay crisp, and is plain bilinear
over the picture: the signal pass writes 1 to alpha on samples the OSD
covers, and the scaler reads the alpha once, filtered, at the centre of
the four samples it blends, which sees a mark on any of them. (It used to
be sharp over the whole picture, which at 1440 × 960 drew every field
line as a flat band four pixels tall with a hard step to the next: the
picture looked hard and blocky, which no goggles or DVR shows.) In both renderers
(`fpv_video_gl3.h`, `fpv_video_gl2.h`) the signal shader is built as three
programs, one per mode (lens only, analog, digital), not one that branches
on the mode: as one, ANGLE's Direct3D 11 backend (Chrome and Edge on
Windows) hands FXC a shader too big for it, and the browser's GPU process
dies of a stack overflow while compiling it (found on an Intel UHD laptop,
2026-11; each mode alone compiles in about 0.1 s). If a program still
fails, init logs the reason and returns 0, and the host draws the FPV view
without it (the standalone example and sm64ex-fpv: their plain lens pass and
the OSD). After a GPU process crash the WebGL context is lost and stays lost,
so a fallback can't render either: in Emscripten builds even state queries
such as `glGetVertexAttribiv` throw on it (sm64ex-fpv's page crashed with
"Cannot read properties of null (reading 'length')"), which is why the
shader itself has to stay small enough.

Both effects are on by default (settings `videoNoise`,
`videoResolution`; first off, turned on by default in 2026-11 so new
pilots see the feed as goggles show it), and offered only to games that
pass `FPV_INIT_VIDEO_FEED`. It is opt-in, unlike the lens's opt-out
`FPV_INIT_NO_LENS_DISTORTION`, because the feed came after the games: an
opt-out would have shown settings that do nothing in every integration
that has not ported the renderer yet. Presets name real parts: the Freestyler is a Caddx
Ratel 2 (4:3, NTSC) on a TBS Unify Pro32 HV at 400 mW, and the Air65 II a
BetaFPV C03 on its Matrix 1S AIO VTX at 25 mW (1S brownouts).

Reversing: both settings off is the old picture without its static. The
old `fpv_video_state` fields remain, derived from the new budget.

**Calibrated against a DVR (2026-11).** A five-minute DVR of an Air65 II
(25 mW, C03 camera, NTSC) flown around and inside a parked car, recorded
by Skyzone Cobra X V4 goggles, put next to the same conditions through the
model (a still from the DVR as the scene, the link scripted), showed five
things the first version got wrong. The goggles' look follows the Cobra X.

- *Never noisy up close.* At a few metres a whoop's carrier is 50-60 dB
  over the noise, so even a 30 dB fade stayed clean, while the real feed
  glitches near the pilot: a torn, sparkling band for a field, now and then,
  much more often among reflections (inside the car). That is multipath
  distortion: when echoes cancel the direct path, the FM demodulator's phase
  jumps (clicks) whatever the carrier's strength. Each line's CNR now adds,
  in power, a distortion `kVideoMultipathMarginDb` (17 dB) under the
  selected antenna's fade, so it is reached only in deep fades. At -50 dBm
  and 5 m/s: with K = 3.3 (a whoop 1 m over the ground) 1.6 % of the fields
  sparkle and the colour is thrown off every 7 s; among reflections (K = 1)
  7 % and every 2 s; with a clear line of sight (K = 10) hardly ever (none in
  two minutes). It drives clicks and sync kicks, not
  the FM grain, the antenna switch's glitch, the colour killer or vertical
  sync, which follow the thermal CNR (the burst and the vertical sync are
  long, averaged pulses that ride it out), so the picture never rolls or
  goes grey up close.
- *Colours too tame.* On a weak or hit signal the real decoder's colour goes
  wrong, not away. The colour killer now acts only once the burst is really
  gone (field CNR under -6 dB, back over -2), colour speckle is finer and
  stronger, and clicks come mostly in the bright parts of the picture
  (whites sparkle first; dark grass stays clean). A click is a speck a
  sample or two long, as often coloured as dark, not the comet with a
  fading tail it was: lone black dashes over a clean sky are not in the DVR.
  The goggles show clicks in dense bands, so a line with few of them
  (under ~1.5 per line) keeps fewer still, and the odd lone one is gone. A band of hit lines (25 of them two times in three), a hit or a
  surge throws the decoder's subcarrier and colour gain off: for 2-5 fields
  the colour is oversaturated (`chroma_boost`), turned (up to half a turn,
  drifting down the picture), sweeping across the line (`hue_sweep`) and the
  highlights tinted (`chroma_tint`), then it locks again within ~0.1 s, and
  not again for 0.25-0.75 s. The DVR shows exactly that after a glitch
  inside the car: red LEDs turned magenta, then a neon green and purple
  picture for four or five frames.
- *Dark lines across the picture.* The ESC's diagonal stripes and the
  motors' hum bars were on for every analog VTX and crawled over the whole
  picture. The DVR has none: a decent VTX filters its supply. They now
  start above `powerNoise` 0.35 (the cheap VTX keeps them).
- *Surges and hits.* A crash flip or a punch on a whoop's 1S supply tears a
  band of lines sideways for a field: the line PLL holds the offset over the
  band (it decays 0.5 % a line, not 14 %), and a line shifted past its end
  shows the black porch and then its own start, as the DVR's crash flip
  does. The chance follows the throttle's rise per second and `powerNoise`;
  a hit makes one band, not one per field while the jolt decays.
- *A soft picture and a mangled OSD.* The luma low-pass was twice too soft
  (its -6 dB point sat at half of `luma_cycles`), so edges now rise over
  about two samples like the DVR's. (The halos first added with it, the
  camera's `sharpening` plus a fixed goggle peaking, 1.56 times the edge
  for the C03, overshot each edge by 35 % on both sides and made the whole
  picture painfully sharp; see the soft-bleed entry below.) The OSD was drawn at 3 % / 4 % margins into a linear 640 × 480
  texture and then through the camera's filters: 16 rows of 18 glyph lines
  squeezed onto 13.8 field lines each, then blurred and sharpened, so
  glyphs lost rows and grew coloured garbage at the corners. It is now built
  the way the OSD chip (MAX7456 / AT7456E) writes it: 30 × 16 cells of 12 ×
  18 font pixels, one field line per glyph row, switched in after the
  camera (the picture's colour off under it), softened only a little by the
  link, with faint colour dots where its steps ring in the colour filter.
  The DVR's cells are 24 samples by 18 field lines, the 30 columns across
  the whole line (it starts 9 samples in, which cuts column 29), and NTSC
  shows 13 rows: 234 of the 240 lines.
  libfpv first squeezed its 16 rows into NTSC by cutting glyph rows, which
  cut icons (the battery's top, the LQ bars) and drew the glyphs too tall
  for their width; NTSC now has 13 rows like the real thing (D15), every
  glyph complete, PAL its 16 rows in 288 lines. The grid is looked up at
  each field's own frame line (interlaced, as the chip writes it) between
  the four nearest font pixels, each step kept to about a sample or a
  line, so it can take a scale across (`S.x`) and down (`S.y`) with every
  glyph row kept and the edges crisp.

  *The DVR's aspect (2026-11).* A scale of 0.95 around the centre (a margin
  all round) made the OSD look too tall next to the DVR. The DVR records
  720 × 480 with no aspect flag, so players show it at 3:2, a font pixel
  square: in its frame the cells are exactly 24 × 36 pixels (the chip's
  2 samples by 1 field line, unscaled), column 0 starts 9 pixels in
  (column 29 is cut at the right edge) and NTSC's row 0 4 frame lines
  above the top (a "0"'s white starts on line 402 of row 11). The grid is
  now drawn exactly that way (`S` = 1, 1), and a camera preset's
  `displayAspect` (3:2 on the BetaFPV C03 and the Caddx Ratel 2) sets
  `fpv_video_feed.display_aspect`: the renderers fit that rectangle and
  stretch the camera's 4:3 picture into it, keeping its field of view, as
  the DVR does. Measured on a libfpv screenshot scaled to 720 × 480, every
  glyph edge lands within a pixel of the DVR's. Without `displayAspect` the
  picture stays at the camera's aspect, the OSD the same grid. The field
  crosses the client/server wire, so its protocol is version 2.

  *Only on the Air65 II (2026-11).* The 3:2 picture and 13 rows suit the
  Air65 II's DVR, not the Caddx Ratel 2's quads, which show their 4:3
  picture again with all 16 rows (D15): `displayAspect` is now only on the
  BetaFPV C03. 16 rows (288 glyph lines) can't fit NTSC's 240 lines
  unscaled, so the grid is scaled to 0.95 of the picture across and down
  around its centre (a glyph row 1.58 frame lines, a font pixel 1.9
  samples), still looked up between the four nearest font pixels, so every
  glyph row stays and the edges stay crisp; `fpv_osd_draw_data` draws the
  same. 13 rows and PAL keep the chip's unscaled grid. The camera's
  `osdRows` crosses the wire: protocol version 3. The chip's white sits over the picture's
  peak (110 %), so the strokes clip to white as the DVR's do. No renderer draws the
  in-signal OSD with the host's OSD renderer any more: they build it from
  `frame.osd` and `fpv_texture_get(FPV_TEXTURE_OSD_FONT)`.
- *Coloured glyphs.* In the DVR some glyphs come out a solid colour that
  flips to its complement every field: a green "4" then a magenta one, a
  magenta "mA" then a green one, while wide glyphs ("0", "8") stay white.
  It is NTSC cross-colour: the goggles' decoder separates colour with a
  notch, and the OSD's one-pixel strokes sit near the subcarrier's period,
  so they demodulate into a hue set by where they fall against the
  subcarrier, which turns half a cycle a field. The renderers demodulate
  the OSD's luma against the subcarrier over ±4 samples (σ 2.5) near a
  glyph and add 0.35 of it. Under noise the glyphs get tinted in patches on
  top (the noise's colour, three times stronger over the OSD's strokes).
- *Soft bleed, not halos (2026-11).* The picture's sharpening was then
  fitted to the DVR's strong edges (VID00013: 6,800 steps of over 110
  levels between flat sides, averaged). The real edge rises from 10 % to
  95 % over three samples with no halo worth the name, and a soft bleed of
  about 10 % reaches six samples out on both sides (the lens and the
  sensor). The low-pass alone already matched the rise; the sharpening on
  top (1.56 × the edge for the C03) undershot and overshot every edge by
  35 %, which made the whole picture hurt to watch. The luma is now
  sharpened by 0.6 × `sharpening` (0.36 for the C03) and mixed with 0.2 of
  the colour filter's wider blur (σ ≈ 5 samples, no extra taps), the
  closest fit to the DVR's edge. The OSD is switched in after this filter, so it
  stays crisp.

**Direct3D 9 (`fpv_video_d3d9.h`).** GTA San Andreas FPV had its own HLSL
port of the GL3 shaders, so every renderer change meant porting it again in
the mod. The port now lives in libfpv next to the GL headers, with what the
mod needed: shaders compiled by `d3dcompiler_47.dll` on any thread (about
1 s, which the mod spends on a thread at startup), the game's UI kept in
the picture as an optional premultiplied texture, the device's state saved
with a state block, and `FPV_VIDEO_D3D9_TEXTURE_GET` /
`FPV_VIDEO_D3D9_OSD_DRAW_DATA` / `FPV_VIDEO_D3D9_LOG` for a game that
loads libfpv at runtime. A renderer change is now made in libfpv's three
headers, and the mod only rebuilds. HLSL reserves `line`, so that shader
says `ln`.

Under all of it there is now the camera's and the receiver's own fine grain
(0.012), which the DVR shows even with the drone lying still a metre away.
The new fields sit after `lines`, so an older host's `struct_size` leaves
them out, and the renderers check that they are covered (an older libfpv:
no upsets). The Cobra's habit of brightening the picture while the OSD is
hidden (no OSD white to keep room for) is not reproduced.

### D35. The About tab: what the binary was built from

A bug report from a player has to name two builds: libfpv's and the game
integration's. F8 > About (the last tab) shows both, plus a "Copy build
info" button that puts the same lines on the clipboard as plain text.

**libfpv.** `cmake/BuildInfo.cmake` runs as a custom target on every build
(not only at configure time: a commit made without re-running CMake would
otherwise show the old hash) and writes `generated/build_info.cpp` only when
its text changed, so an unchanged tree recompiles and relinks nothing. It
records the version, the full commit hash, a dirty flag (tracked files
only), the commit date, and the commits of Betaflight and SimITL with the
number of libfpv patches on top. Those two submodules are always changed by
the patch series, so they get the patch count instead of a dirty flag. A
commit counts only from a repository rooted exactly at that folder: libfpv
vendored inside a game's repository would otherwise report the game's
commit. Without git it reads "unknown". The build date has day precision,
UTC (or `SOURCE_DATE_EPOCH`), because a finer one would relink libfpv on
every build. "Built for" comes from the compiler's own macros (OS,
architecture, compiler and version, the input backend) and the CMake
configuration (`LIBFPV_BUILD_CONFIG=$<CONFIG>`). Build options that are kept
private don't appear there.

**The integration.** `fpv_global_config.host_name`, `host_build` and
`host_url` are added after `flags`, so an older host's smaller
`struct_size` leaves them NULL. They are copied during `fpv_global_init`.
The section is hidden when all three are empty, as in the standalone
example, which is the integration itself. libfpv doesn't parse `host_build`:
each game builds it from whatever its own build system knows.

The tab ends with the credits and licenses: libfpv's GPL-3.0-or-later and
what it means for mods, Betaflight, SimITL, Dear ImGui, SDL3 (native),
libwebsockets (native), {fmt} and nlohmann/json.

### D36. The RC link: ExpressLRS packets, LQ and dynamic power (`src/shared/rc/rc_link.*`, patch `0013`)

Before this the radio link was perfect unless the radio was unplugged
(`radioLost`). The option (off by default) simulates
an ExpressLRS link packet by packet. A real link degrades through stale
sticks and a falling LQ before it fails, and that is what pilots learn to
read.

**The budget** (`Flight::updateVideo`, every step, for the next step's
packets): TX power + 2 dBi (the radio's dipole, at the pilot's chest, 1.3 m)
+ the receiver's antenna − free-space loss − the two-ray ground loss (both
at the band's wavelength, the video link's functions) − obstruction − 8 dB
system loss (polarisation, the carbon frame detuning, hands round the
radio). The receiver's dipole sits at the back of the frame pointing back
and down. Its doughnut null is along that axis, and the frame, battery and
stack shadow it by up to 8 dB at 2.4 GHz (3 dB at 900 MHz) when the pilot
is ahead of the drone. 900 MHz bends round obstacles of the size of the
frame and a body, about 3 to 5 dB better than 2.4 GHz in the off-body
measurements. With these numbers, 100 mW at 2.4 GHz 500 Hz reaches the
sensitivity limit at about 4.5 km in the open (drone 30 m up). Fades take
packets before that, and pilots report a few km.

**Obstruction** reuses the video link's rays (D34), both the simulation's
own world and the host raycast probe in `fpv_drone_tick`: the direct ray's
blockage, the four Fresnel-zone paths and the obstacle's depth. Only the
cost is per band. At 2.4 GHz it is 8 dB plus 0.35 dB/m of depth, capped at
35 dB. At 900 MHz it is 4 dB plus 0.17 dB/m, capped at 25 dB. A wall at
2.4 GHz costs 2.5-4 dB for drywall, 3-8 for brick and 10-22 for concrete;
at 900 MHz about half (1-2, 2-3.5, 10-12). Vegetation is 0.34 dB/m at
2.1 GHz and 0.17 dB/m at 950 MHz (ITU-R P.833), which matches Weissberger's
~10 dB vs ~7.5 dB through 20 m of trees. The caps follow P.833's maximum
attenuation (26-34 dB). The Fresnel offsets are at the 5.8 GHz zone's
radius, narrower than 2.4 GHz's (1.5×) and 900 MHz's (2.5×), so partial
blockage at the lower bands is a little under-counted. That is the price
of sharing the rays.

**Packets** (`RcLink`). SimITL's receiver delivers one packet per interval
at the mode's rate (`setRcPacketRate`, patch SimITL `0018`) and asks libfpv
for each one (`setRcPacketLossHook`). The packet sees the mean received
power plus fading: a Gauss-Markov sum of scattered paths with the Doppler's
coherence time (Clarke, `0.423 / f_d`), the direct path on top with the
probe's Rician K, and 2 dB of log-normal shadowing over 0.5 s. Its chance
of arriving is a logistic of the margin over the mode's sensitivity limit,
84 % at the limit, 99 % 2 dB above, under 1 % 4 dB below. LoRa's
packet-error waterfall is about that steep. A lost packet reports
`RX_FRAME_DROPPED`, so Betaflight keeps the last sticks. Its own RXLOSS
(150 ms) and failsafe stages follow without libfpv doing anything more. As
in ELRS, the receiver drops the sync after 1 s without a packet, reports
LQ 0, and needs 0.3 s of good link to find it again.

The modes (`fpv_rc_link_mode`) are ELRS's: sensitivity limits and the RFMD
numbers the OSD shows come from the ELRS 4.x table on expresslrs.org
(Signal Health). F1000 is -104 dBm (RFMD 33). 2.4 GHz 500/250/150/50 Hz are
-105/-108/-112/-115 (29/27/24/21). 900 MHz 200/100/50/25 Hz are
-112/-117/-120/-123 (5/2/1/0). The SNR a LoRa packet needs follows the
spreading factor (SF5 -2.5 dB, each step 2.5 dB lower), so the reported SNR
is the RSSI over `sensitivity − that`, saturating at +13 dB like the
radios' estimate.

**Dynamic power** follows expresslrs.org (Dynamic Transmit Power). The TX
starts at the lowest level and acts on each telemetry packet the receiver
sends back. libfpv sends 4 a second, at the TX's power up to 100 mW, over
the same, reciprocal path, which can also be lost:

- it lowers one level when SNR ≥ the mode's down threshold (FLRC: RSSI ≥
  -83 dBm) at LQ ≥ 95;
- it raises one level when SNR ≤ the up threshold (FLRC: ≤ -89 dBm), or
  when LQ < 85;
- it jumps to the maximum when LQ < 50, on a sudden drop (15 points under
  its average), or when telemetry is lost while armed;
- after a missed telemetry packet, the next one changes nothing, and each
  miss after the second raises a level;
- while disarmed it holds its level without telemetry (a battery swap).

The thresholds for 500/250/150 Hz and 900 MHz 200 Hz are the documented
ones. The slower rates keep the same spacing above their lower SNR floor.
Fixed power (dynamic off) sits at the maximum. Levels are ELRS's: 10, 25,
50, 100, 250, 500, 1000 mW.

**Betaflight** gets the statistics like an ELRS receiver's CRSF link
statistics frame: `bfb_rc_link_stats` switches `rssiSource` and
`linkQualitySource` to CRSF while the link is simulated (restored when it
is off, so the default LQ stays the one Betaflight counts itself), then
sets LQ, the RF mode, RSSI %, RSSI dBm, RSNR and the uplink TX power. The
OSD's `RFMD:LQ`, RSSI dBm, RSNR and TX power elements, their warnings and
the post-flight minimums all work. The SITL target has no CRSF receiver, so
Betaflight compiled `USE_RX_RSSI_DBM` out. Patch `0013` keeps it in
simulator builds. It is one line in `common_post.h` and does not change
any parameter group's layout (the alarm field is unconditional).

With the link off, nothing reports these statistics, and the elements
would show Betaflight's defaults (-130 dBm, 0 mW). Patch `0014` adds
`osdHostLinkElementsHidden` (simulator builds only). While it is set,
`osdAddActiveElement` leaves out RSSI %, link quality, RSSI dBm, RSNR and
TX power. A change rebuilds the active list on the next canvas update, the
way a change of GPS state does. `bfb_rc_link_stats` clears the flag and
`bfb_rc_link_release` sets it. It starts set, like the link (off). The
elements' positions and visibility in the EEPROM are left alone, so the
player's layout is kept for when the link is on. The flag is not a
parameter group, so it survives Betaflight's reboots.

Not modelled: the pilot's own body between the radio and the drone (libfpv
doesn't know which way he faces; up to ~10 dB at 2.4 GHz behind him),
antenna diversity, interference from other links, and ELRS's
switch/telemetry ratio settings.

Sources: expresslrs.org, "Signal Health" (packet rates, sensitivity
limits, RFMD, LQ/RSSI/SNR) and "Dynamic Transmit Power"; ITU-R P.833
(vegetation); Weissberger's model; wall losses from vendor and Wi-Fi
design tables (MaxStream 900 MHz note, NHS wireless building guidance,
Network World 2.4 GHz); Mellios et al. (Bristol), 2.4 GHz vs 868 MHz
off-body channels; SMU UAV ground-link measurements at 900 MHz / 2.5 GHz
(body effect on K).

### D40. The collision view (F8 > Camera & sound)

Integrators had no way to see what libfpv collides with: a wrong scale,
an axis flipped in the export, a missing mesh or a material without
`FPV_MATERIAL_KILLS_PROPS` only showed up as a drone flying through a wall
or living through lava. F8 > Camera & sound > "Show collision" draws it,
instead of the game's picture (solid) or over it (x-ray): static triangles
and surface objects at their pose for the frame, coloured by material
(red breaks props, green is clear to video, a violet outline is no
collision but blocks video, blue is a moving object), the raycast
callback's planes, the drone's collision spheres and prop discs, its
contacts at the drawn pose, and the impacts of the last 2 s (magenta and
"VTX" above `kVtxHitSpeed`, the speed that damages the video transmitter).
VTX damage has no material: it comes from hard hits and from the game's
`fpv_drone_damage_props`, so the impact colour is all the view can show.

libfpv draws it itself with Dear ImGui triangles in the overlay's
background draw list, so every game that draws the overlay has it with no
code of its own. The alternative, a 3D debug API the game draws with its
depth buffer, would be exact but needs a renderer in every integration.
The cost of the 2D route:

- No depth buffer. Triangles are sorted back to front (painter's
  algorithm). Sorted whole, a big triangle went wrong: the standalone's
  ground is two triangles hundreds of metres across, and facing south one
  of them had its centroid beside or behind the camera, so it was drawn
  last, over everything standing on it. Each triangle is now cut at its
  midpoints, up to 10 levels, until no piece is longer than its distance
  from the camera (not below 0.25 m), and every piece sorts by the centroid
  of its part in front of the near plane. Only the triangle's own sides
  are outlined. Hairline cracks between thin, far pieces showed the
  background, so opaque fills grow by 0.6 px, and in solid mode each cut
  triangle is first drawn whole under its pieces. Fog is per corner, by
  distance: per piece, it drew bands across the ground. Intersecting
  triangles can still come out in the wrong order.
- The CPU projects everything. Only triangles within the draw distance
  (150 m by default) and inside the frustum are drawn, at most the
  nearest 40 000 (both settings in the tab), and the gathering stops at
  400 000 near the camera.
- The camera is libfpv's: the one `fpv_set_view` selects (FPV, chase,
  LOS; in the pilot view the camera given to `fpv_audio_set_listener`,
  else the pilot's head). In the FPV view the picture follows the video
  feed's rectangle (camera aspect with bars) and the fisheye: each vertex
  goes through the inverse of the lens post-process (Newton on the radial
  polynomial), and triangles larger on screen are cut into up to 256
  pieces so their edges bend like the game's. A game that draws its own
  camera with another FOV will not line up exactly in x-ray.

The overlay now runs an ImGui frame while closed when the view is on (no
windows, no cursor), and `fpv_overlay_render` returns draw data then.
`ImDrawIdx` is 32-bit (a compile definition on the `fpv` target): one list
far exceeds 65 536 vertices, and the public `fpv_draw_cmd` indices were
32-bit already, so hosts see no change.

The drone's sphere layout moved from `Flight::buildShape` to
`world/drone_shape.cpp` (`buildDroneShape`), shared by the flight model and
the view, so the view draws exactly what collides. The flight model adds
the cargo and, in kamikaze builds, the warhead itself; the view leaves
those out. The setting is not saved: a player who once turned it on would
otherwise start every session without the game's picture.
`LIBFPV_COLLISION_VIEW=solid` (or `xray`) starts with it on. The legend
shows for 10 s each time a view is chosen in "Show collision", then hides,
so it doesn't stay over the view.

Verified with `fpv_standalone --screenshot` (FPV with fisheye and bars,
chase, both modes). No automated test: the overlay reads the variable once,
when it is created, and `fpv_tests` creates it in an earlier test.

## Platforms

### D19. Windows: MinGW-w64 cross build, fully static runtimes

`cmake/toolchains/mingw-w64-x86_64.cmake` uses the `-posix` GCC
(winpthreads, needed for `std::thread`) and links libgcc, libstdc++ and
winpthreads statically into every binary. Without that, executables need
`libgcc_s_seh-1.dll`, `libstdc++-6.dll` and `libwinpthread-1.dll` and fail
to start silently: found when `fpv_tests.exe` exited with code 53. Now
`libfpv.dll` imports only Windows system DLLs. `libfpv.def` is emitted so
MSVC hosts can build an import library with `lib /def:libfpv.def
/machine:x64`. They can also `LoadLibrary` it, since the API is pure C.
The full test suite passes on Windows 11 (run from WSL interop).

SimITL's `string_light.c` re-implements libc's ctype and `strcasecmp`
functions. With whole-archive linking (D20) these collide with msvcrt, so
it is excluded on every platform and the platform libc is used. MinGW's
missing `strcasestr` comes from `src/server/core/bf_compat.c`. Betaflight C files
are compiled with `_GNU_SOURCE`, so `strcasestr` has a real prototype
rather than an implicit `int` that would truncate the pointer. Betaflight's
structs use GCC bitfield layout (`-mno-ms-bitfields`), applied to C and
C++ alike inside the core.

### D19b. 32-bit Windows (i686) for 32-bit host games

`cmake/toolchains/mingw-w64-i686.cmake` mirrors the x86_64 one for 32-bit
hosts such as GTA IV (a PE32 executable, so it can only load a 32-bit
DLL). It adds `-msse2 -mfpmath=sse`: with GCC's default x87 maths,
`wall_crash` failed (a 63 m/s drone tunnelled through the wall). With SSE2
the full suite passes (20 tests, 321 checks) on Windows 11 through WSL
interop. MSVC hosts use `lib /def:libfpv.def /machine:x86`.

### D20. Whole-archive linking of the core

Betaflight's parameter groups register themselves in a linker section that
no code references by name. `simitl_core` is therefore linked with
`$<LINK_LIBRARY:WHOLE_ARCHIVE,...>`, so no PG can be dropped by the linker.

### D28. Web builds (Emscripten)

A game with source code available can be compiled to WebAssembly with its
mod, so that players fly from a link (docs/WEB.md). libfpv builds for it
with `emcmake cmake`.

* **Static, one object.** The game's module is linked statically (dynamic
  linking in WebAssembly is slower and needs the game built as a main
  module). `libfpv.a` holds a single relocatable object (`emcc -r` with the
  whole core), so a plain link brings every parameter group (D20) and the
  game's build needs no `--whole-archive`. `tools/wasm-localize-hidden.py`
  then makes every hidden symbol of that object local, which
  llvm-objcopy cannot do for wasm. The object is then as sealed as the
  native shared library: Betaflight's `init`, `delay` or `debug`, Dear
  ImGui and fmt cannot clash with the game's symbols, and only `fpv_*` is
  visible. Weak symbols (templates, inline functions) stay global so that
  COMDAT deduplication works. The partial link treats warnings as errors,
  because a signature mismatch there becomes a trap at run time (SimITL
  patch 0015).
* **No threads.** WebAssembly threads need `SharedArrayBuffer`, which
  browsers only enable for pages served with COOP/COEP headers. GitHub
  Pages and itch.io cannot send them. The Gamepad API is only readable on
  the main thread anyway. `FPV_TIMING_THREAD` therefore keeps its meaning
  (real time, fixed 1 kHz steps, the 50 ms catch-up cap, auto-pause,
  interpolation one step in the past) but runs cooperatively:
  `Runner::advance` is the body of the thread's loop, and the host's
  per-frame calls run it (`fpv_drone_tick` before its heartbeat, so a gap
  in ticking still auto-pauses; `fpv_overlay_render`, so the radio wizard
  sees live sticks without a drone). `Runner::call` already ran jobs
  inline without a thread. Hosts need no change.
* **Gamepad API instead of SDL.** `src/client/input/web_backend.cpp` gives the same
  `RawDevice`s: devices with the browser's "standard" mapping have the
  gamepad layout (buttons renumbered to SDL's), the rest are raw axes,
  and the radio name list is shared with the SDL backend. A game that uses
  SDL on the web brings its own, so libfpv embedding one would only add
  size.
* **No Configurator.** Its WebSocket server cannot exist in a page.
  libwebsockets is left out, `src/server/core/serial_ws_web.c` opens Betaflight's
  UARTs with no transport, and `FPV_INIT_NO_CONFIGURATOR` is forced. The
  overlay's tab says why and points to the CLI and diff import. Making it
  work would mean hosting the Configurator in the same page with a
  transport bridged into the module.
* **Persistence.** `src/web/libfpv_web.js` mounts `FPV_WEB_DATA_DIR`
  (`/libfpv`, or `Module.libfpvDataDir`) on IDBFS with `autoPersist`
  before `main()`, as a run dependency. Every close, rename and unlink
  there is synced to IndexedDB, which covers libfpv's atomic writes and
  Betaflight's EEPROM, which is closed after each save (`configLock`).
* **Exceptions** are WebAssembly instructions (`-fwasm-exceptions`). They
  are the fast kind, and supported by every browser with WebGL 2.
  Emscripten's default would abort on a JSON parse error.
* **Clipboard.** SDL's browser backend cancels the default action of every
  key pressed with Ctrl, so no `paste` event ever fires. On Ctrl+V the
  overlay reads `navigator.clipboard` (asynchronous; the browser asks once)
  and types the text in. ImGui's own paste finds an empty clipboard, so
  nothing arrives twice.

`fpv_tests` runs on Node (`-sNODERAWFS`). All tests pass; `pinned` checks
less on the web, where nothing can unload the module.

## Modes

### D21. Turtle mode (flip over after crash)

Betaflight decides crash flip when arming with the FLIP OVER AFTER CRASH
switch on, then reverses the motors with a DShot command. The simulated
ESCs do not act on DShot commands, and PWM has none (D5, D44), so instead
`bfb_crash_flip_active()` (armed and `isCrashFlipModeActive()`) feeds
`StateInput::motorsReversed` every step.
Betaflight's own crash flip mixer picks the motors from the sticks.
Fresh EEPROMs get turtle on AUX4 (`aux 3 35 3 1700 2100 0 0`). The F8
radio wizard's turtle switch step binds AUX4, the keyboard fallback toggles
it with T and the gamepad fallback with Y / Triangle.
`FPV_DRONE_TURTLE` is set while armed in turtle mode.

AUX switches on an axis are positions, not ranges: a switch is on when the
axis is closer to its "on" value than half the way to any other known
position of that axis (rest, or another switch's "on"). So one 3-position
switch can carry beeper in the middle and turtle at the top, and a mode
mapped to the middle alone is off at the top. The wizard offers an axis
already used by a switch again only in a position no switch uses yet.
Saved mappings keep their format (on = `max`, or `min` when inverted).

libfpv sets `crashflip_rate = 30`, the default of Betaflight's Race Pro
builds (stock is 0). It cuts the turtle power as the quad spins faster and
once it has turned about 90°, so holding the stick works, and turning the
switch off re-arms in normal mode at once (quick restart). At stock 0 a 5"
launches off its pivot above about 60 % stick (two reversed props lift
1.4-2.9 times its weight) and rolls on. The Air65 II's tune keeps its real
firmware's 0: its reversed props lift a third of its weight, it pivots and
lands upright. The `turtle` test holds the stick until upright with the
cut-off, and with full power lets go after about 35°.

A real Air65 II's turtle flips (D38) show what the simulator now does too.
The ESCs take about 0.1 s to start a stopped motor before it turns at all
(SimITL `0022`). The flips turn it at about 14500 deg/s² at 60000 rpm
backwards: a reversed prop gives about 30 % of its forward thrust (SimITL
had 50 %), and a ducted one gets no lip thrust from its duct (`0023`).

EEPROMs made before this get, once, the turtle switch on AUX4 (if there is
no crash flip mode yet and AUX4 is free) and `crashflip_rate = 30` (if it
is 0). A quad whose tune sets `crashflip_rate` (the Air65 II) gets the
tune's value on an older EEPROM (upgrade `crashflip-tune-5`). A
`<eeprom>.upgrade` file records the last upgrade applied, so later edits
by the player stay.

### D23. Air mode and OSD disable switches (AUX5, AUX6)

Two more Betaflight modes that pilots keep on switches: AIR MODE (box 28)
on AUX5 and OSD DISABLE (box 19) on AUX6, both on the high position, in
mode slots 4 and 5 of fresh EEPROMs. The F8 radio wizard binds them after
turtle, both optional, with the same position logic as beeper and turtle
(D21): one 3-position switch can carry air mode in the middle and OSD
disable at the top. The keyboard and gamepad fallbacks leave both off.

libfpv now drives 10 channels (AETR + AUX1..6, `FPV_CHANNEL_COUNT`).
SimITL passed only 8 to Betaflight; `0009-sixteen-rc-channels.patch`
widens `StateInput::rcData` and `BF::setRcData` to Betaflight's
`SIMULATOR_MAX_RC_CHANNELS` (16). libfpv sends -1 on AUX7 and up.
`fpv_frame.channels` grew to 10 at the end of the struct, so hosts built
against the old header still get the first 8 (`struct_size`), and
`fpv_input_set_channels` with 8 channels leaves AUX5/AUX6 off.

Air mode: Betaflight's AIRMODE feature is on by default, and then air
mode is always on and the switch changes nothing, exactly like on a real
quad. The switch matters when the player turns the feature off
(`feature -AIRMODE`, or a `diff all` from a quad flown that way).

OSD disable: Betaflight clears the whole OSD while the mode is on
(including warnings and the post-flight statistics), so `frame.osd` is
blank. `FPV_DRONE_OSD_DISABLED` (from `IS_RC_MODE_ACTIVE(BOXOSD)`) tells
the host, which must then hide every bit of its own UI that it still
draws over the FPV view (INTEGRATION.md, "OSD disable switch"); the
player asked for a clean picture.

Older EEPROMs get both switches once, each only if that mode is not
configured yet and its channel is free, through the same `.upgrade` file.
Upgrades are now an ordered list and the file holds the last one applied
(`turtle-2`, then `air-osd-3`), so an EEPROM already upgraded for turtle
gets only the new switches, and turtle is never re-added after the player
removed it. A `diff all` import may now move modes to AUX2..AUX6.

### D26. Keyboard hover throttle

Auto, on or off (F8 > Controls, `keyboardHover` in settings.json, "auto"
by default; the old `true` reads as "on"). Auto turns it on while no radio
is detected, mapped or not, like the forced ANGLE mode of the fallbacks, so
a keyboard-only player gets it at once and it never surprises a radio pilot.
No key (or W and S together) hovers, S cuts the throttle, and W multiplies
it by e^(expo * dt) from at least the hover up to 100%
(`keyboardThrottleExpo`, 1/s, default 3).
The hover is a climb-rate hold: throttle = hover / sqrt(cos tilt) *
(1 - 3 * climb / 2g), since thrust grows about with the throttle squared.
`hover` is learned while holding (an integrator on the climb rate, clamped
to 2 m/s, frozen while the throttle is pinned at 0 or 100%), so it adapts to
each quad, its battery sag and payload without a thrust model; it is kept in
memory per preset (first guess 0.25; the presets hover at 0.16..0.31). On
the ground, upside down or in turtle mode no key means idle, so arming never
lifts off by itself. A/D yaw ramps the same way, always (not only with the
hover throttle): 15% right after the press, times e^(expo * dt) up to full
(`keyboardYawExpo`, default 4, so about half a second). The runner feeds the previous step's climb rate, tilt
and ground contact to the input manager (`HoverFeedback`).

## Sensors

### D22. GPS and barometer

Every simulated quad has a GPS and a barometer, so Betaflight's GPS and
altitude features work: speed, home arrow and distance, distance flown,
altitude, vario, efficiency, their post-flight statistics, the GPS lap
timer and GPS Rescue.

* `Flight::step` feeds Betaflight's virtual GPS provider (`setVirtualGPS`,
  the target's default) and its virtual barometer every step. Betaflight
  samples the GPS at 10 Hz like a real module. Satellites are always 12.
* The local frame is laid on a flat patch of earth at 0° N 0° E, with
  internal +Z (host forward) as north and +X (host right) as east. At the
  equator a degree of latitude and of longitude have the same length
  (`EARTH_ANGLE_TO_CM`), so Betaflight's distance and bearing maths gives
  exact metres. Latitude, longitude and plus code are therefore
  meaningless and not supported.
* Home is Betaflight's own (where the drone arms) until the host calls
  `fpv_gps_set_home`. The host's home is re-applied after every step,
  because arming moves it. When the arming point replaced it, the
  distance and direction to home are recomputed at once, so the OSD never
  shows the arming point for a GPS cycle. The value is kept behind a mutex
  rather than a simulation-thread job, so hosts can set a moving home
  every frame without waiting.
* GPS is a libfpv essential, like the motor protocol: `ensureEssentials` turns
  `feature GPS` back on and reboots Betaflight in-process when it is not
  running (features only apply at boot). The first-run script and every
  import include `feature GPS`.
* The barometer is fed too. Without it the virtual baro reads sea level,
  and Betaflight's altitude mix, which prefers the baro when the two
  disagree by more than 1 m, kept the altitude near 0.
* `bfb_cli_run` calls `osdAnalyzeActiveElements` after each CLI session.
  A real CLI `save` reboots, which rebuilds the list of elements to draw;
  libfpv's `save noreboot` did not, so elements moved or enabled through
  `fpv_cli_exec` stayed hidden until the next reboot.

Measured limits of the simulator's scheduler: SimITL calls Betaflight's
scheduler once per 125 µs gyro step, and each call runs at most one
non-realtime task. RX takes about half of those slots, so low-priority
tasks run below their rate (ALTITUDE 33 Hz instead of 100, GPS 50 instead
of 100), and each OSD element is redrawn only about every 0.4 s. Altitude
on the OSD trails the drone by up to about 0.9 s. Calling the scheduler
four times per step brought every task to its rate, but it changes
Betaflight's timing everywhere (RX then ran at 8 kHz) and was not
evaluated beyond that, so it was not kept. Patch 0005 fixes the part that
was wrong rather than late.

## Kamikaze builds

### D24. Warhead, bomb arm and explode switches (`LIBFPV_KAMIKAZE`)

Kamikaze (strike) FPV is a separate build, `-DLIBFPV_KAMIKAZE=ON`, off by
default. Without it nothing changes: no new symbols, no new channels, same
header (the kamikaze declarations sit behind `#if defined(LIBFPV_KAMIKAZE)`),
same behaviour. The option is a PUBLIC compile definition of `libfpv::fpv`,
so hosts linking through CMake see the API automatically. A host built
with it against a library built without it fails to link or to resolve
`fpv_kamikaze_*`, which is the feature check.

**Installed or not.** The bomb is a per-quad choice, off by default:
`fpv_kamikaze_set_installed` or the overlay's "Bomb installed" (F8 >
Fly). Uninstalled, the drone is the plain quad of other builds: no
payload weight or collision sphere, no warhead OSD, and `Warhead` ignores
AUX7 and AUX8 (it does not hold AUX1, reports both switches off, and
neither the switches nor `fpv_kamikaze_detonate` can fire it). The flag
is saved per quad: `Settings::kamikazeInstalled` lists the preset file
names that carry it (`"kamikazeInstalled"` in `settings.json`), and
selecting a preset pushes its choice to the `Warhead` of the one `Flight`,
which outlives drones, so a new, respawned or repaired drone keeps it. It
used to be a session-only flag, not saved, on the idea that a game decides
per mission; but a player who builds a strike quad expects it to stay one,
like any other part of the quad, and a game deciding per mission still
can by calling `fpv_kamikaze_set_installed` at spawn. Installing or removing it safes the warhead (a switch left
on must be cycled), except on a wreck, which stays one until repaired.

**Switches.** AUX7 is the bomb arm switch and AUX8 the explode switch.
`FPV_CHANNEL_COUNT` becomes 12 and `fpv_frame.channels` grows at its end,
the same `struct_size`-safe way as in D23. Betaflight receives both
channels like any other (the Configurator's Receiver tab shows them) but
has no mode on them: on real strike drones the firing board, not the
flight controller, reads them. The radio wizard adds both steps after the
OSD switch, both skippable; the keyboard fallback uses K (bomb toggle) and
X (explode, held), the gamepad West (toggle) and the right shoulder.

**Rules,** in `src/server/sim/kamikaze.cpp` (`Warhead`, simulation thread):

* The motors do not arm while the bomb arm switch is on, and a destroyed
  drone never arms. libfpv holds AUX1 low towards Betaflight, and keeps
  holding it until the pilot cycles the arm switch, like Betaflight's own
  ARMSWITCH rule; otherwise turning the bomb off with the arm switch on
  would arm the motors at once. Betaflight has no arming-disable flag for
  this and none was patched in, so `arming_disable_flags` stays 0 and
  `arming_disable_text` reads "BOMB".
* Switch on with the motors armed: ARMED at once, or ARMING for
  `arming_delay` first when one is set (default 0: strike pilots fly with
  a live warhead the moment they flip it; a delay is a game's or a
  player's choice). Switch off at any time: SAFE. Motors disarmed while ARMING: SAFE
  and the switch must be cycled. Motors disarmed while ARMED: it stays
  live (a falling live drone is still a bomb).
* A switch that is on when the drone spawns or is repaired must be turned
  off first, so a respawn never comes with a live or arming warhead.
* Only ARMED explodes, on a contact, on the explode switch's off-to-on
  edge (an explode switch already on when it goes live does nothing until
  cycled), or on `fpv_kamikaze_detonate`.
* A contact or the explode switch lights the fuse: IGNITING for
  `ignition_delay` (default 0.075 s, 0..1, the player's in ms), then the
  blast. A real initiator does not go off in the same instant either, and
  the fraction of a second is the pilot's last look at the target: the OSD is
  cleared to `!...BOOM...!`, drawn by libfpv over the grid like
  Betaflight's `ARMED` screen, whatever the OSD disable switch. The motors
  are cut for it (the props get full damage towards SimITL so there is no
  thrust, the motor state is zeroed every step so there is no sound), so
  the drone can't escape the target in the last milliseconds and only
  falls or bounces. The video link is untouched until the blast, so the
  feed's impact jolt (D34) carries the message. Switches can't stop a lit
  fuse; `fpv_kamikaze_detonate` still fires at once (a game's bullet into
  a burning fuse). The queued event is made at the blast: the sim time
  then, the impact's normal, material and object, and the impact point
  moved by however far the drone went meanwhile, so it matches where the
  wreck freezes. 0 s keeps the old instant blast.
* The fuse is a contact fuse: every collision contact approaching faster
  than `min_impact_speed` (default 0.25 m/s, 0.1..30, tunable in the
  overlay) fires it, and so does a spinning prop striking something. That
  is far below the 1.5 m/s of `fpv_frame.impacts`, so a light tap on the
  ground counts. Contacts slower than 0.1 m/s never reach it: a drone
  resting on the ground with a live warhead does not explode.

**After the explosion** the drone is destroyed but not deleted, so every
kind of game can build on it: the wreck stays frozen where it exploded,
the props are broken (CRASHED), the motors are stopped, AUX1 is held low,
the video reports no signal (noise 1, signal 0), and the OSD is blank
until the repair: with the video transmitter gone, the goggles show only
static. libfpv used to write "SIGNAL LOST" in the middle of it; that was
dropped so the screen is just the static. With the video feed's noise on
(D34) the static is the feed's: the detonation sets the received power to
nothing, so the per-line table goes to snow with no sync. The blanking bar
and torn line starts mix into that snow by each line's signal, so a
missing transmitter shows uniform receiver noise edge to edge. Games paint
their own static only while the noise is off (the default). Games can still draw their own
OSD with `FPV_KAMIKAZE_NO_OSD`. `fpv_drone_repair` is the new drone with a
new, safe warhead. libfpv never respawns on its own: whether the game
cuts to a cutscene, respawns at once, or leaves the pilot on static until
they take the next drone is the game's choice (INTEGRATION.md, "Kamikaze
builds"). Detonations are queued (`fpv_kamikaze_poll_detonation`, up to
16) rather than put in `fpv_frame`, so frames keep their layout and a
host polling less often loses nothing.

**OSD.** The warhead state is a Betaflight OSD element: `BOMB_SAFE`,
`BOMB SW ON` (blinking, the motors are blocked), `ARMING 3`, `!ACTIVE!`
(live) and `NOT ARMED` (explode switch on a safe bomb). The exact wording
of real firing boards is not public. `BOMB_SAFE` and `!ACTIVE!` were
`SAFE` and `STEADY` at first, then `!ACTIVE!` was `BOMB_ARMED`.
The aiming dot of a live warhead used to replace the crosshairs element's
glyphs, so it existed only if the player had the crosshairs on and
vanished with the OSD disable switch. Now Betaflight (patch 0016) hides the
crosshairs and the artificial horizon while it is live, and libfpv draws
the dot (the font's `0x09`) on the crosshairs' default centre, row 7
column 14, over the composed grid: always there, OSD disabled or not. With
`FPV_KAMIKAZE_NO_OSD` neither is done (the game draws its own).
At first libfpv wrote the text into `frame.osd` over Betaflight's grid,
which put it above everything, the post-flight stats included, and fixed
on row 3. It now takes Betaflight's "Custom message 1" (`OSD_CUSTOM_MSG0`,
meant for text sent over MSPv2): Betaflight patch 0006 lets the host
supply that element's text (`bfb_osd_set_host_message`, set every step by
`Flight`), and SimITL patch 0010 adds `osd_custom_msg0_pos`..`3_pos` to
the CLI, which upstream lacks (the Configurator moves it over MSP), so the
overlay's OSD tab ("Bomb status") and scripts can place it too. Betaflight
then layers it like any element, as the pilot name became one after
patch 0003. The text is centred in a 10-character field so it stays put as
it changes, the field defaults to columns 10..19 of row 3, and EEPROM
upgrade `bomb-osd-4` shows it there unless the player already placed it.
Blinking is 1 Hz because Betaflight redraws the simulated OSD only every
~0.3 s. After the blast libfpv blanks the grid itself. Both can be
turned off (`FPV_KAMIKAZE_NO_OSD`) for games that draw their own, and
since Betaflight's `osd_revision` cannot see the blanked grid, kamikaze
builds count revisions of the composed grid instead.

**Payload.** The warhead is real weight. Its mass is added to the quad's
in SimITL, its inertia is added as a point mass `drop` below the centre
(parallel axis) plus its own solid sphere, and it adds a collision sphere.
The default is the preset's `"payload": {"mass", "drop"}`, or else 60 % of
the quad's own mass strapped under the frame, flush with the landing
points; a sphere hanging below them made the drone balance on it and tip
over (ANGLE). The centre of mass does not move, because SimITL has no
offset for it. Changing the payload patches SimITL's live `stateInit`
instead of re-initialising the physics, so it can change in flight.
`fpv_kamikaze_config` and the overlay (F8 > Fly at first, F8 > Kamikaze
since D29) edit the same values (payload, arming delay, fuse threshold,
ending, OSD), saved in `settings.json`. The
ending duration (3 s by default; 1 s was too short to see the blast) is
only a number libfpv stores for the game: the game times its own
cutscene or static, so it can still pick any ending, and the player tunes
one slider for every game. D29 turned it into three options.

Tests: `kamikaze_*` in `fpv_tests` (only in kamikaze builds; the other
tests run there without the bomb installed, since they check a bare quad
and Betaflight's own OSD), the wizard test with both switches (their own
setup since D29),
and `fpv_standalone --kamikaze-demo`, which flies a scripted strike in the
real app and exits 1 if any rule fails.

### D29. The ending: view, duration, then (and the Kamikaze tab)

Games first offered two endings of their own, "arcade" (a cutscene, then
respawn) and "signal lost" (static until the pilot acts), with libfpv's
`ending_duration` as the only shared setting, and gtasa-fpv put a button
for the choice in F8 > Fly. Every game then had its own words and
defaults for the same thing. The ending is now three player options in
`fpv_kamikaze_config`: the **view** (`SIGNAL_LOST` or `CUTSCENE`), its
**duration** (0..30 s, 0 = no view before Respawn or Recall) and **then**
(`RESPAWN`, `RECALL` or `NOTHING`, which holds the view until the player
acts, also with 0 s). The player's own respawn, recall and "Move <pilot>
here" work at once during the view and replace "then": waiting out a
timer to take the next drone felt broken.

libfpv still only stores the choice; the game times the ending with its
own clock, so it pauses with the game and libfpv needs no ending state.
Not every game can do every ending (the standalone example has no pilot
to recall to), so `fpv_kamikaze_set_ending_support` declares the masks
and the game's defaults. A saved choice the game can't do reads back as
its default but is kept in `settings.json`, so it never overwrites the
player's choice; a supported saved choice always wins. A player who never
chose has nothing saved (`kamikazeEndingView`/`Then` absent, by name when
present), so each game's own default applies. Signal lost and Nothing need
nothing from the game and are always allowed. The new fields come after
`ending_duration`, so an older host (a smaller `struct_size`) keeps the
player's choice, as with the duration.

All kamikaze settings moved from F8 > Fly to a "Kamikaze" tab (compiled
only in kamikaze builds). The bomb switches left the main radio setup:
it asked for them only with the bomb installed, so a player who set up
the radio first and installed the bomb later had to redo the whole radio.
The tab has its own setup for AUX7 and AUX8 on a radio that is set up
already (`RadioWizard::startBombSwitches`). It avoids the positions the
other switches use and merges only those two channels into
`radio_mappings.json`. A new full setup keeps the bomb switches unless
their axis position or button is now taken (`mergeBombSwitches`).

Tests: `kamikaze_ending` in `fpv_tests` (defaults, old `struct_size`,
range checks, `settings.json` across a restart, support masks and
fallback), the bomb switches' setup and merge in `fpv_wizard_tests`, and
`fpv_standalone --kamikaze-demo`, which after the strike checks no view,
a held view, an early respawn cancelling the timer and 0 s with Nothing.
