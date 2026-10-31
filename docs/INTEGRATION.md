# Integrating libfpv into a game

The full hook sequence is shown in `examples/standalone/main.cpp` (the
comment at the top lists every call in order). This file holds the details
that need more than a comment. For a game compiled to run in a web browser,
read [WEB.md](WEB.md) as well.

## Lens distortion

FPV cameras use wide fisheye lenses. libfpv gives the parameters to use and
the game draws the effect, as a post-process on the FPV view only (not the
chase or LOS cameras):

1. Render the scene from `frame.camera` into an offscreen texture, using
   `frame.camera.fov_y_degrees` as the vertical FOV.
2. Draw a full-screen pass that samples that texture with this shader
   (GLSL; port it to HLSL or anything else as needed):

```glsl
uniform sampler2D scene;
uniform vec2 k;       // frame.camera.distortion_k1, distortion_k2
uniform float zoom;   // frame.camera.distortion_zoom
uniform float aspect; // output width / height
in vec2 uv;           // 0..1 across the output
out vec4 o;
void main() {
  vec2 scale = vec2(aspect, 1.0) / (0.5 * sqrt(aspect * aspect + 1.0)); // r = 1 at the corners
  vec2 c = (uv - 0.5) * scale;
  float r2 = dot(c, c);
  vec2 suv = c * (1.0 + k.x * r2 + k.y * r2 * r2) / zoom / scale + 0.5;
  o = (any(lessThan(suv, vec2(0.0))) || any(greaterThan(suv, vec2(1.0))))
      ? vec4(0.0, 0.0, 0.0, 1.0) : texture(scene, suv);
}
```

3. Draw the OSD (`fpv_osd_draw_data`) and the overlay on top, undistorted.

Without shaders (for example fixed-function D3D9), draw the copied scene
through a grid of about 64x36 cells instead. Give each vertex at output
position `uv` the texture coordinate `suv` from the formula above. The warp
is smooth, so the result matches the shader to within a pixel.

The player turns the fisheye on or off and sets its strength in F8 > Camera >
"Fisheye lens". It is on by default. When it is on, libfpv picks `k1`, `k2`
and a `zoom` that keep the image's corners filled (no black borders), and
widens `fov_y_degrees` so the top and bottom edges of the distorted image
still show the FOV the player chose. This widening uses the camera's
`aspect_hint`, so the vertical FOV is only exact when the output has that
aspect. When the slider is at 0, the preset's own `camera.k1` / `k2` / `zoom`
are used (normally none). A game that skips the post-process still works, but
shows a slightly wider, undistorted view.

## GPS and the home point

Every simulated quad has a GPS and a barometer, so Betaflight's GPS OSD
elements work with no work from the game: speed, home arrow, home
distance, distance flown, altitude, vario, efficiency (mAh/km), their
post-flight statistics and the GPS lap timer. GPS Rescue works too, if the
player sets it up in the Configurator. Players turn the elements on in
the Configurator's OSD tab, like on a real quad.

Not supported, because a game world has no real place on earth: latitude,
longitude and plus code (the world sits at 0° N 0° E). The satellite
count always shows 12.

### Which way is north

North is the host's `forward` axis and east is its `right` axis, as given
in `fpv_global_config.coords`. The heading number and compass bar use the
same directions as the GPS, so the home arrow is right whatever you pick.
Only the absolute heading depends on it. If the game's map has north along
another axis, choose `coords` so that `forward` points north (GTA IV:
`forward` = +Y, north). libfpv has no separate "north" setting.

### Telling libfpv where home is

By default home is Betaflight's: the point where the drone last armed, as
on a real quad. That is already right for a pilot who arms the quad at
their feet. To set home yourself, pass a position in host coordinates:

```c
float home[3] = { x, y, z };       /* host coords and units, like everything else */
fpv_gps_set_home(home);            /* home arrow, home distance, GPS Rescue */
fpv_gps_set_home(NULL);            /* back to Betaflight: the next arming point */
```

- The home stays until you change it: across disarm and re-arm,
  `fpv_drone_delete` / `fpv_drone_create` and preset changes. Arming no
  longer moves it.
- It is cheap and doesn't wait for the simulation thread, so a moving home
  (a pilot walking, a car, a boat) can be set every frame from the game
  thread. Any thread is fine, after `fpv_global_init`.
- Pick what "home" means in your game:
  - **The pilot.** If the game has a character who flies the drone, pass
    the character's position (the same point you would give as
    `fpv_drone_desc.pilot_position`). The arrow then always leads back to
    them, even if they walk away from the take-off spot.
  - **A landing pad or base.** Pass it once, after `fpv_drone_create`
    (examples/standalone sets its launch pad).
  - **Nothing.** Leave it alone. Respawning (`fpv_drone_reset`) does not
    move home, but re-arming does.
- Altitude and the vario are relative to where the drone armed, not to
  home, as in Betaflight. GPS Rescue's climb and descent use that same
  zero.
- Moving the drone with `fpv_drone_reset` while it flies can add the jump
  to "distance flown", as carrying an armed quad would.

On the OSD, values update a few times a second and altitude trails the
drone by up to about a second (DECISIONS D22). Don't build gameplay on
reading them back; use `fpv_frame` (position, speed, altitude) instead.

## OSD disable switch

Pilots put Betaflight's OSD DISABLE mode on a switch to get a clean view
for recording or for looking at the scenery. libfpv maps it to AUX6 (the
radio wizard's "OSD disable switch"). While it is on, Betaflight draws no
OSD, so `frame.osd` is blank, and `frame.flags` has
`FPV_DRONE_OSD_DISABLED`.

**While `FPV_DRONE_OSD_DISABLED` is set, the game must hide all of its own
UI over the FPV view too.** Normally the game's HUD is already hidden in
FPV ([Drawing the OSD and the overlay](#drawing-the-osd-and-the-overlay-d3d9-and-other-hooked-renderers)),
but most games keep a few things on screen while flying: a lap or race
timer, checkpoint markers, objective text, subtitles, notifications, a
crosshair, a speedometer, a watermark or the mod's own hint text. The
switch means "nothing on top of the video", so hide all of it, every frame
the flag is set, and show it again as soon as the flag clears. The flag
follows the switch whether or not the drone is armed.

Keep showing:

- the overlay (F8). The player opened it on purpose, and it does not draw
  when closed.
- the game's pause menu and loading screens, which are not drawn over the
  flight.
- video effects that are part of the picture, such as the fisheye, the
  analog noise and breakup (`frame.video`) and a frozen digital image. They
  are not UI.

If the game cannot hide some element (it is baked into the engine), list it
in the mod's README, so players know why it stays.

## Wind

The drone flies in moving air. Its drag, rotor drag, prop inflow and prop
wash all work on the airspeed. In wind the quad leans to hold its place,
drifts in angle mode and gets bumped by gusts, and Betaflight's PID loop
has to deal with it like on a real quad. The game says what its weather
is doing. libfpv works out what that means where the drone is: more wind
higher up, gusts, calm air in the lee of buildings with rougher air
behind them, updrafts in front of walls, and almost nothing indoors. All
of that comes from rays into the same world the drone collides with
(static surfaces, surface objects and the raycast callback), so a game
that has collision gets shelter too.

The player turns wind off, or scales the weather's part, in F8 > Fly >
Wind. It is on by default. With it off, sources and blasts do nothing
either, and the drone flies exactly as in still air.

There are three calls, all optional. A game that makes none of them has
still air.

### The weather: `fpv_wind_set`

```c
fpv_wind w;
fpv_wind_defaults(&w);           /* still air, automatic gusts, 10 m reference */
w.velocity[0] = east_mps * upm;  /* host coords, host units per second: */
w.velocity[1] = north_mps * upm; /* the way the air MOVES (a north wind points south) */
w.flags = indoors ? FPV_WIND_ENCLOSED : 0;
fpv_wind_set(&w);                /* every frame is fine; NULL = still air */
```

- **Give the mean wind, not the gusts.** `velocity` is the steady wind in
  the open, 10 m above flat ground (`reference_height` changes that), as
  a weather station would report it. libfpv makes the gusts, the height
  profile and the shelter. If the game's wind value already wobbles,
  low-pass it over a few seconds first. Otherwise its gusts pile on top
  of libfpv's, and a game's scripted gusts tend to be periodic and the
  same everywhere.
- **Horizontal only.** libfpv adds the vertical air motion (gusts,
  updrafts along walls). A vertical part you give is added as it is.
- **Map the game's numbers to m/s.** Games often have a 0..1
  "windiness". For reference: 2–3 m/s is a light breeze, 5–8 m/s a
  breezy cloudy day, 10–12 m/s a storm. A 5" quad can still fly in about
  15 m/s; a 65 mm whoop is blown away by 5.
- **Interiors.** If the game knows the drone is indoors (a separate
  interior world, a room, a cave), set `FPV_WIND_ENCLOSED`: the weather
  stops, and only sources and blasts move the air. libfpv also detects
  enclosed spaces by itself (a roof overhead and walls nearly all round).
  But a game whose interiors float somewhere above the map would
  otherwise get the wind of that height through every open door.
- `turbulence`: leave it at -1 (automatic: about 0.35 near the ground,
  down to 0.15 high up), or give the gusts' standard deviation over the
  mean, 0..1. 0 is a perfectly steady wind (useful in tests).
- `FPV_WIND_NO_SHELTER` turns the probes off: the same wind everywhere,
  for games without collision, or to measure the probes' cost.

### Things that blow: `fpv_wind_set_sources`

These are the rotors and jets the game simulates: helicopter downwash, a
VTOL's nozzles, a jet's exhaust, a big fan. Pass the ones near the drone
every frame they move or change (at most `FPV_MAX_WIND_SOURCES`, 32).
`count` 0 clears them.

```c
fpv_wind_source s = {0};
/* position: the rotor hub (host coords); direction: where the air is blown */
s.radius = rotor_radius;                                /* host units */
s.speed = sqrtf(thrust_n / (2 * 1.2f * 3.14159f * r_m * r_m)) * upm;
s.type = FPV_WIND_SOURCE_ROTOR;
fpv_wind_set_sources(&s, 1);
```

- **Rotor.** `speed` is the induced velocity at the disc. With the
  thrust T in newtons (for a hovering helicopter, its weight: mass ×
  9.81) and the rotor radius r in metres, `v = sqrt(T / (2 · 1.2 · π ·
  r²))`. A 2.5 t helicopter with a 5 m rotor gives about 11 m/s, so about
  23 m/s in its wake. libfpv finds where the wake lands and spreads it
  out along the ground there. The ray it casts for that starts 0.6 radii
  below the hub, which skips the aircraft's own body under the rotor.
- **Jet.** `speed` at the outlet, `radius` the outlet's.
- Scale `speed` with the rotor or engine speed: thrust ∝ rpm², so the
  induced velocity ∝ rpm. A parked helicopter with its rotor stopped is
  no source.
- Sources blow whatever the weather, the shelter or `FPV_WIND_ENCLOSED`.
  Each costs one ray per frame.

### Explosions: `fpv_wind_add_blast`

```c
fpv_wind_add_blast(explosion_pos, explosion_radius);  /* once, when it goes off */
```

A gust runs outwards from the point at 100 m/s and has passed within
about a second. It is about 40 m/s at `radius` from the centre, a fifth
of that at three radii, and nothing past twelve. The drone gets shoved
and has to recover. Use the game's own explosion radius, or about 2–5 m
for a grenade and 8–10 m for a car or a fuel tank.

### What it costs

When there is a weather wind and shelter is on: about 50 rays through the
raycast callback every 0.1 s, plus one per rotor source per frame. All of
them are cast on the host thread inside `fpv_drone_tick`. They are longer
than the collision probes (up to 80 m upwind and 400 m down). In the
simulation it costs a few microseconds per step. If the game's line test
is slow, give the callback a cheaper path for long rays, or set
`FPV_WIND_NO_SHELTER`.

### Seeing it

`frame.wind` is the air's velocity at the drone (host units / s) after
everything: weather, height, shelter, gusts, sources and blasts. Use it
for the game's own effects if you like (a wind sound in the goggles,
dust). F8 > Fly > Wind shows the speed, where the wind comes from and how
sheltered the drone is. DECISIONS D30 describes the model.

### Case study: GTA San Andreas

- The game has one wind for the whole map. `CWeather::Wind` (0xC812F0)
  is set by the weather type: 0 to 0.25 when sunny, 0.7 cloudy, 1.0 in
  the rain storms, 1.5 in the sandstorm. `CWeather::WindDir` (0xC813E0)
  always blows towards the north-east, with its own scripted 1–2 s
  wobble and gusts on top. The mod low-passes `WindDir` over 4 s for the
  direction and maps `Wind` 1.0 to 11 m/s.
- The extra sunny weathers have `Wind` 0: dead calm everywhere, even 300 m
  up, while the game's wind sound still blows there (it gets louder with
  the camera's height). Players noticed. The mod never gives less than
  2 m/s; real air is never that still, least of all high up. Do the same
  in any game whose weather can be perfectly calm.
- The game does apply its wind to its own aircraft
  (`CVehicle::FlyingControl` works on the airspeed), but without height
  or shelter. The stronger wind you hear on a rooftop is the wind sound
  scaling with the camera's height, not physics.
- Interiors are separate areas placed high above the map
  (`CGame::currArea != 0`), so the mod sets `FPV_WIND_ENCLOSED` there.
- Helicopters (`m_nVehicleSubType` 3) from the vehicle pool become rotor
  sources: the rotor speed at +0x84C (the game counts 0.15 or more as
  full power), the mass at +0x8C and the rotor radius from the model's
  collision box.
- New entries in `CExplosion::aExplosions` (0xC88950, 16 entries of 0x7C
  bytes) become blasts.

## Kamikaze builds

> **Dev-gatekept.** The kamikaze feature is only documented here and in
> DECISIONS.md, and only exists in builds with `LIBFPV_KAMIKAZE`. Don't
> mention or enable it anywhere public: not in a README, install guide,
> mod page, store page or changelog, not in CI/CD or release builds, and not
> in a game integration's default build. Turn it on only in local
> development builds until it is cleared for release.

Build libfpv with `-DLIBFPV_KAMIKAZE=ON` for kamikaze (strike) FPV. The
drone then carries a warhead, and `libfpv.h` gains the "Kamikaze" section
(the definition `LIBFPV_KAMIKAZE` comes with `libfpv::fpv`; a host not
using CMake defines it itself). The rules are in the header and in
DECISIONS D24. In short:

| Switch | Channel | Keyboard | Gamepad |
|---|---|---|---|
| Bomb arm | AUX7 | K (toggle) | West / X / Square (toggle) |
| Explode | AUX8 | X (hold) | right shoulder |

The bomb is not installed by default: until it is, the drone flies like in
any other build (no extra weight, no warhead OSD, the bomb switches do
nothing). The player installs it in the overlay (F8 > Kamikaze > "Bomb
installed"), or the game calls `fpv_kamikaze_set_installed(1)` (a
kamikaze mission, a drone picked up). It stays installed for the session,
not per drone: a new, respawned or repaired drone carries it too. It is
not saved, so every launch of the game starts without it.

The motors only arm with the bomb switch off. Take off, flip it, and the
warhead is live (OSD `BOMB_ARMED`; with an arming delay set, `ARMING 3` first).
Any touch then sets it off, even a light tap on the ground, and so does the
explode switch. With the bomb switch off nothing explodes. The warhead
weighs something (by default 60 % of the quad), so a loaded drone climbs
and turns slower.

All kamikaze settings are in the overlay's own tab, F8 > Kamikaze (only in
kamikaze builds): the bomb, the arming delay, the impact fuse, the payload,
the warhead OSD line, the ending (below) and a **Bomb switches** setup.
That one works like the radio setup in F8 > Radio but binds only AUX7 and
AUX8, on a radio that is set up already, and merges them into
`radio_mappings.json` without touching the other bindings. The main radio
setup never asks for the bomb switches (a new full setup keeps the bomb
switches that still fit). Keyboard and gamepad keep their fixed keys.

### What the game does

libfpv tells the game what happened and leaves the consequences to it:

```c
fpv_detonation d;
d.struct_size = sizeof d;
while (fpv_kamikaze_poll_detonation(&d)) {
    play_explosion(d.position);               /* host coords */
    damage_around(d.position, d.object_id);   /* object_id: a direct hit on a surface object */
    /* then one of the endings below */
}
```

While destroyed (`frame.flags & FPV_DRONE_DETONATED`), the wreck sits frozen
where it exploded, the props are broken, `frame.video` reports no signal,
the OSD is blank (whatever the OSD disable switch), and
the drone does not arm. Don't draw the drone model.

Then comes the **ending**, which the player chooses in F8 > Kamikaze (saved
in `settings.json`). Read it from `fpv_kamikaze_get_config(&c)` at the
moment `fpv_kamikaze_poll_detonation` returns the detonation:

1. **View** (`c.ending_view`), what the screen shows:
   - `FPV_KAMIKAZE_VIEW_SIGNAL_LOST` (default): the FPV view, with full
     static while `FPV_DRONE_DETONATED` is set and a blank OSD, like a
     real lost feed. Consider hiding the game's own UI
     too, as while `FPV_DRONE_OSD_DISABLED` is set
     ([OSD disable switch](#osd-disable-switch)), so the screen shows only
     the static.
   - `FPV_KAMIKAZE_VIEW_CUTSCENE`: the game camera shows the explosion from
     another angle (an external shot of the blast).
2. **Duration** (`c.ending_duration`, 0..30 s, default 3): how long the view
   shows. 0 skips the view for Respawn and Recall: they happen right after
   the blast.
3. **Then** (`c.ending_then`), what the game does when the time is up:
   - `FPV_KAMIKAZE_THEN_RESPAWN` (default): `fpv_drone_repair(drone)` and
     `fpv_drone_reset(drone, spawn, rot)`, the next drone at the spawn.
   - `FPV_KAMIKAZE_THEN_RECALL`: end the flight, put the drone away, and
     give control back to the pilot (the character on foot, in a game that
     has one).
   - `FPV_KAMIKAZE_THEN_NOTHING`: keep the view on screen until the player
     acts. The view shows and stays whatever the duration, also with 0 s.

**The player's own commands work at once during the view**, timed or held:
their respawn, recall, and "Move <pilot> here" (Y, see [Input](#input))
cancel the timer and replace step 3. Never make the player wait for the
ending to finish.

libfpv never times the ending: the game does, with its own clock, so the
view pauses with the game (a pause menu stops the countdown). Keep the
values the game read at the detonation; the player may change the setting
during the view, which only counts for the next one.

**Declare what the game can do**, once at startup after `fpv_global_init`:

```c
fpv_kamikaze_set_ending_support(
    (1u << FPV_KAMIKAZE_VIEW_SIGNAL_LOST) | (1u << FPV_KAMIKAZE_VIEW_CUTSCENE),
    (1u << FPV_KAMIKAZE_THEN_RESPAWN) | (1u << FPV_KAMIKAZE_THEN_RECALL) | (1u << FPV_KAMIKAZE_THEN_NOTHING),
    FPV_KAMIKAZE_VIEW_SIGNAL_LOST, FPV_KAMIKAZE_THEN_RESPAWN);  /* the game's defaults */
```

The overlay then only offers those choices, and a saved choice the game
can't do reads back as the game's default (the saved choice itself is
kept, so it comes back where it is supported). A supported saved choice
always wins over the default. Not calling it means everything is
supported, with the defaults signal lost, 3 s, respawn. Signal lost and
Nothing need nothing from the game, so libfpv always allows them. A game
without a character to hand control back to leaves out Recall.

`fpv_drone_repair` always gives a new drone with a new, safe warhead. If the
pilot's bomb switch is still on, it must be turned off before the new
warhead can arm, so a respawn never starts with a live bomb.

The standalone example supports both views and then Respawn or Nothing
(the default: it has no pilot to recall to); R respawns at any time, also
during the view. `fpv_standalone [--view signal|cutscene] [--duration S]
[--then respawn|nothing]` sets the ending as the overlay would, and
`fpv_standalone --kamikaze-demo` flies a scripted strike that checks every
rule and the ending options (no view, a held view, an early respawn that
cancels the timer), and exits 1 on failure.

### Tuning

`fpv_kamikaze_set_config` sets the arming delay (default 0, none), the contact fuse threshold
(`min_impact_speed`, default 0.25 m/s), the payload (`payload_mass` kg and
`payload_drop` below the frame, -1 = automatic), the ending
(`ending_view`, `ending_duration`, `ending_then`) and `FPV_KAMIKAZE_NO_OSD`.
Players edit the same values in the overlay (F8 > Kamikaze), and both are saved
in `settings.json`, so only set them at startup if the game must override
the player (a mission with a fixed payload, for example).
`fpv_kamikaze_get_state` reports the warhead state and the payload and
all-up mass for the game's own UI. A quad preset can carry its own default
payload:

```json
"payload": { "mass": 0.8, "drop": 0.04 }
```

Without `FPV_KAMIKAZE_NO_OSD`, the warhead state is part of `frame.osd`:
it is Betaflight's "Custom message 1" element, centred on row 3 by
default. Players move or hide it like any other element, in the overlay
(F8 > OSD > "Bomb status") or in the Betaflight Configurator's OSD tab,
and other elements, the post-flight stats and the OSD disable switch
cover it as they do on a real quad. With the flag, the game draws its own,
from `fpv_kamikaze_get_state`.

## Lessons from real integrations

These notes come from integrating libfpv into GTA IV
([gtaiv-fpv](https://github.com/hammerill/gtaiv-fpv): a native ASI mod on
a closed, 32-bit D3D9 engine with no source). They apply to most game mods.
Engine-specific details (addresses, natives) are in that repository's
`MODLOG.md`. The case study at the end summarises them.

A game repository's `MODLOG.md` is a historical log: it records what was
tried and learned at the time, and it may be outdated. When it disagrees
with this document or `libfpv.h`, this document and the header win. Use
mod logs only as a reference for engine specifics (addresses, natives,
engine quirks), not for how to use libfpv.

### Building and loading

- **Match the game's bitness.** A 32-bit game can only load a 32-bit DLL.
  Build with `cmake/toolchains/mingw-w64-i686.cmake`, which keeps
  `-msse2 -mfpmath=sse`: with x87 maths a fast drone tunnels through walls
  (DECISIONS D19b). Check the target exe with `file` before you start.
- **Loading at runtime is fine.** You can include `libfpv.h` with
  `LIBFPV_STATIC` defined (to get only the declarations), `LoadLibrary` the
  DLL from the game folder, and resolve the functions you use into a table
  (an X-macro list of names keeps this short). Then the mod builds without
  an import library, and libfpv loads only when the player first uses it.
- **`fpv_global_init` pins `libfpv.dll`.** It stays loaded and locked
  until the game exits. If your mod hot-reloads its own DLL, keep anything
  libfpv calls back into from its own threads (the log callback above all)
  in the module that is never unloaded. Otherwise libfpv calls into unmapped
  code after a reload. A libfpv build that adds exports needs a game
  restart.
- **Use the shared data folder by default.** Pass the folder from
  `fpv_shared_data_dir()` as `cfg.data_dir`, not a folder inside the game
  or the mod. Then the pilot's radio mappings, Betaflight tune and
  settings follow them from game to game, and reinstalling the game
  doesn't wipe them. Fall back to the mod's own folder only when the call
  fails (no home folder). If an earlier version of your mod used its own
  folder, copy it into the shared one the first time, but only while the
  shared folder doesn't exist yet: if it does, another game already set it
  up and the pilot's setup there wins. Keep the mod's own files (test
  triggers, screenshots) in the mod's folder. Ship the same libfpv version
  as the other games, because each one upgrades the shared EEPROM to its
  own Betaflight. reVC-mac-fpv's `src/fpvapi.cpp` shows the whole thing.
- **Don't write to stdout.** Pass `cfg.log` and send libfpv's log to the
  mod's log file. One log file with both the mod's and libfpv's lines
  is the easiest to debug.

### Coordinates and the camera

- **Z-up worlds** describe themselves with `coords`: GTA IV is
  `{FPV_AXIS_POS_X, FPV_AXIS_POS_Z, FPV_AXIS_POS_Y, 1.0f}` (east, up,
  north, metres).
- **Script cameras usually take Euler angles. Measure the engine's order;
  don't guess it.** The vertical slice looked right in level flight with the
  wrong order, and the view mirrored only in flips. A quick way to measure:
  place the camera at known rotations, project points on the predicted
  forward/up/right axes to the screen with the engine's own
  world-to-screen function, and see which of the six orders keep them
  where they should be. GTA IV builds cameras as
  Rz(heading) · Ry(roll) · Rx(pitch). Use the camera's `forward`, `up` and
  `right` vectors rather than the quaternion. Decompose them for that
  order, and handle the knife-edge singularity (put everything in heading
  when roll is ±90°). Test inverted, knife-edge and past-vertical
  attitudes.
- **Choose the near plane per frame for the FPV view.** The lens sits a few
  centimetres above the ground. With a fixed near plane of 0.2 m, the road
  under the camera was clipped away and the screen showed whatever was left
  there from the previous frame. A very small fixed plane (0.05 m) caused
  z-fighting instead. Instead, cast rays through the frustum's centre, edge
  midpoints and corners (use the widest aspect you support), and set the
  near plane to 0.8 × the closest depth along the view axis. Clamp it to
  about 0.02–0.2 m. `frame.camera.near_plane` is only a default.
- **Recompute the lens FOV for the real aspect.** libfpv widens
  `fov_y_degrees` for the fisheye using `aspect_hint` (4:3 analog). For a
  16:9 back buffer, redo it from the same formula. Let
  `edge(a) = (1 + k1·r2 + k2·r2²) / zoom` with `r2 = 1 / (1 + a²)` (the
  squared radius of the top edge's centre). Then
  `fov = 2·atan(tan(fov_y/2) · edge(aspect_hint) / edge(real_aspect))`.
  Clamp it to an angle the engine accepts (170°). At 16:9 the default FPV
  camera comes out at about 119°.
- **If you don't draw the lens pass, pass `FPV_INIT_NO_LENS_DISTORTION`.**
  Otherwise the FOV is widened for a warp that never happens, and the view
  looks too wide. The lens and the OSD only belong to the FPV view, not to
  the chase or LOS cameras.

### World collision through `fpv_world_set_raycast_callback`

When the world can't be exported as triangles (streamed, closed engine),
answer libfpv's rays with the engine's own line test.

- **Find the engine's general line probe.** A ground-height function
  answers only downward rays, so the drone flies through walls. In GTA IV,
  disassembling the handler of a ground-height native led to the general
  probe it calls. A native's handler address is easy to get at runtime
  from the script hook. On a closed engine, signature-check the call sites
  you rely on, and fall back to ground-only if they don't match.
- **Exclude the player's own body, all of it.** The pilot character is
  near the spawn point, and a ray from the drone hits it. Passing the
  character's main physics instance as "exclude" was not enough, because a
  ped has several instances (for example its ragdoll). When a ray hits the
  ignored entity, restart it 25 cm past the hit, up to 8 times.
- **Turn hits into `object_id`s** that tell you what was hit, for example
  `kind << 28 | handle` (ped, vehicle, object, building). libfpv returns
  the id in `frame.impacts`, so the game can react to the right entity.
- The callback runs on the host thread inside `fpv_drone_tick`, so engine
  calls that must be made on the game thread are safe there.
- The same raycast is useful outside libfpv: for the near plane, for
  placing the character at "move here", and for logging a probe sweep
  around the spawn to check it works.
- Seen in GTA IV, not yet explained: `frame.altitude` is often -1 once the
  drone is more than a few metres up. Don't build gameplay on it yet.

### Impacts and game effects

`frame.impacts` holds the speed, impulse, position and `object_id` of each
hit. What worked in GTA IV:

- Ped hit at 4 m/s or more: ragdoll, damage of about 2 × speed, and a
  push. Vehicle hit at 5 m/s or more: health loss of about 3 × speed, and a
  push. Prop: push only.
- Use a per-entity cooldown (0.5 s), and start it only when an effect was
  applied. If it starts on every contact, a 2 m/s graze just before the
  real hit uses up the cooldown, and the real hit does nothing.
- Never apply effects to the player character.

### Drawing the OSD and the overlay (D3D9 and other hooked renderers)

- **Disturb the game's frame as little as possible.** In GTA IV, capturing
  a full state block (`D3DSBT_ALL`) and swapping the render target and depth
  surface each frame made the next frame show stale regions and jumps in
  exposure. This happened even when nothing was drawn, and it looked like
  texture flicker. Draw on the back buffer that is already bound, and save
  and restore only the states you change.
- Fixed-function D3D9 is enough: `D3DFVF_XYZRHW | DIFFUSE | TEX1`,
  `DrawIndexedPrimitiveUP` with 32-bit indices, and managed `A8R8G8B8`
  textures. libfpv's textures are RGBA, so swap them to BGRA when you
  upload. Upload again when the texture's revision changes.
- The fisheye works without shaders: `StretchRect` the back buffer into a
  `D3DPOOL_DEFAULT` render-target texture, then draw it back through a
  64×36 grid ([Lens distortion](#lens-distortion)). Release that texture
  when the device is lost.
- The game's present hook runs on the render thread, and the drone ticks on
  the script thread. Copy the frame under a mutex and draw from the copy.
- Hooks provided by a script hook can disappear: GTA IV's ScriptHook
  restarts every script thread when a save is loaded, and the D3D hook went
  with it (no OSD after "Load Game"). Detect the restart, and register the
  draw hook again if no frame arrives soon after.
- Hide the game's own HUD and minimap while flying (every frame, if that
  is how the engine does it). Whatever stays on screen in FPV must go too
  while `FPV_DRONE_OSD_DISABLED` is set ([OSD disable switch](#osd-disable-switch)).

### Input

- Forward keys to `fpv_input_key` on edges, and only while the game window
  has focus. **Take the first reading of each key as the baseline, not as a
  press.** On the developer's PC, F8 read as held down system-wide, and the
  overlay opened by itself at startup.
- Forward letters and digits by physical position (scancodes), not by the
  label the layout puts on them: `'W'` is the key right of Tab. Keycodes
  (`SDLK_*`, Windows `VK_*`/`GetAsyncKeyState('W')`) follow the layout, so
  on AZERTY throttle up and yaw left moved to the keys labelled W and A,
  away from S and D. On Windows, poll
  `MapVirtualKeyW(0x11 /* W scan code */, MAPVK_VSC_TO_VK)` (or read the
  scan code from `WM_KEYDOWN`) instead of the letter's VK.
- While `fpv_overlay_wants_input()` is non-zero, take player control away
  from the game (also on foot) and give the keys (Esc, Tab, Backspace,
  Space, arrows) to the overlay. The overlay draws its own cursor. Feed it
  the cursor position relative to the game window.
- Games often have no text output you can use from a mod. A help text
  drawn through the game was also pushed off screen by the fisheye. Put
  game-specific commands in the overlay with
  `fpv_overlay_set_action(id, label)` and `fpv_overlay_poll_action()`
  (GTA IV adds "Move Niko here (Y)" in F8 > Fly).
- If the game can end a flight with the player moved to the drone (GTA IV:
  Move Niko here; Super Mario 64: Mario under the recalled drone), put it
  on **Y**, so the same key does it in every libfpv game. Like the game's
  other own keys, it can follow the label (the key marked Y on any layout).
- The radio mapping (`radio_mappings.json`) is per data folder. A player
  who already set up the radio for another libfpv game can copy that file
  into the new game's data folder.

### Spawning, flights and the world around the drone

- **Ignore the first frames after `fpv_drone_create`.** They can still
  show the origin or the previous drone, because the simulation clock is
  global and not reset for each drone. The GTA IV mod skips frames until
  `frame.sim_time` is 50 ms past its first reading. Before that, moving
  the camera or anything else to the frame's position teleports it to
  (0, 0, 0) for a moment. This should be fixed in libfpv.
- **Spawn on the ground in front of the character**, facing the
  character's heading (the frame's bottom rests at `position`). Then
  freeze the character and switch to the script camera.
- **Streaming follows the player, not the camera.** In an open world the
  terrain, buildings, LODs, peds and cars around a drone far from the
  character stop loading. GTA IV has no single function to redirect
  streaming. So the mod uses the character as an anchor: once the drone is
  80 m from home, the character is made invisible, frozen, without
  collision, invincible and ignored by everyone, and moved to the drone
  every 10 m. It is taken out of its car if needed. Within 40 m of home, or
  at the end of the flight, put the character back (and back into its
  car). Load the area first.
- **The pilot stays visible at the piloting spot.** This is a requirement
  for integrations. When the game moves the character (or another world
  anchor) along with the drone for streaming or AI, as above, it must
  leave a stand-in of the pilot where the flight started, ideally in an
  FPV piloting pose (standing, holding a radio transmitter), so that
  turning the drone back home shows the pilot there, as in real FPV.
  Super Mario 64 does this (Mario stays visible where he stands) and so
  does GTA San Andreas; GTA IV does not yet (the hidden character leaves
  nobody at home).
- "Return the character to where the drone is" (move here): ray down from
  the drone, load the area, put the character 1 m above the ground facing
  the drone's heading, and put the game camera behind it.
- Auto-pause (default 0.5 s) covers pause menus and loading screens. Loading
  a save while flying is not handled yet.

### Audio

A `waveOut` thread (44.1 kHz, stereo, 16-bit, 4 × 512-frame buffers) that
pulls `fpv_audio_read` is enough, and the sound plays through the game's
output device. Stop and join it before deleting the drone.

### Sound effects for the game's reactions

Short sounds as reactions to the drone are a nice touch, and worth adding:
a click when the drone launches or a move-here works, a back sound on
recall, a buzz when an action is refused, a tick on a camera switch, and a
bump on an impact (a minimum speed, e.g. 3 m/s, and a short cooldown, so
scrapes don't play a sound on every contact frame).

- **Use only sounds the game already has.** libfpv will never ship its own
  sounds for this. If the game has no fitting sound for a reaction, skip that
  reaction.
- Play them through the game's sound system, so they follow its volume and
  positioning: the menu sounds for actions, and the game's collision sound
  at the impact point (with the surface that was hit, if the engine picks the
  sound by surface).
- Super Mario 64 uses `SOUND_MENU_CLICK_FILE_SELECT`, `SOUND_MENU_CAMERA_BUZZ`,
  `SOUND_MENU_CHANGE_SELECT` and `SOUND_ACTION_BONK`. GTA San Andreas uses the
  frontend events 1 to 4 (select, back, highlight, error) and
  `CAudioEngine::ReportCollision` with the drone as plastic.

### Testing without a human

- `FPV_INPUT_SOURCE_HOST` / `fpv_input_set_channels` let the mod fly
  scripted test flights. The GTA IV mod's autotest arms, holds altitude,
  pitches forward into a car or ped, and logs everything. For the default
  Freestyler 5" in ANGLE mode, hover is about -0.56 throttle (−1..1).
- Log the drone state every couple of seconds (position, flags, altitude,
  throttle, `arming_disable_text`, FOV, near plane). Most bugs showed up
  there before they showed up on screen.
- Flicker is hard to see in screenshots. Count A-B-A pixel toggles across
  consecutive frames of a recording, and compare the count with and without
  the change.
- A file the test harness creates to start the autotest must be deleted
  afterwards, so a normal game start doesn't run it.

### Case study: GTA IV (1.2.0.59)

| Hook | How gtaiv-fpv does it |
|---|---|
| Loader | FusionFix's ASI loader. A small `.asi` owns the script thread, the log and the D3D hook, and hot-reloads the core DLL. |
| Script access | Aru's C++ ScriptHook, called from MinGW through mangled-name `GetProcAddress` and a hand-built `ScriptThread` vtable. Natives are called by joaat hash. |
| Coordinates | X east, Y north, Z up, metres. |
| Camera | A scripted camera. Euler order Rz(heading) Ry(roll) Rx(pitch). Per-frame near clip from frustum probes. |
| Collision | The engine's physics line probe (found by disassembly), mapped to peds, vehicles and objects through the entity pools. |
| Effects | Natives: ragdoll, damage, health, apply force. |
| Drawing | ScriptHook's D3D9 device hook. Fixed-function triangles, and the fisheye through a 64×36 grid. |
| Audio | `waveOut`. |
| Streaming | The player character used as an invisible anchor. |
