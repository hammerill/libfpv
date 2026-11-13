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

## The video feed

Real FPV is watched through a video link, and `frame.video_feed` describes
what the goggles show. A game that draws it passes `FPV_INIT_VIDEO_FEED` in
`fpv_global_config.flags`. Only then can the player turn on two effects in
F8 > Camera > "Video feed"; in other games the settings show as
unavailable. Both are off by default, and while they are off `flags` is 0
and the FPV view is drawn as above.

- **Analog video noise** (`FPV_VIDEO_FEED_NOISE`). A link budget runs on
  the simulation thread: the VTX's power, free-space loss at 5.8 GHz, the
  ground's reflection when flying low and far, the patterns of both antennas
  (a flip that points the VTX antenna at the pilot, or flying right
  overhead, costs a lot), and obstruction measured with rays through the
  first Fresnel zone, so the signal fades as the drone slides behind a
  corner. The rays test the static surfaces and objects, and the game's
  raycast callback if it set one (one ray per `fpv_drone_tick`, from the
  pilot position to the drone, which can be long: hundreds of metres). On the host thread, multipath fading runs line by line at the
  Doppler of the drone's speed through two diversity antennas. Each line's
  carrier-to-noise ratio then sets what that line looks like:
  - grain stretched along the line;
  - FM clicks (sparklies) at Rice's rate below the threshold, white on dark
    and black on bright;
  - colour speckle, and the colour killer once the burst is lost;
  - torn line starts from sync hits, which the PLL pulls back over the next
    lines;
  - a rolling picture with the black blanking bar once vertical sync goes;
  - snow when nothing is left.

  Hard impacts jolt the feed, a sagging 1S pack browns out a whoop's VTX,
  and ESC noise on the power wiring draws crawling diagonal stripes with
  throttle (strongly on a cheap VTX). Digital links break into macroblocks
  and freeze instead.
- **Camera resolution and aspect** (`FPV_VIDEO_FEED_RESOLUTION`). The
  picture is drawn at the feed's resolution and at the camera's aspect, with
  black bars like in goggles: a 4:3 camera on a 16:9 screen gets bars at the
  sides. Analog is 720 samples by 240 (NTSC) or 288 (PAL) lines per field,
  interlaced and shown by bobbing. Luma is limited to `luma_cycles` (the
  camera's TVL through the link's ~5 MHz, about 400 TVL whatever the
  camera claims). Colour is limited to `chroma_cycles`, and fine detail near
  the colour subcarrier decodes as rainbows. The camera's edge enhancement
  adds halos. Digital uses the transmitted mode's pixels (e.g. 720 × 540 for
  a 4:3 camera in HDZero's 540p90).

The easy way is `docs/renderers/fpv_video_gl3.h` (OpenGL 3.3, GLES 3,
WebGL 2). It replaces the lens shader above and draws the OSD too:

```c
fpv_video_gl3 v; fpv_video_gl3_init(&v);   // once
// every frame, the scene rendered from frame.camera into scene_tex:
fpv_video_gl3_draw(&v, &ui, &frame, scene_tex, scene_w, scene_h, fb_w, fb_h, 1);
fpv_gl3_render(&ui, fpv_overlay_render(dt), fb_w, fb_h);
```

Render the scene with `fov_y_degrees` as the vertical FOV at the screen's
aspect, as usual. When the picture is 4:3, its middle is used. With the
resolution effect on, the lens works on the camera's own aspect
(`aspect_hint`), so the FOV is exact. A host with another renderer ports it
in three steps:

1. **The signal.** Draw a full-screen pass into a texture of `raster_width`
   × an analog field's lines (240 NTSC, 288 PAL) with the resolution effect
   on, or into a texture of the picture's size with it off. Redraw it only
   when `field` changes: the feed runs at 59.94 or 50 fields a second (the
   digital mode's frame rate), and the texture holds still in between, like
   goggles. For each pixel:
   - Find its field line. `roll` adds `roll × 263` (313 PAL) lines, wrapping;
     lines past the picture are the blanking, drawn black.
   - Read that line's two texels from `lines` (uploaded as a 2 ×
     `line_count` RGBA8 texture, nearest; the layout is in libfpv.h) and
     shift the sample by its horizontal shift.
   - Sample the scene through the lens. The lens is the formula above, with
     the picture's aspect.
   - With the resolution effect, low-pass the luma across the line to
     `luma_cycles` and the colour (YIQ) to `chroma_cycles`.
   - Add the line's grain and sparkles, apply its colour gain and hue, and
     the `interference` stripes.
   - Mix toward snow by the line's picture value.

   Analog, with `FPV_VIDEO_FEED_OSD_IN_SIGNAL`: draw the OSD (analog style)
   into a texture first and mix it into the scene samples, so it is noisy
   and soft like the rest of the picture.
2. **The goggles.** Draw that texture with bilinear filtering into the
   picture's rectangle: `picture_aspect` fitted to the screen, black around
   it. For an analog field, move it down half a line when `field_parity`
   is 1 (bob).
3. **The OSD on top**, digital or with the effects off, in the player's
   `osd_style`.

`fpv_video_gl3.h`'s GLSL is the reference for every step. HLSL needs only
renames: `texelFetch` → `Load`, `fract` → `frac`, `mix` → `lerp`. Its hash
avoids `sin`, so it gives the same noise on every GPU. Pass
`FPV_INIT_VIDEO_FEED` once the port works.

Presets give the parts: a quad's `"camera": {"model": "Caddx Ratel 2"}`
names a file in `presets/camera` (`tvl`, `aspect` "4:3" or "16:9", `system`
NTSC or PAL, `sharpening`; a digital camera's `height` and `fps`). The VTX
file gives `powerMw`, `powerNoise` (0..1, motor noise reaching the video),
`minCellVoltage` (brownouts) and, for digital links, `height`, `fps`,
`freezes` and `sensitivityDbm`.

## The RC link

By default the radio link is perfect: every stick movement reaches
Betaflight, however far the drone flies. The player can turn on a
simulated ExpressLRS link in F8 > Drone > "RC link". It is off by default,
and a game needs nothing for it beyond what the video link uses: the pilot
position (`fpv_drone_set_pilot_position`) and, if it has one, the raycast
callback.

With it on, each packet from the radio has to get through:

- **The link budget.** The transmitter's power, the antennas (the
  receiver's dipole at the back of the frame, with the frame, battery and
  stack shadowing it when the pilot is ahead of the drone), free-space loss
  at 2.4 GHz or 900 MHz, the ground's reflection when flying low and far,
  and what stands between the pilot and the drone. That last part comes
  from the same rays as the video link (static surfaces, objects and the
  raycast callback), costed at the link's band: walls and trees take less
  at 900 MHz than at 2.4 GHz.
- **Fading.** The signal fades with the drone's speed, deeper behind
  obstacles where only scattered paths remain.
- **Packets.** Each packet at the mode's rate (1000 to 25 Hz) arrives or
  is lost, depending on the signal against the mode's sensitivity
  (-104 dBm at F1000 down to -123 dBm at 900 MHz 25 Hz). A lost packet
  leaves Betaflight with the sticks of the last one. After 150 ms without
  packets Betaflight shows RXLOSS and starts its failsafe stages. After 1 s
  the receiver loses the sync and needs a moment of good signal to find it
  again.
- **Power.** The player picks the transmitter's power (10 mW to 1 W) and
  ELRS's dynamic power. Dynamic power starts at 10 mW and steps up and
  down on the receiver's telemetry (SNR, or RSSI for F1000, and LQ). It
  jumps to the maximum when the LQ drops below 50 %, drops suddenly, or the
  telemetry is lost while armed.

Betaflight gets the link statistics the way it gets them from an ELRS
receiver over CRSF. The OSD's link quality element shows `RFMD:LQ` (for
example `29:100` at 500 Hz), and the RSSI dBm, RSNR and TX power
(`osd_link_tx_power_pos`) elements and the LQ, RSSI and RSNR warnings
work. The player places them in F8 > OSD or in the Configurator.

The packet rates, from the overlay or `fpv_rc_link_mode`:

| Mode | Rate | Sensitivity | For |
|---|---|---|---|
| 2.4 GHz F1000 | 1000 Hz | -104 dBm | racing |
| 2.4 GHz 500 Hz | 500 Hz | -105 dBm | freestyle (default) |
| 2.4 GHz 250 / 150 Hz | 250 / 150 Hz | -108 / -112 dBm | freestyle, more range |
| 2.4 GHz 50 Hz | 50 Hz | -115 dBm | long range |
| 900 MHz 200 / 100 Hz | 200 / 100 Hz | -112 / -117 dBm | through walls and trees |
| 900 MHz 50 / 25 Hz | 50 / 25 Hz | -120 / -123 dBm | long range |

In the open, with the drone 30 m up and 100 mW, packets start to drop at
about 4.5 km at 2.4 GHz 500 Hz, 9 km at 2.4 GHz 50 Hz and 15 km at 900 MHz
50 Hz. Indoors and behind buildings it is much less, as with a real link.

Games normally leave the setting to the player. A game can still read or
set it with `fpv_rc_link_get_config` and `fpv_rc_link_set_config` (it is
saved like the player's choice), and read the link's state (LQ, RSSI, SNR,
TX power, whether the receiver is in sync) with `fpv_rc_link_get_state`,
for its own HUD, for example.

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
  0.5 m/s; real air is never that still, least of all high up. A floor of
  2 m/s felt too windy on calm days: libfpv's height profile and gusts
  already add to it up high. Do the same in any game whose weather can be
  perfectly calm, and keep the floor low.
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

## The pilot

In a game with a character, the character is the pilot: the drone launches
from them, the LOS camera and the video link start from them, and they stay
in the world while the drone flies.

### The F key: back to the pilot, the drone left flying

Implementing **F** is standard for every libfpv game with a character:

- **F** puts the view back on the pilot character (the game's own camera and
  controls), while the drone stays intact and present in the world. Don't
  delete it, don't disarm it, keep calling `fpv_drone_tick` and keep drawing
  it. The radio still flies it: technically this is another line-of-sight
  view, from the character's own camera. If it is in the air when the player
  presses F, it keeps flying (or falls, if the player lets go of the sticks).
- **F7** launches a drone when there is none, as before. With a drone left
  in the world by F, F7 doesn't spawn a new one: it puts the view back on
  that drone. What F7 does while flying stays the game's (e.g. recall).
- Tell libfpv: `fpv_set_view(FPV_VIEW_PILOT)` on F, and the view you go
  back to on F7 (`FPV_VIEW_FPV`, or the chase / LOS view the player had).
  The drone sound treats the pilot view like LOS (heard from the pilot, see
  [Audio](#audio)): pass the game camera every frame with
  `fpv_audio_set_listener(camera_position, camera_rotation)`, and
  `fpv_audio_set_listener(NULL, NULL)` when you leave the pilot view.
- The character walks and drives in the pilot view. Move the pilot with it,
  `fpv_drone_set_pilot_position(drone, feet)`, so the LOS camera, the video
  range and the ear follow (also the home point, if you set it to the pilot,
  see [GPS](#telling-libfpv-where-home-is)).
- While the drone is left in the world, keep the game's own reactions to it
  (impacts, damage, pedestrians) running as during a flight, and keep the
  game's HUD shown: the player is on foot.
- Y (move the pilot to the drone) can stay available from the pilot view.

### The piloting pose

While the player flies from the drone's views (FPV, chase, LOS) and the
radio is in use, the character at the piloting spot (or its stand-in) plays
an FPV piloting pose: standing, holding a radio transmitter in both hands.
"In use" is `FPV_DRONE_RADIO_ACTIVE` in `frame.flags`: armed, or a channel
moved in the last 3 s. A radio is never perfectly still (AUX pots and
switches jitter by 1-2 %), so the flag only counts moves of more than 5 % of
a channel's range; don't use your own threshold. Without it, release the
pose back to the game's own idle animation.

Not in the pilot view (F): there the character is the player's again and
behaves as if there were no drone, with the game's own animations (idles,
walking, driving), while the radio still flies the drone. Release the pose
when the player presses F, and don't play it again until they go back to a
drone view.

### The player's choices: `fpv_pilot_set_support`

The stand-in ([Standing in](#standing-in-for-the-drone))
and the pose are the game's code, but whether they happen is the player's
choice, the same in every game (F8 > Fly > Pilot, both on by default).
Once after `fpv_global_init`, say what the game does and its own stand-in
distances:

```c
// the stand-in takes over beyond 20 m and ends back within 10 m (host units)
fpv_pilot_set_support(FPV_PILOT_STAND_IN | FPV_PILOT_POSE, 20 * upm, 10 * upm);
```

Then read the choice whenever it is needed (cheap, every frame is fine):

```c
fpv_pilot_config pc = {sizeof pc};
fpv_pilot_get_config(&pc);
bool standIn = pc.features & FPV_PILOT_STAND_IN;  // distances: pc.stand_in_distance, pc.stand_in_return_distance
bool pose = pc.features & FPV_PILOT_POSE;
```

- Declare `FPV_PILOT_STAND_IN` in every game with a character and
  implement it fully ([Standing in](#standing-in-for-the-drone)): it is on
  by default for the player.
- Stand-in off: the character is not carried with the drone. It stays at
  the piloting spot as itself, whatever the distance (the world may then
  load less far from it, and that's the player's call).
- Pose off: the character keeps the game's idle animation, also as a
  stand-in. (In the pilot view it does either way.)
- The overlay only shows what the game supports; a game that doesn't call
  `fpv_pilot_set_support` shows no Pilot section.

## Damage

Make everything that looks like it could damage a prop damage it. That is
the rule for every integration, as on a real quad:

- **Surfaces.** Water, lava, fire, saws, fans, spinning blades: give their
  triangles `FPV_MATERIAL_KILLS_PROPS` (or return such a material from the
  raycast callback). A prop touching one breaks.
- **Everything libfpv can't see.** Fire and flames (also a burning car or
  a molotov's area), water volumes and the sea, bullets
  and shotgun pellets through the props, melee hits, explosions (with or
  without a warhead), a helicopter's rotor: call
  `fpv_drone_damage_props(drone, motor, amount)`, with the prop that was
  hit (the one nearest to the hit point, from `frame.motors[i].position`)
  or -1 for all four. A bullet through a prop: about 0.5 to 1 on that prop.
  Fire or water: about 0.5 per second on all four, every frame
  (`0.5 * frame_dt`). An explosion: 1 on all four within its radius.
- **Don't check the setting.** The player can make the drone immune
  (F8 > Drone > "Props and motors take damage" off, `FPV_SIM_NO_DAMAGE`):
  libfpv then ignores all of it, including `fpv_drone_damage_props`. The game
  always calls it.
- Effects that are only the game's (sparks, smoke, sound) can still play
  when the drone is immune.

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
DECISIONS D24.

`-DLIBFPV_KAMIKAZE_PROMOTE=ON` keeps the plain build (no warhead, no
kamikaze API) but still shows the overlay's Kamikaze tab, filled with a
note about the Kamikaze version on Patreon, an "Open Patreon" button and a
QR code to the page. Both fill the same tab, so CMake refuses to configure
with both options on.

The kamikaze build in short:

| Switch | Channel | Keyboard | Gamepad |
|---|---|---|---|
| Bomb arm | AUX7 | K (toggle) | West / X / Square (toggle) |
| Explode | AUX8 | X (hold) | right shoulder |

The bomb is not installed by default: until it is, the drone flies like in
any other build (no extra weight, no warhead OSD, the bomb switches do
nothing). The player installs it in the overlay (F8 > Kamikaze > "Bomb
installed"), or the game calls `fpv_kamikaze_set_installed(1)` (a
kamikaze mission, a drone picked up). The choice is per quad (preset) and
saved in `settings.json`: it applies to the current quad, switching quads
applies that quad's own choice, and a new, respawned or repaired drone of
the quad carries it too. A game that decides per mission sets it each time
it spawns the drone.

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
     real lost feed. When the player has the video noise on
     (`frame.video_feed.flags & FPV_VIDEO_FEED_NOISE`), the feed already
     is that static: the link reports no transmitter, and
     [the video feed](#the-video-feed) draws the receiver's own snow (grey,
     streaked along the lines, no sync). Keep drawing the FPV view through
     it then, and paint your own static only while the noise is off.
     Consider hiding the game's own UI
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

## Game controls

Most games give the pilot something to do besides flying: open a door, fire,
grab, change the camera. Put those on `fpv_controls_set` rather than reading
keys yourself. The pilot then gets them on the radio as well as on the
keyboard and gamepad, which is the point: a real FPV pilot keeps both hands
on the sticks.

```c
fpv_control_desc c[2] = {0};
c[0].struct_size = sizeof c[0];
c[0].id = 1;
c[0].name = "portal2.fire_blue";      /* stable and unique across games */
c[0].label = "Fire blue portal";
c[0].kind = FPV_CONTROL_INSTANT;      /* once per press (the default) */
c[0].key = 'Q';
c[0].gamepad_button = FPV_GAMEPAD_LEFT_TRIGGER;
c[1] = c[0];
c[1].id = 2;
c[1].name = "portal2.grab";
c[1].label = "Grab";
c[1].kind = FPV_CONTROL_SWITCH;       /* on or off; a key press flips it */
c[1].key = 'G';
c[1].gamepad_button = FPV_GAMEPAD_LEFT_SHOULDER;
fpv_controls_set(c, 2);               /* once, after fpv_global_init */

/* every frame */
uint32_t presses;
fpv_control_poll(1, &presses);
for (uint32_t i = 0; i < presses; i++) fire_blue();
bool grab = fpv_control_poll(2, NULL);
```

- **Act on presses, not on the state, for one-shot actions.** The state is
  read at the simulation's 1 kHz, so a tap shorter than a game frame still
  counts once in `presses`.
- The player binds them in F8 > Radio > "Set up game controls" (all of them,
  one after the other, each one optional) or one by one in F8 > Controls,
  where the keys can be changed too. The setup only offers switch positions
  and buttons that the flight modes and the other controls don't use. Several
  controls can share a 3-position switch, one per position, as the flight
  modes do.
- **Say what kind each action is.** `INSTANT` (the default) is an action
  that happens at the moment of the press, and holding does nothing more:
  fire, use, jump, take a photo. `HOLD` acts for as long as it is held: a
  winch, a boost, zoom. `SWITCH` is on or off: a light, a mode.
- The player picks a button or a switch for each in the setup ("It's a
  button" box). An `INSTANT` control on a radio push button does it on each
  press. On a 2- or 3-position switch it does it on each flip, to ON and
  back to OFF, so the pilot never flips a switch back for nothing:
  `presses` counts both flips. A `HOLD` control on a switch acts while it
  is ON.
- A `SWITCH` control on a radio switch follows the switch; on a key, a
  gamepad button or a radio push button, each press flips it. The first
  reading of a radio switch is where it is, not a press, so a switch left on
  doesn't fire anything when the game starts (for every kind).
- The bindings are saved in `radio_mappings.json` under each radio, by
  `name`. A new full radio setup keeps the ones that still fit. Every game's
  controls stay in the file, so pick names that can't clash
  (`"<game>.<action>"`) and never rename them.
- Keys only count while the overlay is closed. Choose default keys that the
  keyboard fallback doesn't fly with (W A S D, the arrows, Enter, B, T) and
  that the standard libfpv keys don't use (F, Y, F7, F8), and gamepad buttons
  other than SOUTH, EAST and NORTH.
- They never reach Betaflight: they are not AUX channels, and the
  Configurator's Modes tab doesn't see them.

## Portals

For a game with portals or seamless teleporters (two openings that are one),
give libfpv the linked pairs. The simulation then takes the drone through at
its own rate, so a fast drone never stops on the surface around the opening
or shows the wall behind it for a frame.

```c
fpv_portal_link l;
l.a.position = ...;    /* centre of the opening, on the surface */
l.a.rotation = ...;    /* local forward out of the surface, local up = the opening's up */
l.a.half_width = 32;   /* host units */
l.a.half_height = 56;
l.b = ...;
fpv_portals_set(&l, 1);   /* whenever a portal opens, moves or closes; NULL, 0 = none */
```

- Pass only linked pairs. An opening without a partner is just a wall.
- The drone goes through when its centre crosses an opening from the front.
  Its attitude, velocity and spin turn with it, so a drone that dives into a
  floor portal comes out of a wall one flying horizontally, as anything does
  in a portal game. Betaflight gets the new attitude at once.
- The surface around an opening doesn't stop the drone inside it, but its
  frame still hits the rim: a quad wider than the opening can't get through.
- Rays of the raycast callback that hit inside an opening are cast on from
  the linked one, so the world beyond the opening is solid while the drone is
  half way through. The callback needs nothing special. It can (and should)
  keep reporting the wall around the opening as solid.
- `frame.camera` looks through an opening the lens has already passed: the
  lens sits a few centimetres ahead of the centre, so the camera can be on
  the far side before the drone is. Render from `frame.camera` as usual. The
  frame isn't interpolated across a pass.
- `frame.portal_passes` counts the passes of the current drone, and
  `frame.last_portal` says which (link × 2 + the side entered). Play the
  game's own sound or effect when the count changes.

## Pushes and cargo

- `fpv_drone_push(drone, velocity, replace)` changes the drone's velocity at
  once: a launch pad (replace it with the launch velocity), a bouncy
  surface, a gust the game simulates itself (add to it). Attitude and spin
  are kept, and Betaflight flies on from there.
- `fpv_drone_set_cargo(drone, &cargo)` picks something up. Two ways:
  - **On a line** (`line_length > 0`), for picking things up off the
    ground: the cargo hangs from under the frame on a winch line, a body of
    its own that falls, swings, rests and slides on the world (a sphere of
    `radius`). Pass where it is at the pickup as `offset` (drone-local): the
    line starts that long and the winch reels it to `line_length` at
    0.5 m/s, dragging or lifting the cargo to the quad. Only the line's pull
    reaches the quad, through its centre of mass, so the pilot feels the
    weight and the swing while the flight controller keeps its authority.
    Draw the object at `frame.cargo_position` (`frame.cargo_flags` has
    `FPV_CARGO_LINE_TAUT` while it is lifted or dragged), and give it back
    to the game with `frame.cargo_velocity` when it is dropped. Leave the
    game object out of your raycast callback's results: libfpv probes the
    world around the cargo as it does around the drone.
  - **Rigid** (`line_length` 0): strapped to the frame. Its mass and
    inertia add to the quad's, and its sphere at `offset` collides as part
    of the quad. Keep it small and close to the centre: a big sphere or a
    long offset adds enough inertia to make the quad unflyable, and a
    sphere that starts inside the ground pushes the quad the wrong way
    (libfpv doesn't move the drone). Portal 2 FPV first carried cubes this
    way and the quad went out of control at the pickup; the line fixed it.
  Its impacts come with `motor_index` `FPV_IMPACT_CARGO`. A new drone
  carries nothing. With 1 kg rigidly under a 5" freestyle quad, a
  full-throttle climb is about a third of the bare quad's.

## The About tab: name your build

F8 > About shows libfpv's build (version, commit, dirty flag, dates,
platform and compiler, the Betaflight and SimITL commits) and the
integration's own, and a "Copy build info" button copies all of it for a
bug report. libfpv fills in its own part. The game's part comes from three
fields of `fpv_global_config`. **Fill in all three**: a report that names
libfpv's commit but not the mod's can't be reproduced.

```c
cfg.host_name  = "GTA IV FPV";                       /* the mod, as players know it */
cfg.host_build = "1.4.0 (3f2a9c1e7b0d, dirty)";      /* version + full or short commit */
cfg.host_url   = "https://github.com/you/gta4-fpv";  /* where players get it, report bugs */
```

- `host_build` is free text, so put everything that tells your build
  apart: the mod's version, its commit (`git rev-parse HEAD` at build time,
  not at configure time, so a new commit shows without re-running the
  configure step), whether the tree was dirty and, if it matters, the game
  version it targets. Generate it in the mod's build. Don't type it in by
  hand: hand-typed versions go stale.
- The strings are UTF-8 and copied during `fpv_global_init`, so they can be
  temporaries. A NULL or empty field hides its line, and with all three
  empty the section is hidden.
- Older hosts (a smaller `struct_size`) keep working and show no section.

## Lessons from real integrations

These notes come from integrating libfpv into GTA IV
([gtaiv-fpv](https://github.com/hammerill/gtaiv-fpv)) and GTA San Andreas
([gtasa-fpv](https://github.com/hammerill/gtasa-fpv)). Both are native ASI
mods on closed, 32-bit D3D9 engines with no source. The notes apply to most
game mods. Engine-specific details (addresses, natives) are in each
repository's `MODLOG.md`. The case studies at the end summarise them.

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
- **Other mods have hooked the same places.** In a modded GTA SA, CLEO, a
  widescreen fix and a limit adjuster had already redirected calls the mod
  needed. When you redirect a call, send it on to whatever it calls now,
  not to the original function. Before you detour a function, check that
  its first bytes are the ones you expect. If they aren't, install nothing
  and log it, so a different exe or another mod's patch never leaves a
  hook half-installed.

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
- **Better: set the camera inside the game's own camera update.** Code
  that runs after the 3D scene can't change the frame being drawn, and a
  camera the engine recomputes every frame drops what you wrote. An
  earlier GTA SA drone mod couldn't roll its camera for these reasons, so
  it put the player in an RC vehicle. gtasa-fpv steps libfpv from inside
  `CCamera::Process`, at its last call (`CDraw::SetFOV`). There it writes
  the camera matrix and the near plane, and the game then builds the
  frustum and copies it to the renderer itself. That gives full roll and
  no frame of lag. No Euler angles are needed: write the axes straight
  from `frame.camera`.
- **Check the matrix's axes as well.** The "right" row of SA's camera
  matrix is up × forward, which points to the screen's left. Writing
  screen right there mirrored the image.
- **The game's FOV may not be vertical.** SA turns its FOV into a view
  window whose vertical half-tangent is `scale · tan(fov/2)`. The scale
  depends on the aspect ratio and on widescreen fixes. Read the scale back
  from the render camera every frame, and convert libfpv's vertical FOV
  with it.
- **Sometimes the game skips its camera code.** During a replay, or with a
  photo mod that freezes the world, SA doesn't run its camera update, so a
  hook inside it never runs. If the hook didn't run this frame, apply the
  camera yourself after the game's update, with the same steps that update
  takes.
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
- A game's line test may see characters only roughly. SA's line test hits
  only a ped's torso and head, and a drone 0.45 m up flew between a ped's
  legs. That's acceptable for flying, but don't expect hits on arms or
  legs.

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
- **Hurt peds through the game's own damage event, not by writing their
  health.** Name the player as the attacker and use a weapon type that
  fits (GTA SA: `CWeapon::GenerateDamageEvent` with "rammed by car"). The
  game's own reactions then follow: the victim flinches, peds run away or
  fight back, and the wanted level rises.

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
- **Draw the drone model with the game's own transforms.** In the chase,
  LOS and pilot views the drone is an object in the world. gtasa-fpv draws
  it with fixed-function triangles right after the 3D scene. It uses the
  view and projection transforms the game left on the device, so the model
  sits in the world and is hidden behind walls by the depth buffer. It
  writes RGB only: also writing alpha made the model glow through SA's
  post-processing. And it tints the model with the frame's ambient and
  directional light, so it doesn't stay bright at night.
- **The pilot's radio needs no asset.** No model in GTA SA is an RC
  transmitter. A procedural low-poly one (body, grips, gimbals, sticks,
  switches, antenna, about 17 × 11 × 5 cm), drawn the same way between the
  character's hand bones and facing its head, looks like a radio at any
  distance. Nothing extra has to ship.

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
- Back to the pilot with the drone left in the world is **F**, and F7 goes
  back to that drone ([The F key](#the-f-key-back-to-the-pilot-the-drone-left-flying)).
- **Return control only after the key is released.** F is also SA's "get
  in a vehicle" key. When the mod gave CJ his controls back in the same
  frame, the game saw F go down and he climbed into a nearby car. Hold the
  character's controls until the key is up. By then the game's previous
  and current key states agree, so it sees no press. Do this for any
  libfpv key the game also uses.
- **The Esc that closes the overlay must not reach the game.** The overlay
  closes on Esc by itself. SA opened its pause menu on that same press.
  Clear the game's Esc state while the overlay is open and until Esc is
  released.
- **Stop the game re-centring the mouse while the overlay is open.** SA
  moves the cursor to the screen centre every frame for its mouse look, so
  the overlay's cursor snapped back. Skip that call while the overlay is
  open.
- **Use your own bit to lock the player's controls.** SA's
  `CPad::DisablePlayerControls` is a bit field: scripts and cutscenes set
  their own bits. Set and clear only yours, so you never undo a script's
  lock. Watch for side effects, too. SA's replay stops on any frame in
  which player controls are disabled, so leave them alone during a
  replay.
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
  nobody at home). The player can turn the stand-in and the pose off and
  change the distances, so read them from `fpv_pilot_get_config`
  ([The player's choices](#the-players-choices-fpv_pilot_set_support)).
  [Standing in for the drone](#standing-in-for-the-drone) shows how GTA San
  Andreas does it.
- "Return the character to where the drone is" (move here): ray down from
  the drone, load the area, put the character 1 m above the ground facing
  the drone's heading, and put the game camera behind it.
- **The pilot position is the feet.** A character's position is often its
  pelvis (about 1 m up in the GTAs). Pass the feet to
  `fpv_drone_set_pilot_position`, or the LOS camera and the ear end up at
  2.6 m instead of 1.6 m.
- **Respawn where the pilot can see it.** By default, R and a respawn put
  the drone back at the launch point. After the player has used the pilot
  view (F), the character may have walked or driven away. gtasa-fpv then
  respawns the drone in front of where the character is now.
- **Flights in a replay or a frozen world.** A drone launched while a
  replay plays or a photo mod freezes the world only watches: it starts
  under the game's camera, leaves the character alone, and the flight ends
  when that scene does. Check the game's own state (its time scale, its
  user pause), not a particular mod. The game's clock stops or jumps
  there, so time anything during such a flight with real time.
- Auto-pause (default 0.5 s) covers pause menus and loading screens. Loading
  a save while flying is not handled yet.

### Standing in for the drone

**Implement the stand-in in every game with a character, on by default,
whether or not it was asked for.** It is what turns a drone flying through
a game into playing the game as the drone. For the game, the main
character *is* the drone while it stands in:

- **Enemies react to the drone.** Never make the character ignored by the
  AI (no notarget, no "invisible to enemies" flag): they spot, chase, aim
  at and shoot the drone, and the pilot dodges with standard moves.
- **Everything aims at the drone's exact position, in all three axes**:
  height too, not just the ground under it. Find the point the game aims
  at (an eye, a bone, a body target) and put it on the drone's centre.
- **Damage to the character is damage to the drone.** Turn what the game
  takes from the character into broken props (`fpv_drone_damage_props`),
  and keep the character from dying of it.
- **The world is the drone's**: streaming, LODs, culling, visibility and
  areas, triggers (doors that open for the player, pickups, checkpoints).
  If the engine decides what to load or send from the player's eye, the
  eye goes on the drone.
- **The character itself stays out of the way**: invisible, without
  collision with the drone or the world, without its own physics, so only
  its game-facing role moves with the drone.
- The player can turn it off in F8 > Fly > Pilot
  (`fpv_pilot_get_config`); then the character stays home as itself.

The rest of this section is how real integrations did it.

GTA IV moves an invisible character along with the drone so the world
streams in around it. gtasa-fpv goes further: the world treats the drone
as the player. Peds turn to look at it, police chase and shoot at it,
pickups on its path are collected, and the map loads around it, while the
pilot still stands at home. This was the mod's most reworked part. Here is
what it needed (the distances and the on/off switch come from
`fpv_pilot_get_config`):

- **If the engine asks one function where the player is, hook it.** SA's
  streaming, population and much of its AI go through `FindPlayerCoors`
  and `FindPlayerCentreOfWorld*`, which are real functions there (inlined
  in GTA IV). Detoured to return the drone, they move the streaming and
  the population with nothing else changed. This is the light stand-in.
- **Some code reads the character itself.** Pickups test the player ped's
  own position, peds' look-at IK aims at its head bone, and gunfire aims
  at its spine. For those, the character goes to the drone:
  - Make it invisible and collision-less. The game shows the player again
    by itself, so clear those bits every frame.
  - Skip its own control and physics update (no gravity, no tasks).
  - Ignore all damage to it (SA: detour the ped damage response).
- **Put the point the AI aims at on the drone, not the root.** With the
  head on the drone, police bullets went half a metre below it. With the
  spine (SA's aim bone) on the drone, they go at the drone and miss only by
  the shooter's own aim. Measure the bone's offset from the root every
  time. Until the bones are in place (the offset is over 1.5 m), use the
  root.
- **Move it without the engine's teleport.** SA's `CPed::Teleport` clears
  the ped's tasks, which knocked it out of its pose on every call. Set the
  matrix position instead, and take it out of the world's sectors and put
  it back in only every 2 m.
- **Load the collision, not just the scene.** SA's `CStreaming::LoadScene`
  loads models but not collision, and collision streams around the
  player's position, which is now the drone. When the character comes
  home, load the collision there (`CColStore::EnsureCollisionIsInMemory`)
  before putting it down. Otherwise it falls through the ground.
- **Draw the pilot at home while the character is at the drone.** For the
  frame's render passes, move the character home and make it visible, then
  move it back to the drone for the world's update. (Super Mario 64 does
  the same for Mario's own update and camera.) SA swaps it home before the
  render list is built and back after the effects pass. It stays filed in
  the drone's world sector, which is always scanned for drawing. Watch out
  for what runs between those two points:
  - Bone matrices are in world space and are rebuilt only when the
    character is drawn. Rebuild them after each swap. Otherwise AI code
    reads the home bones, and the character flickered at a distance on
    frames where its animations were skipped.
  - The renderer decides visibility at home. If it judged the character
    off screen, SA updated only the root of the skeleton. Force full
    animation updates.
  - SA's peds aim and fire between the render passes
    (`ProcessPedsAfterPreRender`). Every cop faced the drone but fired at
    the pilot's home spot. Move the character back to the drone around
    that call.
- **The pose has to work with the engine's animations.** A borrowed
  animation that doesn't move every bone leaves the others at identity:
  CJ's jaw twisted. Keep those bones as the idle animation left them.
  Hold your own reference on a streamed animation block. Otherwise the
  streaming frees it after a death or a long trip, and the next launch
  reads freed memory. To end the pose, blend back to the character's
  idle animation. SA has no animation of anyone holding an RC transmitter,
  but a punter reading a betting slip (`otb.ifp`, `betslp_loop`: head
  down, both hands at the waist) looks like a pilot.
- **Flying from a car: get out, stand in on foot, draw the character
  seated.** With only the "where is the player" functions moved, a
  driver didn't pick up pickups, and peds didn't look at the drone. So the
  driver gets out at once, as the game's warp-out-of-car script command
  does, and stands in as if on foot. At the end, it is warped back behind
  the wheel, and also before "move here", so the car comes along. At home
  it is drawn in its seat: its matrix relative to the car and the
  animation it sat with were saved when it got out. Use only the light
  stand-in when the character is on a mission (scripts check that the
  player stays in the car) or a passenger (there is no seat to warp it
  back into). The car needs care:
  - Keep the car radio playing. SA turns it off in the frame the player
    stops driving.
  - Keep the car from being deleted. The population removes cars far from
    the player, even the one it was driving. Mark the car as a mission
    car until the stand-in ends, then restore its old status.
  - Hold the car in place once the drone is far away (100 m), because the
    collision under it streams out and it would sink. Closer than that,
    leave it free, so a truck can still push it.
  - Set the car down after a hold. An empty car isn't simulated: it hangs
    in place with its wheels marked off the ground. When the hold ended,
    its first physics step threw it off a roof, onto its side or onto its
    roof. Placing it on its wheels (`PlaceOnRoadProperly`), then pinning
    it there with zero speed for 500 ms of game time while the wheels find
    the ground, fixed it every time.
  - Right after the warp back in, the ped's position catches up with the
    car only on the next frame. Measure anything from the car instead (a
    respawn in front of the pilot, for example).
- **The price:** with the stand-in on, the population follows the drone,
  so the pilot's surroundings empty out ([Audio](#audio)). Players who
  want a living world around the pilot turn the stand-in off.
- **Log every change of state.** Log when the stand-in starts and ends,
  and why (on a mission, a passenger, didn't get out); when the car is
  held, released and set down (with its up vector); and the first hits on
  the stand-in that were ignored. Most of the bugs above showed up in
  those lines first.

### The game's weapons against the drone

- **The drone isn't in the game's collision, so bullets fly through it.**
  Hook the game's per-bullet function and test each shot's path against
  the drone. In SA, `CWeapon::DoBulletImpact` receives every instant-hit
  bullet and shotgun pellet, from the muzzle to where it stopped, even
  when it hits nothing. A shot through a body sphere of about 6 cm breaks
  all four props. A shot through a prop's
  disc breaks that prop. A shot stops at whatever it hit, so a bullet into
  a wall never reaches a drone behind it.
- **Turn shots aimed at the stand-in onto the drone.** The AI aims at the
  stand-in's bone, which is near the drone but not on it. Rotate any
  non-player shot that passed within 1.5 m of that bone about its muzzle
  onto the drone, so it misses the drone by as much as it missed the
  bone. SA gives a helicopter gunner's shots the helicopter as their
  owner.
- What else damages the props in SA: water above a prop (the water level
  at its position), fires from the game's fire manager (on the ground, or
  following a burning car), a new explosion within its radius, and a
  turning helicopter rotor's disc.
- To debug the AI's aim, log each shot that passes within 2 m of the drone
  and where it passed relative to the drone. A bias (all shots below it,
  say) means the aim point is wrong.

### Audio

A `waveOut` thread (44.1 kHz, stereo, 16-bit, 4 × 512-frame buffers) that
pulls `fpv_audio_read` is enough, and the sound plays through the game's
output device. Stop and join it before deleting the drone.

- The sound is stereo: configure it with `fpv_audio_configure(rate, 2)` and
  `fpv_audio_read` writes interleaved left/right floats. Heard from the
  pilot, the drone is panned to its side, the far ear hears it a little
  later and duller, and behind the ear it is a little quieter and duller.
  With 1 channel you get a mono mix (no sides); with more than 2, channels
  3 and up are silent. The pilot's head (no listener set) turns to follow
  the drone a little behind it, so a hovering drone is centred and a fast
  pass is heard crossing.
- Call `fpv_set_view` whenever the view changes. The player chooses where
  the drone is heard from (F8 > Camera & sound): Auto (the default) hears it
  onboard in the FPV and chase views and from the pilot in the LOS and pilot
  views, where it is quieter far away and behind walls (through the same
  collision as the drone: static surfaces, objects and the raycast
  callback), with Doppler, the ground's reflection and echoes. Without the
  call every view sounds onboard.
- In the pilot view (F), pass the game camera with `fpv_audio_set_listener`
  every frame, so the drone pans to where it is on screen.
- **The game's own sounds from the pilot too.** While the drone is heard
  from the pilot, `frame.flags` has `FPV_DRONE_HEARD_FROM_PILOT`. Move the
  game's own listener to the pilot then, so every sound (traffic, peds,
  gunfire, explosions near the drone) is heard from where the pilot stands,
  as for a real pilot flying something far away: at `frame.los_camera`
  (the pilot's head, facing the drone) in the FPV and chase views. In the
  LOS and pilot views the game's camera is already there. Back to the
  game's camera as soon as the flag goes. Most engines position sounds
  relative to one listener or camera; redirect that, not the render camera.
  - **If the engine has no single listener, redirect its camera reads.**
    SA positions every sound relative to `TheCamera`, and its vehicle, ped
    and weather sounds are updated from many places, so there is no single
    call to wrap. gtasa-fpv points every operand in the audio code that
    reads the camera's position or matrix (85 of them, found by
    disassembling that code) at a fake camera placed at
    `frame.los_camera`. Check every site's bytes first and patch all of
    them or none. Undo the patches when the flag goes off and when the mod
    unloads.
  - With a stand-in ([Standing in](#standing-in-for-the-drone)),
    the world's population and streaming follow the drone, so once it is far
    the pilot's surroundings empty out and get quiet (the drone and the
    world around it are heard far off, correctly). That is the price of the
    stand-in, not a reason to drop this: players who want a living world
    around the pilot turn the stand-in off (F8 > Fly > Pilot).
- There are two volumes: the motors (`fpv_audio_set_volume`, with the
  beeper switch's beeps, which come from the motors on a real quad) and
  Betaflight's other beeps (`fpv_audio_set_beep_volume`). Players set both
  in the overlay; a game only calls them to follow its own volume sliders.

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
  afterwards, so a normal game start doesn't run it. A hot reload of the
  mod's core can run it again too.
- Many games pause while their window doesn't have focus (GTA SA does).
  Bring the window to the front before a test, or the test only starts
  when someone clicks the game.
- Make test targets stay put. Traffic that follows the road network moves
  itself back onto the road, so the autotest marks its target car as
  abandoned. When the test needs a car that isn't there, it spawns one.

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

### Case study: Portal 2

- Chell is the stand-in beyond 2 m (chambers are small; doors open a few
  metres before she reaches them). She is moved with the drone every
  server tick (VScript `SetOrigin`), noclip (`m_MoveType`), not drawn
  (`DisableDraw`), feet 12 units above the drone so a landed drone doesn't
  press floor buttons.
- Her eye is the drone's centre: her view offset points down from her feet
  to it. Writing `m_vecViewOffset` alone didn't last (the player's movement
  resets it every tick from the game's view vectors), so the view vectors'
  eye height is patched while she stands in. Check that what you write
  survives a frame before trusting it. Source aims at a player between 70 % and 100 % of the way from
  the feet to the eye (`CBasePlayer::BodyTarget`), so turrets shoot within
  4 units of the drone in X, Y and Z, and the server's PVS (what it sends
  the client) is the drone's. With the eye 76 units above the drone it sat
  inside ceilings, and objects near the drone flickered out.
- Her health is held at 100000 while she stands in; what she loses each
  tick breaks the drone's props, and is topped up again.
- An early version made her notarget and immune: turrets ignored the drone,
  which took the game out of it. Don't.
- Going home lands a tick later: until her position catches up, she counts
  as home, or the stand-in restarts at once with the drone as her spot.
  On a moving floor (the arrival elevator), keep the spot relative to it.

### Case study: GTA San Andreas (1.0 US)

| Hook | How gtasa-fpv does it |
|---|---|
| Loader | Modloader loads a small `.asi` from `modloader/FPV Drone/`. It owns the log and hot-reloads the core DLL. The game loop is entered by redirecting `Idle`'s call to `CGame::Process`, chained after CLEO. |
| Script access | None: calls into the exe at fixed addresses, checked against the community `gta_sa.pdb`. Every detour checks the prologue bytes first. |
| Coordinates | X east, Y north, Z up, metres (as GTA IV). |
| Camera | Written inside `CCamera::Process` at its `SetFOV` call: the camera matrix (its "right" is screen left), the near plane from frustum probes, and an FOV converted through the view window's scale. Replays and frozen worlds apply it after `CGame::Process`. |
| Collision | `CWorld::ProcessLineOfSight` with every entity type in one call, ignoring the player ped. |
| Effects | `CWeapon::GenerateDamageEvent` for peds, damage and a push for vehicles. Bullets through the drone come from a `DoBulletImpact` detour. Sounds are frontend events and `ReportCollision` (the drone as plastic). |
| Drawing | The D3D9 device from RenderWare. The OSD, overlay and fisheye go as in GTA IV. The drone model and the pilot's radio are fixed-function triangles with the game's transforms and light. |
| Audio | `waveOut`. The game's own sounds move to the pilot when the drone is heard from there, by re-pointing the audio code's camera operands. |
| Wind | The weather's wind and direction, helicopter rotors and explosions ([Wind](#case-study-gta-san-andreas)). |
| Streaming | `FindPlayerCoors` and `FindPlayerCentreOfWorld*` detoured to the drone, and CJ himself at the drone (out of his car if he was driving), drawn at home ([Standing in](#standing-in-for-the-drone)). |
