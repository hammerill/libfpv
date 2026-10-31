/*
 * libfpv - drop-in Betaflight-flown FPV quadcopter for any game.
 *
 * Copyright (C) 2026 libfpv contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * libfpv embeds Betaflight (via SimITL) and is licensed under the GPL-3.0.
 * Any mod or program that links libfpv must be distributed under GPL-3.0
 * compatible terms.
 *
 * Kamikaze builds (CMake -DLIBFPV_KAMIKAZE=ON, which defines LIBFPV_KAMIKAZE
 * for libfpv and for every target linking libfpv::fpv) add a warhead: bomb
 * arm and explode switches, detonation on impact and a warhead OSD. See the
 * "Kamikaze" section below and docs/INTEGRATION.md#kamikaze-builds.
 *
 * ---------------------------------------------------------------------------
 * Integration in five calls (see docs/INTEGRATION.md for the full guide):
 *
 *   fpv_global_config cfg;
 *   fpv_global_config_defaults(&cfg);
 *   cfg.data_dir = "<writable folder>";
 *   cfg.coords   = fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1.0f);
 *   fpv_global_init(&cfg);                               // once
 *   fpv_static_surfaces_load(&level_mesh, 1);             // world collision
 *   fpv_drone* d; fpv_drone_create(NULL, &d);              // spawn
 *   every frame:  fpv_drone_tick(d, frame_dt, &frame);     // pose, camera, OSD
 *                 place the game camera at frame.camera, draw the drone at
 *                 frame.pose, draw fpv_osd_draw_data() and fpv_overlay_render()
 *   fpv_global_terminate();                                // on unload / exit
 *
 * Threading: every function is thread-safe (serialised internally) unless
 * stated otherwise. libfpv runs its own simulation thread, so flight and
 * radio input keep their own rate whatever the game's frame rate.
 * Web builds (Emscripten) have no thread: the same 1 kHz loop runs inside
 * fpv_drone_tick() and fpv_overlay_render(), so call them every frame from
 * the browser's main thread (docs/WEB.md).
 * Callbacks are always invoked on the thread that called into libfpv
 * (never on the simulation thread) unless stated otherwise.
 *
 * Memory: every pointer returned by libfpv is owned by libfpv. Pointers in
 * returned structs stay valid until the next call of the same function,
 * except strings returned through `char**`, which must be released with
 * fpv_free().
 *
 * Structs passed in or out carry `struct_size`. Set it to sizeof(the struct)
 * (the *_defaults() helpers do this) so newer libfpv versions stay ABI
 * compatible with mods built against older headers.
 * ---------------------------------------------------------------------------
 */
#ifndef LIBFPV_H
#define LIBFPV_H

#include <stddef.h>
#include <stdint.h>

#define LIBFPV_VERSION_MAJOR 0
#define LIBFPV_VERSION_MINOR 1
#define LIBFPV_VERSION_PATCH 0
#define LIBFPV_VERSION_STRING "0.1.0"
/* Bumped on any incompatible ABI change. Compare with fpv_abi_version(). */
#define LIBFPV_ABI_VERSION 1

#if defined(_WIN32)
#  if defined(LIBFPV_BUILD)
#    define FPV_API __declspec(dllexport)
#  elif defined(LIBFPV_STATIC)
#    define FPV_API
#  else
#    define FPV_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define FPV_API __attribute__((visibility("default")))
#else
#  define FPV_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * Versions, results, memory
 * ======================================================================== */

typedef enum fpv_result {
    FPV_OK = 0,
    FPV_ERROR_INVALID_ARGUMENT = -1,
    FPV_ERROR_NOT_INITIALIZED = -2,
    FPV_ERROR_ALREADY_INITIALIZED = -3,
    /* Only one drone can exist per process (Betaflight is a singleton). */
    FPV_ERROR_DRONE_EXISTS = -4,
    FPV_ERROR_NOT_FOUND = -5,
    FPV_ERROR_IO = -6,
    /* The Betaflight CLI is in use by the Configurator; retry later. */
    FPV_ERROR_BUSY = -7,
    FPV_ERROR_ABI_MISMATCH = -8,
    FPV_ERROR_PARSE = -9,
    FPV_ERROR_TIMEOUT = -10,
    FPV_ERROR_INTERNAL = -11
} fpv_result;

FPV_API uint32_t fpv_abi_version(void);
FPV_API const char *fpv_version_string(void);
/* Name of a result code, e.g. "FPV_ERROR_DRONE_EXISTS". */
FPV_API const char *fpv_result_string(fpv_result result);
/* Human readable detail of the last error raised on the calling thread. */
FPV_API const char *fpv_last_error(void);
/* Releases memory returned through `char **` out parameters. */
FPV_API void fpv_free(void *ptr);

/* ===========================================================================
 * Host world conventions
 *
 * libfpv works in metres, Y up, X right, Z forward internally. The host
 * describes its own world by naming which host axis points right, up and
 * forward, and how many host units make a metre. Every position, direction
 * and rotation crossing the API is in host conventions.
 *
 * Rotations are quaternions {x, y, z, w}. An object's rotation maps its
 * local axes (host right/up/forward) to world axes. So the camera rotation
 * turns the host's "forward" axis to where the camera looks. For an OpenGL
 * style host (forward = -Z) that is exactly the camera's world rotation.
 * ======================================================================== */

typedef enum fpv_axis {
    FPV_AXIS_POS_X = 0,
    FPV_AXIS_NEG_X = 1,
    FPV_AXIS_POS_Y = 2,
    FPV_AXIS_NEG_Y = 3,
    FPV_AXIS_POS_Z = 4,
    FPV_AXIS_NEG_Z = 5
} fpv_axis;

typedef struct fpv_coord_system {
    fpv_axis right;
    fpv_axis up;
    fpv_axis forward;
    float units_per_meter; /* e.g. 1 (Unity, Godot), 100 (Unreal), 39.37 (Source) */
} fpv_coord_system;

typedef enum fpv_coords_preset_id {
    /* X right, Y up, -Z forward: OpenGL, Godot, three.js, Blender glTF. */
    FPV_COORDS_Y_UP_RIGHT_HANDED = 0,
    /* X right, Y up, +Z forward: Unity, Direct3D samples, SimITL. */
    FPV_COORDS_Y_UP_LEFT_HANDED = 1,
    /* X forward, Y right, Z up: Unreal Engine. */
    FPV_COORDS_Z_UP_LEFT_HANDED = 2,
    /* X forward, Y left, Z up: Source, ROS, many id Tech derived engines. */
    FPV_COORDS_Z_UP_RIGHT_HANDED = 3
} fpv_coords_preset_id;

FPV_API fpv_coord_system fpv_coords_preset(fpv_coords_preset_id preset, float units_per_meter);

/* ===========================================================================
 * Global init
 * ======================================================================== */

typedef enum fpv_log_level {
    FPV_LOG_DEBUG = 0,
    FPV_LOG_INFO = 1,
    FPV_LOG_WARN = 2,
    FPV_LOG_ERROR = 3
} fpv_log_level;

/* May be called from libfpv's internal threads; must be thread-safe. */
typedef void (*fpv_log_fn)(void *user, fpv_log_level level, const char *message);

typedef enum fpv_timing_mode {
    /* Default. A libfpv thread steps the simulation in real time at
     * sim_rate_hz and polls the radio at the same rate. Web builds step it
     * in real time from fpv_drone_tick() and fpv_overlay_render() instead,
     * polling the radio once per frame. */
    FPV_TIMING_THREAD = 0,
    /* No thread. The host advances time explicitly with fpv_step().
     * Deterministic; used by tests, replays and lockstep engines. */
    FPV_TIMING_MANUAL = 1
} fpv_timing_mode;

enum fpv_init_flags {
    /* Do not open radios/gamepads (SDL). Use fpv_input_set_channels(). */
    FPV_INIT_NO_DEVICE_INPUT = 1u << 0,
    /* Do not start the Betaflight Configurator WebSocket server. Always set
     * in web builds: a web page cannot accept connections. */
    FPV_INIT_NO_CONFIGURATOR = 1u << 1,
    /* Keep simulating when the host stops calling fpv_drone_tick(). By default
     * libfpv auto-pauses after `auto_pause_seconds` without a tick, so a game
     * pause menu or loading screen freezes the drone too. */
    FPV_INIT_NO_AUTO_PAUSE = 1u << 2,
    /* The host does not draw the lens distortion post-process: frame.camera
     * keeps the chosen FOV with k1 = k2 = 0, and the overlay's fisheye
     * setting is shown as unavailable. */
    FPV_INIT_NO_LENS_DISTORTION = 1u << 3
};

typedef struct fpv_global_config {
    uint32_t struct_size;
    /* UTF-8 path of a writable folder for EEPROM, presets and radio
     * mappings. Created if missing. Required. Web builds: FPV_WEB_DATA_DIR
     * (or a folder inside it), which the browser keeps across visits. */
    const char *data_dir;
    fpv_coord_system coords;
    fpv_log_fn log;
    void *log_user;
    fpv_log_level log_level;
    /* WebSocket port of UART1 for Betaflight Configurator ("Manual"
     * connection to ws://localhost:<port>). 0 = 5761. */
    int configurator_port;
    fpv_timing_mode timing;
    float sim_rate_hz;         /* outer simulation rate, default 1000 */
    float auto_pause_seconds;  /* default 0.5 */
    uint32_t flags;            /* fpv_init_flags */
} fpv_global_config;

FPV_API void fpv_global_config_defaults(fpv_global_config *config);

/* A per-user folder that every libfpv game can pass as data_dir, so the
 * radio mappings, the Betaflight tune (EEPROM) and the settings follow the
 * pilot from game to game:
 *   Windows  %APPDATA%\libfpv
 *   macOS    ~/Library/Application Support/libfpv
 *   others   $XDG_CONFIG_HOME/libfpv (default ~/.config/libfpv)
 * The LIBFPV_DATA_DIR environment variable overrides it. Writes the UTF-8
 * path with its terminating zero into out. FPV_ERROR_INVALID_ARGUMENT when
 * out_size is too small, FPV_ERROR_NOT_FOUND when no home folder is known
 * (and in web builds: use FPV_WEB_DATA_DIR). Games that share the folder
 * should ship the same libfpv version, because each one upgrades the shared
 * EEPROM to its own Betaflight. Does not need fpv_global_init. */
FPV_API fpv_result fpv_shared_data_dir(char *out, uint32_t out_size);

/* Web builds: the folder kept in the browser's IndexedDB, mounted and loaded
 * before main() runs (Module.libfpvDataDir changes it). */
#define FPV_WEB_DATA_DIR "/libfpv"

/* Initialises libfpv and Betaflight. Pins the libfpv module in memory so a
 * host unloading the mod cannot unmap code that internal threads still use.
 * Calling it again after fpv_global_terminate() is allowed (Betaflight keeps
 * its state). Returns FPV_ERROR_ALREADY_INITIALIZED if already running. */
FPV_API fpv_result fpv_global_init(const fpv_global_config *config);

/* Stops the simulation thread and the Configurator server, saves pending
 * settings and releases the radio. Safe to call more than once, and safe
 * to skip: game exit with libfpv still running is handled. */
FPV_API void fpv_global_terminate(void);

FPV_API int fpv_is_initialized(void);

/* ===========================================================================
 * World collision
 *
 * Like libsm64: hand over static triangles once, and create, move and delete
 * dynamic surface objects (moving platforms, doors, gates). libfpv
 * collides the drone against them, feeds contact and ground effect to the
 * flight model, and reports impacts in fpv_frame.
 * ======================================================================== */

enum fpv_material_flags {
    /* Touching this surface destroys the propellers (water, saw, lava...). */
    FPV_MATERIAL_KILLS_PROPS = 1u << 0,
    /* Not collided with, but still occludes the video signal. */
    FPV_MATERIAL_NO_COLLISION = 1u << 1,
    /* Does not degrade the video signal (glass, foliage, nets). */
    FPV_MATERIAL_TRANSPARENT_TO_VIDEO = 1u << 2
};

typedef struct fpv_material {
    float restitution; /* bounce 0..1, default 0.2 */
    float friction;    /* Coulomb coefficient, default 0.6 */
    uint32_t flags;    /* fpv_material_flags */
} fpv_material;

/* 256 material slots. Slot 0 is the default for unspecified triangles. */
FPV_API fpv_result fpv_material_set(uint8_t material_id, const fpv_material *material);

typedef struct fpv_mesh {
    const float *positions;      /* xyz per vertex, host units */
    uint32_t vertex_count;
    uint32_t vertex_stride;      /* bytes between vertices, 0 = 12 */
    const uint32_t *indices;     /* 3 per triangle; NULL = non-indexed */
    uint32_t triangle_count;
    const uint8_t *materials;    /* per-triangle material id, or NULL */
    uint8_t material;            /* used when `materials` is NULL */
} fpv_mesh;

typedef struct fpv_transform {
    float position[3];
    float rotation[4];           /* quaternion x, y, z, w */
    float scale[3];              /* 0 entries are treated as 1 */
} fpv_transform;

/* Replaces all static geometry. Data is copied. Building takes about 1 s
 * per million triangles; keep collision meshes simplified (see the guide). */
FPV_API fpv_result fpv_static_surfaces_load(const fpv_mesh *meshes, uint32_t mesh_count);
FPV_API fpv_result fpv_static_surfaces_clear(void);

/* Dynamic objects. Geometry is in object space; moves are interpolated over
 * the simulation steps until the next move so fast platforms stay solid,
 * and the object's velocity carries a landed drone along. */
FPV_API fpv_result fpv_surface_object_create(const fpv_mesh *mesh, const fpv_transform *transform,
                                             uint32_t *out_object_id);
FPV_API fpv_result fpv_surface_object_move(uint32_t object_id, const fpv_transform *transform);
FPV_API fpv_result fpv_surface_object_delete(uint32_t object_id);

/* Optional, for worlds too large or too dynamic to export as triangles.
 * Called on the host thread inside fpv_drone_tick() (never on the
 * simulation thread). Each hit becomes a temporary collision plane around
 * the drone until the next tick. Return non-zero on hit. */
typedef struct fpv_raycast_hit {
    float distance;    /* host units along the ray */
    float normal[3];   /* host coords, facing the ray origin */
    uint8_t material;
    uint32_t object_id;
} fpv_raycast_hit;
typedef int (*fpv_raycast_fn)(void *user, const float origin[3], const float direction[3],
                              float max_distance, fpv_raycast_hit *out_hit);
FPV_API void fpv_world_set_raycast_callback(fpv_raycast_fn callback, void *user);

/* ===========================================================================
 * Drone
 * ======================================================================== */

typedef struct fpv_drone fpv_drone;

typedef struct fpv_drone_desc {
    uint32_t struct_size;
    /* Preset name ("PropEater 5") or file name in <data_dir>/presets/quad,
     * NULL = the player's last choice or the built-in default. */
    const char *preset;
    float position[3];  /* host coords; bottom of the frame rests here */
    float rotation[4];  /* quaternion x, y, z, w; zero quaternion = identity */
    /* Where the pilot stands (LOS camera, video range). Zero vector = spawn. */
    float pilot_position[3];
} fpv_drone_desc;

FPV_API void fpv_drone_desc_defaults(fpv_drone_desc *desc);

/* Creates the drone. NULL desc = defaults at the origin. Only one drone can
 * exist: a second call returns FPV_ERROR_DRONE_EXISTS. */
FPV_API fpv_result fpv_drone_create(const fpv_drone_desc *desc, fpv_drone **out_drone);
/* Removes the drone (Betaflight keeps running so the Configurator and the
 * overlay keep working). A new drone can be created afterwards. */
FPV_API fpv_result fpv_drone_delete(fpv_drone *drone);

/* Teleports to a pose with zero velocity. The drone stays armed only if the
 * player's arm switch is still on. NULL rotation keeps the current yaw upright. */
FPV_API fpv_result fpv_drone_reset(fpv_drone *drone, const float position[3], const float rotation[4]);
/* Fresh battery, cool undamaged motors and new propellers. */
FPV_API fpv_result fpv_drone_repair(fpv_drone *drone);
/* Switches the physical preset (frame, motors, props, battery, camera). */
FPV_API fpv_result fpv_drone_set_preset(fpv_drone *drone, const char *preset);

typedef struct fpv_camera {
    float position[3];
    float rotation[4];      /* local forward = view direction, local up = screen up */
    float forward[3];       /* unit vectors, host coords */
    float up[3];
    float right[3];
    float fov_y_degrees;    /* vertical field of view */
    float aspect_hint;      /* 4:3 analog, 16:9 HD; the host may ignore it */
    float near_plane;       /* host units */
    /* Brown-Conrady radial distortion (the fisheye lens) for the optional
     * post-process, see docs/INTEGRATION.md#lens-distortion. For each output
     * pixel, with c its offset from the image centre scaled so |c| = 1 at the
     * corners and r2 = dot(c, c), sample the rendered scene at
     * c * (1 + k1 * r2 + k2 * r2 * r2) / zoom. k1 = k2 = 0 = no distortion.
     * When the player enables the fisheye (F8 > Camera, on by default)
     * fov_y_degrees is already widened to make up for the zoom, unless the
     * host passed FPV_INIT_NO_LENS_DISTORTION. */
    float distortion_k1;
    float distortion_k2;
    float distortion_zoom;  /* scale applied after distortion, 1 = none */
} fpv_camera;

enum fpv_motor_flags {
    FPV_MOTOR_BURNED_OUT = 1u << 0,
    FPV_MOTOR_DAMAGED = 1u << 1,
    FPV_MOTOR_PROP_BROKEN = 1u << 2
};

typedef struct fpv_motor_state {
    float rpm;
    float thrust_newton;     /* negative in turtle mode (props spin backwards) */
    float current_amp;
    float temperature_c;
    float command;           /* ESC command 0..1 */
    float prop_damage;       /* 0 = intact, 1 = destroyed */
    float position[3];       /* motor hub in drone-local host coords */
    float spin_direction;    /* +1 clockwise, -1 counter-clockwise seen from above */
    uint32_t flags;          /* fpv_motor_flags */
} fpv_motor_state;

typedef struct fpv_battery_state {
    float voltage;
    float cell_voltage;
    float current_amp;
    float mah_drawn;
    float capacity_mah;
    uint32_t cell_count;
} fpv_battery_state;

enum fpv_drone_flags {
    FPV_DRONE_ARMED = 1u << 0,
    FPV_DRONE_FAILSAFE = 1u << 1,
    /* All props broken or a motor burned out: it will not fly until repaired. */
    FPV_DRONE_CRASHED = 1u << 2,
    FPV_DRONE_BEEPER = 1u << 3,
    FPV_DRONE_PAUSED = 1u << 4,
    FPV_DRONE_ON_GROUND = 1u << 5,
    FPV_DRONE_MODE_ANGLE = 1u << 6,
    FPV_DRONE_MODE_HORIZON = 1u << 7,
    /* Armed in turtle (crash flip) mode: the props spin backwards. */
    FPV_DRONE_TURTLE = 1u << 8,
    /* The pilot's OSD disable switch is on: Betaflight draws no OSD, and the
     * game must hide any UI it still shows over the FPV view too. See
     * docs/INTEGRATION.md#osd-disable-switch. */
    FPV_DRONE_OSD_DISABLED = 1u << 9
#if defined(LIBFPV_KAMIKAZE)
    ,
    /* Kamikaze builds: the warhead exploded. The drone is destroyed (frozen
     * where it exploded, no video signal) until fpv_drone_repair(). */
    FPV_DRONE_DETONATED = 1u << 10
#endif
};

typedef struct fpv_impact {
    float position[3];       /* host coords */
    float normal[3];         /* surface normal pointing towards the drone */
    float speed;             /* relative normal speed, host units / s */
    float impulse;           /* N*s */
    uint8_t material;
    uint32_t object_id;      /* 0 = static world or raycast */
    int32_t motor_index;     /* -1 = frame, 0..3 = a propeller hit */
} fpv_impact;

#define FPV_MAX_IMPACTS 16
#define FPV_OSD_ROWS 16
#define FPV_OSD_COLS 30
#if defined(LIBFPV_KAMIKAZE)
/* AETR + AUX1..AUX8 (AUX7 bomb arm, AUX8 explode) */
#define FPV_CHANNEL_COUNT 12
#else
/* AETR + AUX1..AUX6 */
#define FPV_CHANNEL_COUNT 10
#endif

typedef struct fpv_video_state {
    /* 0 = perfect, 1 = unwatchable. Derived from distance to the pilot,
     * obstructions between them and the transmitter type. */
    float noise;
    float breakup;           /* probability of a torn/rolling frame */
    float signal_strength;   /* 0..1, RSSI-like */
    uint32_t is_digital;     /* digital links freeze/pixelate instead */
} fpv_video_state;

typedef struct fpv_frame {
    uint32_t struct_size;
    double sim_time;          /* seconds of simulated time */
    uint32_t flags;           /* fpv_drone_flags */
    uint32_t arming_disable_flags; /* Betaflight's bitmask, 0 = armable */
    /* Why it does not arm, e.g. "THROTTLE ANGLE", empty when armable. */
    char arming_disable_text[96];

    /* Interpolated for rendering. */
    float position[3];
    float rotation[4];
    float velocity[3];         /* host units / s */
    float angular_velocity[3]; /* rad/s, host world axes */
    float speed_kmh;
    float altitude;            /* above ground directly below, host units; -1 = none */

    fpv_camera camera;         /* FPV camera */
    fpv_camera chase_camera;   /* third person suggestion */
    fpv_camera los_camera;     /* line of sight from the pilot position */

    fpv_motor_state motors[4];
    fpv_battery_state battery;
    fpv_video_state video;

    uint32_t impact_count;     /* impacts since the previous tick */
    fpv_impact impacts[FPV_MAX_IMPACTS];

    /* Betaflight OSD, row major, character codes of the Betaflight font.
     * osd_revision changes whenever any character changes. */
    uint8_t osd[FPV_OSD_ROWS * FPV_OSD_COLS];
    uint32_t osd_revision;

    /* Radio channels as Betaflight sees them, AETR + AUX1..6, -1..1:
     * AUX1 arm, AUX2 angle, AUX3 beeper, AUX4 turtle, AUX5 air mode,
     * AUX6 OSD disable; kamikaze builds add AUX7 bomb arm and AUX8
     * explode. */
    float channels[FPV_CHANNEL_COUNT];

    /* The air's velocity at the drone (wind, gusts, rotor downwash, blasts),
     * host units / s; zero in still air or with wind off. */
    float wind[3];
} fpv_frame;

/* Advances presentation by frame_dt seconds of host time and fills `frame`.
 * In FPV_TIMING_MANUAL mode it does not advance the simulation (use
 * fpv_step). Call once per rendered frame. `frame->struct_size` must be set. */
FPV_API fpv_result fpv_drone_tick(fpv_drone *drone, float frame_dt, fpv_frame *frame);

/* ===========================================================================
 * GPS
 *
 * The drone carries a simulated GPS, so Betaflight's GPS features work:
 * speed, home arrow and distance, distance flown, altitude, vario,
 * efficiency, their post-flight statistics, the GPS lap timer and GPS
 * Rescue. North is
 * the host's forward axis and east its right axis (fpv_coord_system), for
 * the GPS and for the heading and compass elements alike.
 * See docs/INTEGRATION.md#gps-and-the-home-point.
 * ======================================================================== */

/* Sets the home point (host coords) that the home arrow, home distance and
 * GPS Rescue use, for every flight until changed. It replaces the point
 * Betaflight would set when the drone arms. NULL hands home back to
 * Betaflight: like a real quad, home is then where the drone last armed
 * (the default). Cheap; call it every frame if home moves. */
FPV_API fpv_result fpv_gps_set_home(const float position[3]);

/* ===========================================================================
 * Wind
 *
 * The air moves and the drone flies in it: its drag, rotor drag, prop
 * inflow and prop wash work on the airspeed, so Betaflight has to hold the
 * quad against the wind and its gusts like on a real one. The game gives
 * the weather's wind; libfpv adds what changes it around the drone: more
 * wind higher up, gusts, the lee of buildings and walls (found with the
 * same collision as the drone: static surfaces, objects and the raycast
 * callback), updrafts in front of walls, and calm air indoors. The player
 * turns it off or scales it in the overlay (F8 > Fly > Wind); it is on by
 * default. See docs/INTEGRATION.md#wind.
 * ======================================================================== */

enum fpv_wind_flags {
    /* Skip libfpv's obstacle probes (the wind is the same everywhere). */
    FPV_WIND_NO_SHELTER = 1u << 0,
    /* The drone is somewhere no weather reaches (an interior): only the
     * sources and blasts below move the air. */
    FPV_WIND_ENCLOSED = 1u << 1
};

typedef struct fpv_wind {
    uint32_t struct_size;
    /* The weather's mean wind, host units / s: the way the air moves (a
     * north wind points south), in the open at reference_height above the
     * ground. Give it horizontal; libfpv adds the vertical air motion. */
    float velocity[3];
    /* Host units above the ground; 0 = 10 m, the weather stations' height. */
    float reference_height;
    /* Gust strength as the standard deviation over the mean, 0..1 (about
     * 0.15 open country, 0.3 a city street); < 0 = automatic from the
     * height above the ground (the default). */
    float turbulence;
    uint32_t flags; /* fpv_wind_flags */
} fpv_wind;

FPV_API void fpv_wind_defaults(fpv_wind *wind);
/* Sets the weather's wind. Cheap: call it every frame as the weather
 * changes. NULL = still air (the default). */
FPV_API fpv_result fpv_wind_set(const fpv_wind *wind);

enum fpv_wind_source_type {
    /* A rotor (helicopter): `speed` is the induced velocity at the disc,
     * sqrt(thrust / (2 * 1.2 kg/m3 * disc area)) in host units / s, and
     * `radius` the rotor's radius. The wake contracts to twice that speed
     * below the disc and spreads out along the ground where it lands. */
    FPV_WIND_SOURCE_ROTOR = 0,
    /* A jet or fan (jet exhaust, VTOL nozzle, big fan): `speed` at the
     * outlet, `radius` the outlet's; it spreads like a turbulent jet. */
    FPV_WIND_SOURCE_JET = 1
};

typedef struct fpv_wind_source {
    float position[3];  /* host coords: rotor hub or outlet */
    float direction[3]; /* where the air is blown (a hovering rotor: down) */
    float speed;        /* host units / s, see fpv_wind_source_type */
    float radius;       /* host units */
    uint32_t type;      /* fpv_wind_source_type */
} fpv_wind_source;

#define FPV_MAX_WIND_SOURCES 32
/* Replaces the air movers near the drone (at most FPV_MAX_WIND_SOURCES,
 * the rest are ignored). Call it every frame they move or change; count 0
 * clears them. They blow whatever the weather and the shelter, but not
 * with wind turned off by the player. */
FPV_API fpv_result fpv_wind_set_sources(const fpv_wind_source *sources, uint32_t count);

/* An explosion's blast wave: a gust that runs outwards from `position` and
 * is gone within about a second. `radius` is the explosion's size in host
 * units: the gust is about 40 m/s at that distance and weakens with the
 * distance (to a fifth at three times it). Once per explosion. */
FPV_API fpv_result fpv_wind_add_blast(const float position[3], float radius);

#if defined(LIBFPV_KAMIKAZE)
/* ===========================================================================
 * Kamikaze (only in builds with LIBFPV_KAMIKAZE)
 *
 * The drone can carry a warhead with two switches: bomb arm (AUX7) and
 * explode (AUX8). The bomb is not installed by default: the drone then
 * flies like in any other build (no extra weight, no warhead OSD, the
 * bomb switches do nothing). The player installs it in the overlay
 * (F8 > Kamikaze), or the game with fpv_kamikaze_set_installed(); it stays for
 * the session, on every new, respawned or repaired drone. With it, the
 * rules, as on the real thing:
 *
 *   - The motors do not arm while the bomb arm switch is on. Arm the
 *     motors with it off, then turn it on in flight. After a refused arm,
 *     the arm switch must be cycled, like Betaflight's ARMSWITCH safety.
 *   - Bomb arm switch on with the motors armed makes the warhead live
 *     (FPV_WARHEAD_ARMED, OSD "BOMB_ARMED"), after the arming delay if one is
 *     set (none by default; FPV_WARHEAD_ARMING, OSD "ARMING 3"). Off at any
 *     time safes it ("BOMB_SAFE"). The OSD line is Betaflight's "Custom message
 *     1" element: players move or hide it in the overlay (F8 > OSD) or the
 *     Betaflight Configurator, and other elements can draw over it.
 *   - Only a live warhead explodes: on any touch (a contact fuse: every
 *     collision approaching faster than min_impact_speed, 0.25 m/s by
 *     default, so also a light tap on the ground that fpv_frame.impacts
 *     does not report; resting on the ground does not fire it), on the explode
 *     switch (off to on), or on fpv_kamikaze_detonate(). With the bomb arm switch off, nothing
 *     explodes; the explode switch only shows "NOT ARMED".
 *   - A switch that is already on when the drone spawns or is repaired
 *     must be turned off first.
 *   - After the explosion (OSD blank) the drone is destroyed: frame.flags
 *     has FPV_DRONE_DETONATED and FPV_DRONE_CRASHED, the wreck stays where
 *     it exploded, the motors stop, frame.video reports no signal (noise 1,
 *     signal_strength 0), and it does not arm. fpv_drone_repair() gives a
 *     new drone with a new, safe warhead (then fpv_drone_reset() to move it).
 *
 * The game reacts to fpv_kamikaze_poll_detonation(): play the explosion
 * at its position, damage what is around, then show the ending the player
 * chose (fpv_kamikaze_config: ending_view for ending_duration seconds, then
 * ending_then), timed with the game's own clock. While the view shows, the
 * player's own respawn / recall / move-the-pilot commands work at once and
 * replace ending_then. See docs/INTEGRATION.md#kamikaze-builds.
 * ======================================================================== */

typedef enum fpv_warhead_state {
    FPV_WARHEAD_SAFE = 0,      /* bomb arm switch off (or not cycled yet) */
    FPV_WARHEAD_ARMING = 1,    /* switch on, arming delay running */
    FPV_WARHEAD_ARMED = 2,     /* live: explodes on impact or the explode switch */
    FPV_WARHEAD_DETONATED = 3  /* exploded; the drone is destroyed until repaired */
} fpv_warhead_state;

enum fpv_kamikaze_state_flags {
    FPV_KAMIKAZE_BOMB_SWITCH = 1u << 0,     /* bomb arm switch (AUX7) is on */
    FPV_KAMIKAZE_EXPLODE_SWITCH = 1u << 1,  /* explode switch (AUX8) is on */
    /* The motors cannot arm now: the bomb arm switch is on (or the drone is
     * destroyed). arming_disable_text also says "BOMB". */
    FPV_KAMIKAZE_BLOCKS_ARMING = 1u << 2,
    /* The bomb arm switch is on but must be turned off before it can arm. */
    FPV_KAMIKAZE_NEEDS_SWITCH_CYCLE = 1u << 3,
    FPV_KAMIKAZE_INSTALLED = 1u << 4  /* the drone carries the bomb */
};

typedef struct fpv_kamikaze_state {
    uint32_t struct_size;
    fpv_warhead_state warhead;
    float arming_remaining;     /* seconds until live, while ARMING */
    uint32_t flags;             /* fpv_kamikaze_state_flags */
    uint32_t detonation_count;  /* since fpv_global_init */
    float payload_mass;         /* kg, the warhead the drone carries */
    float total_mass;           /* kg, all-up weight with the payload */
} fpv_kamikaze_state;

/* State as of the last fpv_drone_tick(). */
FPV_API fpv_result fpv_kamikaze_get_state(fpv_drone *drone, fpv_kamikaze_state *state);

typedef enum fpv_detonation_cause {
    FPV_DETONATION_IMPACT = 0,  /* hit something */
    FPV_DETONATION_SWITCH = 1,  /* the pilot's explode switch */
    FPV_DETONATION_HOST = 2     /* fpv_kamikaze_detonate() */
} fpv_detonation_cause;

typedef struct fpv_detonation {
    uint32_t struct_size;
    double sim_time;
    fpv_detonation_cause cause;
    float position[3];   /* host coords: the impact point, or the drone */
    float velocity[3];   /* drone velocity at the moment, host units / s */
    float normal[3];     /* surface hit, towards the drone; zero unless an impact */
    uint8_t material;    /* surface hit (impacts) */
    uint32_t object_id;  /* surface object hit, 0 = static world, raycast or none */
    int32_t motor_index; /* -1 = frame, 0..3 = a propeller hit, -1 unless an impact */
} fpv_detonation;

/* Takes the oldest detonation not returned yet. Returns 1 and fills `out`
 * (struct_size set by the caller), or 0 when there is none. Call it every
 * frame; a detonation is also visible as FPV_DRONE_DETONATED. */
FPV_API int fpv_kamikaze_poll_detonation(fpv_detonation *out);

/* Installs (non-zero) or removes (0) the bomb, at once, also in flight.
 * Kept until changed or the program exits (not saved), for every drone:
 * a new or respawned one carries it too. Removing it from a destroyed
 * drone leaves the wreck until fpv_drone_repair(). The player can do the
 * same in the overlay (F8 > Kamikaze). */
FPV_API fpv_result fpv_kamikaze_set_installed(int installed);
FPV_API int fpv_kamikaze_is_installed(void);

/* The game blows the warhead up (shot down, scripted, a timer). Only a live
 * (ARMED) warhead explodes unless `even_if_safe` is non-zero (a bullet hit
 * the explosive), and never without the bomb installed.
 * FPV_ERROR_NOT_FOUND when nothing exploded. */
FPV_API fpv_result fpv_kamikaze_detonate(fpv_drone *drone, int even_if_safe);

enum fpv_kamikaze_config_flags {
    /* Do not draw the warhead state into frame.osd nor blank it after the
     * explosion; the game shows it. */
    FPV_KAMIKAZE_NO_OSD = 1u << 0
};

/* What the game shows right after the explosion (ending_view). */
typedef enum fpv_kamikaze_ending_view {
    FPV_KAMIKAZE_VIEW_SIGNAL_LOST = 0,  /* default: full static, OSD blank */
    FPV_KAMIKAZE_VIEW_CUTSCENE = 1      /* external game camera on the blast */
} fpv_kamikaze_ending_view;

/* What the game does when the view's time is up (ending_then). */
typedef enum fpv_kamikaze_ending_then {
    FPV_KAMIKAZE_THEN_RESPAWN = 0,      /* default: fpv_drone_repair + fpv_drone_reset */
    FPV_KAMIKAZE_THEN_RECALL = 1,       /* end the flight, back to the pilot */
    FPV_KAMIKAZE_THEN_NOTHING = 2       /* keep the view until the player acts (also with 0 s) */
} fpv_kamikaze_ending_then;

typedef struct fpv_kamikaze_config {
    uint32_t struct_size;
    float arming_delay;      /* seconds from bomb arm switch on to live, default 0 (none) */
    /* Contact fuse threshold: the slowest touch that sets it off, host
     * units / s, 0.1..30 m/s. Default 0.25 m/s (a light tap). Players can
     * tune it in the overlay (F8 > Kamikaze). */
    float min_impact_speed;
    uint32_t flags;          /* fpv_kamikaze_config_flags */
    /* The warhead is real weight: it adds to the quad's mass and inertia
     * (it hangs below the frame) and is the lowest collision point, so a
     * loaded drone climbs slower and turns lazier. kg; -1 (default) = the
     * preset's "payload" { "mass": kg, "drop": m }, or else 60% of the
     * quad's own mass. 0 = no extra weight. */
    float payload_mass;
    /* How far below the frame centre it hangs, host units; -1 (default) =
     * the preset's, or just under the frame. */
    float payload_drop;
    /* The ending, the player's choice in the overlay (F8 > Kamikaze): after
     * a detonation the game shows ending_view for ending_duration seconds
     * (0..30, default 3), then does ending_then. 0 s skips the view for
     * RESPAWN and RECALL (they happen at once). With
     * FPV_KAMIKAZE_THEN_NOTHING the view shows and stays until the player
     * acts, whatever the duration (also 0).
     * During the view the player's own respawn / recall / move-the-pilot
     * commands work at once and replace ending_then. libfpv never times the
     * ending itself: the game reads these (fpv_kamikaze_get_config) when
     * the warhead goes off and times it with its own clock. Leave them out
     * (a smaller struct_size) to keep the player's. */
    float ending_duration;
    uint32_t ending_view;   /* fpv_kamikaze_ending_view, default SIGNAL_LOST */
    uint32_t ending_then;   /* fpv_kamikaze_ending_then, default RESPAWN */
} fpv_kamikaze_config;

/* Fills the defaults (struct_size included): the ending is the host's
 * default (fpv_kamikaze_set_ending_support). Call it after fpv_global_init
 * so min_impact_speed is in host units. */
FPV_API void fpv_kamikaze_config_defaults(fpv_kamikaze_config *config);
/* Applies at once (the payload also in flight), for every drone and preset
 * until changed. Players can tweak the same values in the overlay (F8 > Kamikaze);
 * both are saved in <data_dir>/settings.json, so set it at startup only if
 * the game wants to override the player. */
FPV_API fpv_result fpv_kamikaze_set_config(const fpv_kamikaze_config *config);
FPV_API fpv_result fpv_kamikaze_get_config(fpv_kamikaze_config *config);

/* The host says which endings it can do and its own default "then", once at
 * startup after fpv_global_init. Masks are bit sets of (1u << FPV_KAMIKAZE_VIEW_*)
 * and (1u << FPV_KAMIKAZE_THEN_*). The overlay only offers supported choices;
 * a saved player choice the host doesn't support reads back as the host
 * default. It never overrides a supported saved choice. Not calling it =
 * everything supported, default view SIGNAL_LOST, default then RESPAWN.
 * SIGNAL_LOST and NOTHING must always be in the masks (libfpv forces them).
 * FPV_ERROR_INVALID_ARGUMENT for an unknown or unsupported default. */
FPV_API fpv_result fpv_kamikaze_set_ending_support(uint32_t view_mask,
                                                   uint32_t then_mask,
                                                   uint32_t default_view,
                                                   uint32_t default_then);
#endif /* LIBFPV_KAMIKAZE */

/* ===========================================================================
 * Time
 * ======================================================================== */

FPV_API void fpv_set_paused(int paused);
FPV_API int fpv_is_paused(void);
/* Slow motion / fast forward. 1 = real time, clamped to 0.05..4. */
FPV_API void fpv_set_time_scale(float scale);
/* FPV_TIMING_MANUAL only: advances the simulation by `seconds`, polling
 * input and stepping at sim_rate_hz. */
FPV_API fpv_result fpv_step(float seconds);

/* ===========================================================================
 * Input
 *
 * libfpv reads radios (EdgeTX/OpenTX/ELRS/Crossfire handsets in USB joystick
 * mode, simulator dongles) and gamepads itself, in the background, with no
 * window focus required (web builds: through the browser's Gamepad API,
 * which lists a device once a button or stick on it has been moved). The
 * overlay's setup wizard maps them. The host only forwards keyboard events
 * (fpv_input_key), which drive both the overlay and the keyboard fallback.
 * ======================================================================== */

typedef enum fpv_input_source {
    FPV_INPUT_SOURCE_NONE = 0,
    FPV_INPUT_SOURCE_RADIO = 1,
    FPV_INPUT_SOURCE_GAMEPAD = 2,   /* degraded: no proper throttle */
    FPV_INPUT_SOURCE_KEYBOARD = 3,  /* degraded: digital sticks */
    FPV_INPUT_SOURCE_HOST = 4       /* fpv_input_set_channels */
} fpv_input_source;

typedef struct fpv_input_status {
    uint32_t struct_size;
    fpv_input_source source;
    char device_name[128];
    int needs_setup;        /* a radio is connected but not mapped yet */
    int device_count;
} fpv_input_status;

FPV_API fpv_result fpv_input_get_status(fpv_input_status *status);

/* Host-injected channels (AETR, AUX1..AUX6, or AUX1..AUX8 in kamikaze
 * builds, -1..1, throttle -1 = idle; missing AUX channels are off).
 * While injected channels are fresher than 0.5 s they take precedence over
 * devices. Use for scripted tests, replays or a host-side input system. */
FPV_API void fpv_input_set_channels(const float *channels, uint32_t count);
FPV_API void fpv_input_clear_channels(void);

/* Keys, as forwarded by the host. Letters/digits use their ASCII uppercase
 * code ('A', '0'); the rest use these values. Letters and digits name a
 * physical key position on a US QWERTY keyboard, not the label on the key:
 * forward scancodes (SDL_Scancode, KeyboardEvent.code, Windows scan codes
 * rather than virtual-key codes), so the
 * keyboard sticks stay under the left hand on AZERTY, QWERTZ or Dvorak.
 * Typed text goes through fpv_overlay_text, which follows the layout. */
typedef enum fpv_key {
    FPV_KEY_NONE = 0,
    FPV_KEY_TAB = 9, FPV_KEY_ENTER = 13, FPV_KEY_ESCAPE = 27, FPV_KEY_SPACE = 32,
    FPV_KEY_LEFT = 256, FPV_KEY_RIGHT, FPV_KEY_UP, FPV_KEY_DOWN,
    FPV_KEY_PAGE_UP, FPV_KEY_PAGE_DOWN, FPV_KEY_HOME, FPV_KEY_END,
    FPV_KEY_INSERT, FPV_KEY_DELETE, FPV_KEY_BACKSPACE,
    FPV_KEY_LSHIFT, FPV_KEY_RSHIFT, FPV_KEY_LCTRL, FPV_KEY_RCTRL, FPV_KEY_LALT, FPV_KEY_RALT,
    FPV_KEY_F1, FPV_KEY_F2, FPV_KEY_F3, FPV_KEY_F4, FPV_KEY_F5, FPV_KEY_F6,
    FPV_KEY_F7, FPV_KEY_F8, FPV_KEY_F9, FPV_KEY_F10, FPV_KEY_F11, FPV_KEY_F12
} fpv_key;

FPV_API void fpv_input_key(int key, int down);

/* ===========================================================================
 * Overlay (Dear ImGui, rendered by the host)
 *
 * libfpv owns an ImGui context and returns backend-agnostic draw data. The
 * host draws it with its own renderer (snippets for D3D11, D3D12, OpenGL and
 * Vulkan in docs/renderers/). While the overlay is open,
 * fpv_overlay_wants_input() is non-zero and the game should ignore mouse
 * and keyboard.
 * ======================================================================== */

typedef struct fpv_draw_vertex {
    float pos[2];       /* pixels, origin top-left */
    float uv[2];
    uint32_t color;     /* RGBA8, R in the lowest byte (ImGui's IM_COL32) */
} fpv_draw_vertex;

typedef struct fpv_draw_cmd {
    float clip_rect[4];      /* x1, y1, x2, y2 in pixels */
    uint64_t texture_id;     /* FPV_TEXTURE_* */
    uint32_t index_offset;   /* into the list's index buffer */
    uint32_t index_count;
    uint32_t vertex_offset;  /* added to each index */
} fpv_draw_cmd;

typedef struct fpv_draw_list {
    const fpv_draw_vertex *vertices;
    uint32_t vertex_count;
    const uint32_t *indices;
    uint32_t index_count;
    const fpv_draw_cmd *commands;
    uint32_t command_count;
} fpv_draw_list;

typedef struct fpv_draw_data {
    float display_size[2];
    uint32_t list_count;
    const fpv_draw_list *lists;
    uint32_t total_vertex_count;
    uint32_t total_index_count;
} fpv_draw_data;

/* Texture ids that can appear in draw commands. */
#define FPV_TEXTURE_OVERLAY_FONT 1u
#define FPV_TEXTURE_OSD_FONT 2u

typedef struct fpv_texture {
    uint32_t width;
    uint32_t height;
    const uint8_t *rgba;     /* width * height * 4 bytes, straight alpha */
    uint32_t revision;       /* re-upload when it changes */
} fpv_texture;

FPV_API fpv_result fpv_texture_get(uint64_t texture_id, fpv_texture *out);

FPV_API void fpv_overlay_set_open(int open);
FPV_API int fpv_overlay_is_open(void);
FPV_API int fpv_overlay_wants_input(void);
/* Key that toggles the overlay (default FPV_KEY_F8). Players can rebind it
 * in the overlay; the host does not need to handle it. FPV_KEY_ESCAPE also
 * closes the overlay (after any open popup or text field): if Esc opens the
 * game's own pause menu, hide it from the game while fpv_overlay_is_open(). */
FPV_API void fpv_overlay_set_toggle_key(int key);

/* width/height in the coordinates mouse positions use (window points), and
 * ui_scale the OS content scale in those coordinates (1.0 on a Retina Mac,
 * 1.5 at Windows 150%), not the framebuffer/window pixel ratio. */
FPV_API void fpv_overlay_set_display(float width, float height, float ui_scale);
FPV_API void fpv_overlay_mouse_move(float x, float y);
FPV_API void fpv_overlay_mouse_button(int button, int down);   /* 0 left, 1 right, 2 middle */
FPV_API void fpv_overlay_mouse_wheel(float dx, float dy);
FPV_API void fpv_overlay_text(const char *utf8);

/* Host buttons, drawn in the overlay's Fly tab (for example "Move the player
 * here"). `id` is the host's choice (non-zero); setting a NULL or empty label
 * removes the button. Clicks are queued: fpv_overlay_poll_action() returns
 * the id of the next clicked button, or 0. Any thread. */
FPV_API void fpv_overlay_set_action(uint32_t id, const char *label);
FPV_API uint32_t fpv_overlay_poll_action(void);

/* Builds the overlay for this frame. Returns NULL when closed. Valid until
 * the next call. */
FPV_API const fpv_draw_data *fpv_overlay_render(float frame_dt);

/* ===========================================================================
 * OSD
 * ======================================================================== */

typedef enum fpv_osd_style {
    FPV_OSD_STYLE_ANALOG = 0,  /* white glyphs with black outline, slight blur */
    FPV_OSD_STYLE_HD = 1       /* crisp, larger glyphs (DJI/HDZero/Walksnail look) */
} fpv_osd_style;

/* Quads for the current OSD grid, textured with FPV_TEXTURE_OSD_FONT and
 * scaled to fill a viewport of width x height pixels at 4:3 (analog) or 16:9
 * (HD), centred. Draw it with the overlay renderer. */
FPV_API const fpv_draw_data *fpv_osd_draw_data(const fpv_frame *frame, float width, float height,
                                              fpv_osd_style style);
/* Alternatively, rasterises the OSD into an RGBA buffer (CPU). */
FPV_API fpv_result fpv_osd_render_rgba(const fpv_frame *frame, uint8_t *rgba, uint32_t width,
                                       uint32_t height, uint32_t stride_bytes, fpv_osd_style style);

/* ===========================================================================
 * Audio
 * ======================================================================== */

/* Optional built-in motor sound. Pull interleaved float samples from your
 * audio thread; returns the number of frames written (always `frames`). */
FPV_API fpv_result fpv_audio_configure(uint32_t sample_rate, uint32_t channels);
FPV_API uint32_t fpv_audio_read(float *out, uint32_t frames);
FPV_API void fpv_audio_set_volume(float volume);

/* ===========================================================================
 * Configuration
 * ======================================================================== */

/* Runs Betaflight CLI commands (newline separated) and returns the output.
 * "save" is applied without reboot. "defaults", "exit", "bl", "dfu" and
 * "msc" are refused. Returns FPV_ERROR_BUSY while the Configurator is in
 * CLI mode. */
FPV_API fpv_result fpv_cli_exec(const char *commands, char **out_output);

/* Imports a real quad's `diff all` (or `dump`): drops hardware-specific
 * lines, forces PWM and ARM on AUX1 (other modes kept on AUX2-4), applies
 * the rest and saves. `out_report` (optional) lists what was skipped and
 * why. */
FPV_API fpv_result fpv_import_betaflight_diff(const char *diff_text, char **out_report);

/* The game's own `diff all` (cleaned up like fpv_import_betaflight_diff),
 * applied on top of libfpv's setup and a preset's tune whenever libfpv
 * creates a drone's Betaflight configuration: the first flight of each
 * drone, and after the player resets it in the overlay (F8 > Drone).
 * Existing configurations are left alone. Call it before fpv_global_init
 * to cover the drone that init creates; NULL or "" clears it. */
FPV_API void fpv_set_default_betaflight_diff(const char *diff_text);

/* Physical presets (frame, motor, prop, battery, camera). */
FPV_API uint32_t fpv_preset_count(void);
FPV_API const char *fpv_preset_name(uint32_t index);
/* Directory where JSON presets are read from (copied there on first run). */
FPV_API const char *fpv_preset_directory(void);

typedef struct fpv_configurator_info {
    uint32_t struct_size;
    int enabled;
    int port;
    int client_count;
    char url[64];           /* "ws://localhost:5761" */
} fpv_configurator_info;

FPV_API fpv_result fpv_configurator_get_info(fpv_configurator_info *info);

#ifdef __cplusplus
}
#endif

#endif /* LIBFPV_H */
