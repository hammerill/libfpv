// libfpv - global library state shared by the C API implementation.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cstdint>

#include "config/preset.h"
#include "input/input.h"
#include "libfpv.h"
#include "fpv_math.h"
#include "sim/flight.h"
#include "sim/runner.h"
#include "world/collision.h"
#include "world/wind.h"

struct fpv_drone {
    uint32_t magic;
};

namespace fpv {

class Overlay;
class OsdRenderer;
class AudioSynth;

// Where the drone is heard from (F8 > Camera & sound). Auto: onboard in the
// FPV and chase views, from the pilot in the LOS and pilot views.
enum class HearFrom { Auto = 0, Drone = 1, Pilot = 2 };

struct Settings {
    std::string preset;          // file name of the selected preset
    int overlayToggleKey = FPV_KEY_F8;
    float audioVolume = 0.1f;  // the motors, with the beeper switch's beeps (DShot beacon)
    float beepVolume = 0.1f;   // Betaflight's other beeps
    HearFrom hearFrom = HearFrom::Auto;
    int osdStyle = FPV_OSD_STYLE_ANALOG;
    // camera overrides (< 0 = use the preset's value)
    float cameraUptilt = -1, cameraFov = -1;
    float cameraFisheye = 0.5f;  // lens strength 0..1, 0 = off (the preset's own k1/k2)
    bool chaseSmoothing = true;
    KeyboardHover keyboardHover = KeyboardHover::Auto;  // automatic hover throttle, see InputManager
    float keyboardThrottleExpo = kDefaultKeyboardThrottleExpo;  // 1/s, see InputManager
    float keyboardYawExpo = kDefaultKeyboardYawExpo;            // 1/s, see InputManager
    bool windEnabled = true;  // the game's wind, sources and blasts move the air
    float windStrength = 1.0f;  // scales the weather's wind, 0..2
    bool batteryDrain = true;   // off: FPV_SIM_NO_BATTERY_DRAIN
    bool damage = true;         // off: FPV_SIM_NO_DAMAGE
    // the pilot character (fpv_pilot_*): the player's choices, applied
    // only where the game supports them; distances in metres, < 0 = the game's
    bool pilotStandIn = true, pilotPose = true;
    float standInDistance = -1, standInReturnDistance = -1;
    // the player's keys for game controls (fpv_controls_set), by name; 0 = no key
    std::map<std::string, int> controlKeys;
#if defined(LIBFPV_KAMIKAZE)
    // fpv_kamikaze_config, internal units (seconds, m/s, kg, metres)
    float kamikazeArmingDelay = 0.0f, kamikazeMinImpactSpeed = 0.25f;
    float kamikazePayloadKg = -1.0f, kamikazePayloadDrop = -1.0f;  // < 0 = automatic
    float kamikazeEndingDuration = 3.0f;  // the game's ending view after a detonation, s
    // the player's ending view / then (fpv_kamikaze_ending_*), -1 = the host's default
    int kamikazeEndingView = -1, kamikazeEndingThen = -1;
    bool kamikazeOsd = true;
    // file names of the quads (presets) that carry the bomb
    // (fpv_kamikaze_set_installed, overlay); none by default
    std::vector<std::string> kamikazeInstalled;
#endif
};

struct Lib {
    std::recursive_mutex mutex;  // serialises the public API
    bool initialized = false;
    bool everInitialized = false;
    fpv_global_config config{};
    std::string dataDir, presetDir, eepromDir;
    CoordConverter conv;

    CollisionWorld world;
    Flight flight;
    InputManager input;
    Runner runner{flight, input};

    std::vector<DronePreset> presets;
    std::string defaultDiff;  // fpv_set_default_betaflight_diff, applied to new EEPROMs
    int presetIndex = 0;
    Settings settings;

    fpv_drone drone{0};
    Vec3 spawnPos;  // internal frame, for "Respawn"
    Quat spawnRot;
    bool droneExists = false;

    fpv_raycast_fn raycast = nullptr;
    void *raycastUser = nullptr;

    // wind from the game (fpv_wind_*), internal units; probed and handed to
    // the simulation in fpv_drone_tick
    bool windSet = false;
    WindAmbient windAmbient;
    std::vector<WindSource> windSources;
    WindProbe windProbe;

    // the pilot character (fpv_pilot_set_support): what the game can do and
    // its default stand-in distances, metres
    uint32_t pilotFeatures = 0;
    float pilotStandInDefault = 20, pilotReturnDefault = 10;

    // the view the game shows (fpv_set_view) and the ear for hearing the
    // drone from the pilot (fpv_audio_set_listener), internal frame
    int view = FPV_VIEW_FPV;
    bool listenerSet = false;
    Vec3 listenerPos;
    Quat listenerRot;
    // walls between the ear and the drone, and the echoes around the ear
    // (host thread, a few times a second)
    struct HearingProbe {
        float timer = 0;
        float walls = 0;           // smoothed obstacle count
        float enclosure = 0;       // 0 open .. 1 a closed room
        float reflectors = 0;      // share of the directions with a wall near
        float echoDistance = 0;    // mean distance to those walls, m
        float roomSize = 0;        // mean distance to all surfaces probed, m
        bool valid = false;
    } hearingProbe;

    // FPV_DRONE_RADIO_ACTIVE: channels at the last movement, and its time
    float radioRest[FPV_CHANNEL_COUNT] = {};
    double radioMovedAt = -1e9;

    // presentation state (host thread)
    Vec3 chasePos;
    bool chaseValid = false;
    std::vector<ImpactEvent> impactScratch;

    // linked portals (fpv_portals_set), internal frame
    std::vector<PortalLink> portals;
    uint32_t seenTeleports = 0;  // the snapshot's count at the last tick

    // host buttons in the Fly tab (fpv_overlay_set_action)
    struct HostAction {
        uint32_t id;
        std::string label;
    };
    std::vector<HostAction> hostActions;
    std::vector<uint32_t> clickedActions;  // oldest first

    // created lazily, never destroyed (see lib())
    Overlay *overlay = nullptr;
    OsdRenderer *osd = nullptr;
    AudioSynth *audio = nullptr;

#if defined(LIBFPV_KAMIKAZE)
    // the endings the host can do and its defaults (fpv_kamikaze_set_ending_support)
    uint32_t kamikazeViewMask = 0x3, kamikazeThenMask = 0x7;
    uint32_t kamikazeDefaultView = 0, kamikazeDefaultThen = 0;  // SIGNAL_LOST, RESPAWN
    // warhead OSD (host thread): last composed OSD and its revision
    uint8_t kamikazeOsd[FPV_OSD_ROWS * FPV_OSD_COLS] = {};
    uint32_t kamikazeOsdRevision = 0;
#endif

    // last frame for the overlay/audio (host thread)
    fpv_frame lastFrame{};
    bool lastFrameValid = false;
};

Lib &lib();

// Errors
fpv_result setError(fpv_result r, const char *fmt, ...);
void clearError();

// Settings persistence (data dir/settings.json)
void loadSettings();
void saveSettings();

// Preset handling, called with the API mutex held.
const DronePreset &currentPreset();
fpv_result selectPreset(const std::string &nameOrFile, bool applyToFlight);
std::string eepromPathFor(const DronePreset &p);
// Erases the current quad's Betaflight EEPROM and restores its shipped preset
// files, so it flies on libfpv's own configuration again. Disarm first.
fpv_result resetCurrentPreset();
// Puts every libfpv setting back as on first launch: settings, radio
// mappings, every quad's EEPROM and the shipped preset files (players' own
// presets are kept). Disarm first.
fpv_result resetEverything();

#if defined(LIBFPV_KAMIKAZE)
// Pushes the kamikaze settings to the warhead and the flight model, the
// bomb as the current quad has it.
void applyKamikazeSettings();
// The current quad carries the bomb (Settings::kamikazeInstalled).
bool kamikazeInstalledOnCurrent();
// The ending view / then in effect: the player's choice if the host
// supports it, else the host's default (fpv_kamikaze_ending_*).
uint32_t kamikazeEndingView();
uint32_t kamikazeEndingThen();
#endif

constexpr float kPilotEarHeight = 1.6f;  // m above the pilot's feet: the LOS eye and the ear

// Hearing the drone (frame.cpp, api_present.cpp): from the pilot (the
// player's setting and the view), the ear's position, and whether the host
// pulls any sound at all (no probes otherwise).
bool hearsFromPilot();
Vec3 earPosition();
bool audioPulled();

// Pushes the simulation options (battery drain, damage) to the flight model.
void applySimOptions();

// Runs a CLI script on the simulation thread; returns BFB_CLI_* code.
int runCli(const std::string &script, std::string &output);

}  // namespace fpv
