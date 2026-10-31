// libfpv - C API: init/terminate, world, drone, time, input, configuration.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iterator>

#include "config/bf_config.h"
#include "core/bf_bridge.h"
#include "lib.h"
#include "nlohmann/json.hpp"
#include "sim/sim.h"
#include "util/embedded.h"
#include "util/fs.h"
#include "util/log.h"
#include "util/platform.h"
#include "web/web.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace fpv {

Lib &lib() {
    // Never destroyed: internal threads may outlive static destruction.
    static Lib *instance = new Lib();
    return *instance;
}

static thread_local std::string t_lastError;

fpv_result setError(fpv_result r, const char *fmt, ...) {
    char buf[512];
    va_list a;
    va_start(a, fmt);
    vsnprintf(buf, sizeof(buf), fmt, a);
    va_end(a);
    t_lastError = std::string(fpv_result_string(r)) + ": " + buf;
    FPV_LOGD("%s", t_lastError.c_str());
    return r;
}

void clearError() { t_lastError.clear(); }

// --- settings ------------------------------------------------------------

void loadSettings() {
    Lib &L = lib();
    std::string text;
    if (!readTextFile(pathFromUtf8(L.dataDir) / "settings.json", text)) return;
    try {
        json j = json::parse(text);
        Settings &s = L.settings;
        s.preset = j.value("preset", s.preset);
        s.overlayToggleKey = j.value("overlayToggleKey", s.overlayToggleKey);
        s.audioVolume = j.value("audioVolume", s.audioVolume);
        s.osdStyle = j.value("osdStyle", s.osdStyle);
        s.cameraUptilt = j.value("cameraUptilt", s.cameraUptilt);
        s.cameraFov = j.value("cameraFov", s.cameraFov);
        s.cameraFisheye = std::clamp(j.value("cameraFisheye", s.cameraFisheye), 0.0f, 1.0f);
        s.chaseSmoothing = j.value("chaseSmoothing", s.chaseSmoothing);
        // "auto", "on" or "off"; older versions stored a bool (off by default)
        auto hover = j.find("keyboardHover");
        if (hover != j.end() && hover->is_string()) {
            std::string v = hover->get<std::string>();
            s.keyboardHover = v == "on" ? KeyboardHover::On : v == "off" ? KeyboardHover::Off : KeyboardHover::Auto;
        } else if (hover != j.end() && hover->is_boolean() && hover->get<bool>()) {
            s.keyboardHover = KeyboardHover::On;
        }
        s.keyboardThrottleExpo = std::clamp(j.value("keyboardThrottleExpo", s.keyboardThrottleExpo), 0.5f, 10.0f);
        s.keyboardYawExpo = std::clamp(j.value("keyboardYawExpo", s.keyboardYawExpo), 0.5f, 20.0f);
        s.windEnabled = j.value("windEnabled", s.windEnabled);
        s.windStrength = std::clamp(j.value("windStrength", s.windStrength), 0.0f, 2.0f);
#if defined(LIBFPV_KAMIKAZE)
        s.kamikazeArmingDelay = std::clamp(j.value("kamikazeArmingDelay", s.kamikazeArmingDelay), 0.0f, 600.0f);
        s.kamikazeMinImpactSpeed = std::clamp(j.value("kamikazeMinImpactSpeed", s.kamikazeMinImpactSpeed), 0.1f, 30.0f);
        s.kamikazePayloadKg = j.value("kamikazePayloadKg", s.kamikazePayloadKg);
        s.kamikazePayloadDrop = j.value("kamikazePayloadDrop", s.kamikazePayloadDrop);
        s.kamikazeEndingDuration = std::clamp(j.value("kamikazeEndingDuration", s.kamikazeEndingDuration), 0.0f, 30.0f);
        // by name; absent or unknown = the host's default
        std::string view = j.value("kamikazeEndingView", std::string());
        s.kamikazeEndingView = view == "signal_lost" ? FPV_KAMIKAZE_VIEW_SIGNAL_LOST
                               : view == "cutscene"  ? FPV_KAMIKAZE_VIEW_CUTSCENE
                                                     : -1;
        std::string then = j.value("kamikazeEndingThen", std::string());
        s.kamikazeEndingThen = then == "respawn"   ? FPV_KAMIKAZE_THEN_RESPAWN
                               : then == "recall"  ? FPV_KAMIKAZE_THEN_RECALL
                               : then == "nothing" ? FPV_KAMIKAZE_THEN_NOTHING
                                                   : -1;
        s.kamikazeOsd = j.value("kamikazeOsd", s.kamikazeOsd);
#endif
    } catch (const std::exception &e) {
        FPV_LOGW("settings.json ignored: %s", e.what());
    }
}

void saveSettings() {
    Lib &L = lib();
    const Settings &s = L.settings;
    json j = {{"version", 1},
              {"preset", s.preset},
              {"overlayToggleKey", s.overlayToggleKey},
              {"audioVolume", s.audioVolume},
              {"osdStyle", s.osdStyle},
              {"cameraUptilt", s.cameraUptilt},
              {"cameraFov", s.cameraFov},
              {"cameraFisheye", s.cameraFisheye},
              {"chaseSmoothing", s.chaseSmoothing},
              {"keyboardHover", s.keyboardHover == KeyboardHover::On    ? "on"
                                : s.keyboardHover == KeyboardHover::Off ? "off"
                                                                        : "auto"},
              {"keyboardThrottleExpo", s.keyboardThrottleExpo},
              {"keyboardYawExpo", s.keyboardYawExpo},
              {"windEnabled", s.windEnabled},
              {"windStrength", s.windStrength}};
#if defined(LIBFPV_KAMIKAZE)
    j["kamikazeArmingDelay"] = s.kamikazeArmingDelay;
    j["kamikazeMinImpactSpeed"] = s.kamikazeMinImpactSpeed;
    j["kamikazePayloadKg"] = s.kamikazePayloadKg;
    j["kamikazePayloadDrop"] = s.kamikazePayloadDrop;
    j["kamikazeEndingDuration"] = s.kamikazeEndingDuration;
    static const char *const views[] = {"signal_lost", "cutscene"};
    static const char *const thens[] = {"respawn", "recall", "nothing"};
    if (s.kamikazeEndingView >= 0 && s.kamikazeEndingView < 2) j["kamikazeEndingView"] = views[s.kamikazeEndingView];
    if (s.kamikazeEndingThen >= 0 && s.kamikazeEndingThen < 3) j["kamikazeEndingThen"] = thens[s.kamikazeEndingThen];
    j["kamikazeOsd"] = s.kamikazeOsd;
#endif
    writeFileAtomic(pathFromUtf8(L.dataDir) / "settings.json", j.dump(2));
}

// --- presets ---------------------------------------------------------------

const DronePreset &currentPreset() {
    Lib &L = lib();
    return L.presets[std::clamp(L.presetIndex, 0, (int)L.presets.size() - 1)];
}

std::string eepromPathFor(const DronePreset &p) {
    return pathToUtf8(pathFromUtf8(lib().eepromDir) / (p.fileName + ".eeprom"));
}

static int findPreset(const std::string &key) {
    Lib &L = lib();
    for (size_t i = 0; i < L.presets.size(); i++) {
        if (L.presets[i].fileName == key || L.presets[i].name == key) return (int)i;
    }
    // case-insensitive fallback
    auto low = [](std::string s) {
        for (char &c : s) c = (char)tolower((unsigned char)c);
        return s;
    };
    for (size_t i = 0; i < L.presets.size(); i++) {
        if (low(L.presets[i].fileName) == low(key) || low(L.presets[i].name) == low(key)) return (int)i;
    }
    return -1;
}

int runCli(const std::string &script, std::string &output) {
    Lib &L = lib();
    int rc = BFB_CLI_ERROR;
    L.runner.call([&] {
        char *out = nullptr;
        rc = bfb_cli_run(script.c_str(), &out);
        output = out ? out : "";
        bfb_free(out);
    });
    return rc;
}

// One-time upgrades of EEPROMs made by older libfpv versions, in order. The
// last one applied is recorded in a file next to the EEPROM, so a player's
// later edits are never undone.
static const char *const kEepromUpgrades[] = {"turtle-2", "air-osd-3"
#if defined(LIBFPV_KAMIKAZE)
                                              , "bomb-osd-4"
#endif
};
static const char *const kEepromUpgrade = kEepromUpgrades[std::size(kEepromUpgrades) - 1];
static fs::path upgradeMarker(const std::string &eeprom) { return pathFromUtf8(eeprom + ".upgrade"); }

// libfpv's first-run configuration plus the preset's own tune and the
// game's default diff, applied to the active EEPROM. Runs on the simulation
// thread.
static void applyFirstRunConfig(const DronePreset &p) {
    std::string script = betaflightFirstRunScript();
    if (!p.betaflightDiff.empty()) {
        DiffImport imp = buildDiffImport(p.betaflightDiff);
        // the preset tune replaces Betaflight defaults; keep libfpv's OSD/modes
        script = imp.script + betaflightFirstRunScript();
    }
    if (!lib().defaultDiff.empty()) {
        // the game's setup goes on top: without the import's own reset to
        // defaults, which would drop the preset tune
        std::string game = buildDiffImport(lib().defaultDiff).script;
        const std::string reset = "defaults nosave\n";
        if (game.rfind(reset, 0) == 0) game.erase(0, reset.size());
        script += game;
    }
    script += "save noreboot\n";
    char *out = nullptr;
    bfb_cli_run(script.c_str(), &out);
    bfb_free(out);
    writeFileAtomic(upgradeMarker(eepromPathFor(p)), kEepromUpgrade);
    FPV_LOGI("created Betaflight configuration for \"%s\" (%s)", p.name.c_str(), eepromPathFor(p).c_str());
}

static std::string g_activeEeprom;
static void warmUpBetaflight();

// Features (the simulated GPS of D22, air mode in a preset tune) and filters
// only take effect at boot.
static void rebootIfNeeded(bool always) {
    if (always || !bfb_gps_running() || bfb_features_pending()) bfb_reboot();
}

// Makes sure libfpv's essentials survived edits made in the Configurator
// (another motor protocol would leave the motors stopped), then reboots
// Betaflight if it has to, or always with `reboot` (a quad's EEPROM was just
// switched to or created). Simulation thread.
static void ensureEssentials(bool reboot) {
    char *o = nullptr;
    bfb_cli_run("get motor_pwm_protocol", &o);
    std::string out = o ? o : "";
    bfb_free(o);
    if (out.find("= PWM") == std::string::npos) {
        FPV_LOGW("motor protocol was changed away from PWM; restoring PWM");
        bfb_cli_run("set motor_pwm_protocol = PWM\nsave noreboot", &o);
        bfb_free(o);
    }

    // the simulated GPS (D22). Features only take effect at boot.
    if (!bfb_gps_configured()) {
        FPV_LOGI("enabling Betaflight's GPS feature");
        bfb_cli_run("feature GPS\nsave noreboot", &o);
        bfb_free(o);
    }

    std::string done;
    readTextFile(upgradeMarker(g_activeEeprom), done);
    size_t next = 0;  // first upgrade not applied yet
    for (size_t i = 0; i < std::size(kEepromUpgrades); i++)
        if (done == kEepromUpgrades[i]) next = i + 1;
    if (next == std::size(kEepromUpgrades)) {
        rebootIfNeeded(reboot);
        return;
    }
    bfb_cli_run("aux", &o);
    std::string aux = o ? o : "";
    bfb_free(o);
    std::string script;
    if (next == 0) {
        // turtle mode (D21): the AUX4 switch and the crash flip cut-off
        script += betaflightSwitchMigration(aux, {kTurtleSwitch});
        bfb_cli_run("get crashflip_rate", &o);
        const char *rate = o ? std::strstr(o, "crashflip_rate = ") : nullptr;
        if (rate && std::atoi(rate + 17) == 0) script += "set crashflip_rate = 30\n";
        bfb_free(o);
    }
    if (next <= 1) {
        // the air mode (AUX5) and OSD disable (AUX6) switches (D23); after
        // the turtle line, so they never take the slot it was given
        aux += script;
        script += betaflightSwitchMigration(aux, {kAirModeSwitch, kOsdSwitch});
    }
#if defined(LIBFPV_KAMIKAZE)
    if (next <= 2) {
        // the warhead state became the custom message 1 element; keep it
        // where the player put it if it is shown already
        bfb_cli_run("get osd_custom_msg0_pos", &o);
        const char *pos = o ? std::strstr(o, "osd_custom_msg0_pos = ") : nullptr;
        if (!pos || !(std::atoi(pos + 22) & 0x800)) script += betaflightBombOsdLine();
        bfb_free(o);
    }
#endif
    if (!script.empty()) {
        FPV_LOGI("upgrading Betaflight configuration for new switches and OSD elements");
        bfb_cli_run((script + "save noreboot").c_str(), &o);
        bfb_free(o);
    }
    writeFileAtomic(upgradeMarker(g_activeEeprom), kEepromUpgrade);
    rebootIfNeeded(reboot);
}

// Activates the preset's EEPROM (one virtual flight controller per quad).
// Runs on the simulation thread.
static void activateEepromFor(const DronePreset &p) {
    std::string path = eepromPathFor(p);
    if (path == g_activeEeprom) return;
    g_activeEeprom = path;
    int existed = bfb_switch_eeprom(path.c_str());
    if (existed == 0) applyFirstRunConfig(p);
    // reading an EEPROM only reloads the PID and rate profiles: gyro filters,
    // the mixer and features are set up at boot and would stay the previous
    // quad's
    ensureEssentials(true);
    warmUpBetaflight();  // reading an EEPROM suspends RX briefly
}

fpv_result selectPreset(const std::string &key, bool applyToFlight) {
    Lib &L = lib();
    int idx = key.empty() ? L.presetIndex : findPreset(key);
    if (idx < 0) return setError(FPV_ERROR_NOT_FOUND, "no preset named \"%s\"", key.c_str());
    L.presetIndex = idx;
    L.settings.preset = L.presets[idx].fileName;
    saveSettings();
    if (applyToFlight) {
        const DronePreset p = L.presets[idx];
        L.runner.call([&] {
            activateEepromFor(p);
            L.flight.applyPreset(p);
        });
    }
    return FPV_OK;
}

// Rewrites the shipped files a quad is made of (its JSON, Betaflight tune and
// components) from the copies compiled into libfpv. Returns false for a
// player's own quad, which libfpv has no copy of.
static bool restoreShippedPreset(const std::string &fileName) {
    Lib &L = lib();
    fs::path root = pathFromUtf8(L.presetDir);
    auto embedded = [](const std::string &path) -> const EmbeddedFile * {
        for (size_t i = 0; i < kEmbeddedPresetsCount; i++)
            if (path == kEmbeddedPresets[i].path) return &kEmbeddedPresets[i];
        return nullptr;
    };
    auto restore = [&](const EmbeddedFile &f) {
        fs::path p = root / pathFromUtf8(f.path);
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
        if (!writeFileAtomic(p, std::string((const char *)f.data, f.size)))
            FPV_LOGW("cannot restore %s", pathToUtf8(p).c_str());
    };
    const EmbeddedFile *quad = embedded("quad/" + fileName + ".json");
    if (!quad) return false;
    restore(*quad);
    if (const EmbeddedFile *diff = embedded("quad/" + fileName + ".diff.txt")) restore(*diff);
    json q = json::parse(quad->data, quad->data + quad->size, nullptr, false);
    // components are referenced by their "name" field or file stem
    for (const char *kind : {"motor", "propeller", "battery", "vtx"}) {
        if (!q.is_object() || !q.contains(kind) || !q[kind].is_string()) continue;
        std::string name = q[kind].get<std::string>(), dir = std::string(kind) + "/";
        for (size_t i = 0; i < kEmbeddedPresetsCount; i++) {
            const EmbeddedFile &f = kEmbeddedPresets[i];
            std::string path = f.path;
            if (path.rfind(dir, 0) != 0) continue;
            json c = json::parse(f.data, f.data + f.size, nullptr, false);
            std::string stem = pathToUtf8(pathFromUtf8(path).stem());
            if (stem == name || (c.is_object() && c.value("name", "") == name)) restore(f);
        }
    }
    return true;
}

// Re-reads the preset folder, keeping `fileName` selected if it still exists.
static void reloadPresets(const std::string &fileName) {
    Lib &L = lib();
    std::vector<std::string> errors;
    std::vector<DronePreset> presets = loadPresetDirectory(L.presetDir, errors);
    for (auto &e : errors) FPV_LOGW("preset ignored: %s", e.c_str());
    if (presets.empty()) presets.push_back(builtinDefaultPreset());
    L.presets = std::move(presets);
    int idx = findPreset(fileName);
    L.presetIndex = idx >= 0 ? idx : 0;
    L.settings.preset = currentPreset().fileName;
}

// Erases EEPROM files (with their upgrade markers) and flies the current
// preset on a fresh one with libfpv's first-run setup. `all` erases every
// quad's, otherwise only the current one's.
static void recreateEeprom(bool all) {
    Lib &L = lib();
    const DronePreset p = currentPreset();
    L.runner.call([&] {
        // keep nothing pending that would be written back to the old file
        if (bfb_config_dirty()) bfb_write_eeprom();
        std::error_code ec;
        if (all) {
            std::vector<fs::path> files;
            for (auto &e : fs::directory_iterator(pathFromUtf8(L.eepromDir), ec)) files.push_back(e.path());
            for (auto &f : files) fs::remove(f, ec);
        } else {
            std::string path = eepromPathFor(p);
            fs::remove(pathFromUtf8(path), ec);
            fs::remove(upgradeMarker(path), ec);
        }
        g_activeEeprom.clear();
        activateEepromFor(p);
        L.flight.applyPreset(p);
    });
}

fpv_result resetCurrentPreset() {
    std::string fileName = currentPreset().fileName;
    if (restoreShippedPreset(fileName)) reloadPresets(fileName);
    recreateEeprom(false);
    FPV_LOGI("reset \"%s\" to libfpv's configuration", currentPreset().name.c_str());
    return FPV_OK;
}

static bool materializePresets(std::string &error, bool force = false);

fpv_result resetEverything() {
    Lib &L = lib();
    // the overlay must stay reachable with the key the game chose
    int toggleKey = L.settings.overlayToggleKey;
    L.settings = Settings{};
    L.settings.overlayToggleKey = toggleKey;
    L.input.setKeyboardHover(L.settings.keyboardHover);
    L.input.setKeyboardThrottleExpo(L.settings.keyboardThrottleExpo);
    L.input.setKeyboardYawExpo(L.settings.keyboardYawExpo);
    L.input.forgetAllMappings();

    std::string err;
    if (!materializePresets(err, true)) FPV_LOGW("presets: %s", err.c_str());
    reloadPresets("freestyler-5");  // the first-launch drone
    saveSettings();
    recreateEeprom(true);
#if defined(LIBFPV_KAMIKAZE)
    applyKamikazeSettings();
#endif
    FPV_LOGI("reset every libfpv setting and drone to its defaults");
    return FPV_OK;
}

// --- data directory --------------------------------------------------------

static bool materializePresets(std::string &error, bool force) {
    Lib &L = lib();
    std::error_code ec;
    fs::path root = pathFromUtf8(L.presetDir);
    fs::create_directories(root, ec);
    // Shipped files are (re)written only when missing or when the shipped
    // version is newer; players' own files are never touched.
    std::string stamp;
    readTextFile(root / ".libfpv-version", stamp);
    bool upgrade = force || stamp != LIBFPV_VERSION_STRING;
    for (size_t i = 0; i < kEmbeddedPresetsCount; i++) {
        const EmbeddedFile &f = kEmbeddedPresets[i];
        fs::path p = root / pathFromUtf8(f.path);
        if (!upgrade && fs::exists(p, ec)) continue;
        fs::create_directories(p.parent_path(), ec);
        if (!writeFileAtomic(p, std::string((const char *)f.data, f.size))) {
            error = "cannot write " + pathToUtf8(p);
            return false;
        }
    }
    writeFileAtomic(root / ".libfpv-version", LIBFPV_VERSION_STRING);
    return true;
}

// Steps the parked simulation until Betaflight's boot-time sensor
// calibration is done (a few simulated seconds, ~50 ms of wall time), so a
// freshly spawned drone arms immediately. Simulation thread.
static void warmUpBetaflight() {
    Lib &L = lib();
    const float dt = 1.0f / std::max(200.0f, L.config.sim_rate_hz);
    float idle[kChannelCount];
    setIdleChannels(idle);
    double t0 = wallSeconds();
    int steps = 0;
    // the flag only appears once the scheduler ran, so always run 0.5 s first
    while ((steps * dt < 0.5f || bfb_settling()) && steps * dt < 10.0f) {
        L.flight.step(dt, idle, false);
        steps++;
    }
    FPV_LOGD("Betaflight warm-up: %.2f s simulated in %.0f ms", steps * dt, (wallSeconds() - t0) * 1000.0);
}

// --- exit handling -----------------------------------------------------------

static std::atomic<bool> g_exitHandlerInstalled{false};

static void shutdownForExit() {
    Lib &L = lib();
    std::unique_lock<std::recursive_mutex> lock(L.mutex, std::try_to_lock);
    if (!L.initialized) return;
    // Process exit with libfpv still running: stop our threads before
    // static destructors run, persist settings, never block for long.
    L.runner.callWithTimeout([&] {
        if (bfb_config_dirty()) bfb_write_eeprom();
        L.input.stopDevices();
    }, 1.0);
    L.runner.stop();
    SimITL::Sim::getInstance().stop();
    L.initialized = false;
}

}  // namespace fpv

using namespace fpv;

// ============================================================================

extern "C" {

FPV_API uint32_t fpv_abi_version(void) { return LIBFPV_ABI_VERSION; }
FPV_API const char *fpv_version_string(void) { return LIBFPV_VERSION_STRING; }

FPV_API const char *fpv_result_string(fpv_result r) {
    switch (r) {
    case FPV_OK: return "FPV_OK";
    case FPV_ERROR_INVALID_ARGUMENT: return "FPV_ERROR_INVALID_ARGUMENT";
    case FPV_ERROR_NOT_INITIALIZED: return "FPV_ERROR_NOT_INITIALIZED";
    case FPV_ERROR_ALREADY_INITIALIZED: return "FPV_ERROR_ALREADY_INITIALIZED";
    case FPV_ERROR_DRONE_EXISTS: return "FPV_ERROR_DRONE_EXISTS";
    case FPV_ERROR_NOT_FOUND: return "FPV_ERROR_NOT_FOUND";
    case FPV_ERROR_IO: return "FPV_ERROR_IO";
    case FPV_ERROR_BUSY: return "FPV_ERROR_BUSY";
    case FPV_ERROR_ABI_MISMATCH: return "FPV_ERROR_ABI_MISMATCH";
    case FPV_ERROR_PARSE: return "FPV_ERROR_PARSE";
    case FPV_ERROR_TIMEOUT: return "FPV_ERROR_TIMEOUT";
    case FPV_ERROR_INTERNAL: return "FPV_ERROR_INTERNAL";
    }
    return "FPV_ERROR_UNKNOWN";
}

FPV_API const char *fpv_last_error(void) { return t_lastError.c_str(); }

FPV_API void fpv_free(void *ptr) { free(ptr); }

static char *dupString(const std::string &s) {
    char *p = (char *)malloc(s.size() + 1);
    if (p) memcpy(p, s.c_str(), s.size() + 1);
    return p;
}

FPV_API fpv_coord_system fpv_coords_preset(fpv_coords_preset_id preset, float upm) {
    fpv_coord_system c{};
    c.units_per_meter = upm > 0 ? upm : 1.0f;
    switch (preset) {
    case FPV_COORDS_Y_UP_LEFT_HANDED:
        c.right = FPV_AXIS_POS_X; c.up = FPV_AXIS_POS_Y; c.forward = FPV_AXIS_POS_Z;
        break;
    case FPV_COORDS_Z_UP_LEFT_HANDED:
        c.right = FPV_AXIS_POS_Y; c.up = FPV_AXIS_POS_Z; c.forward = FPV_AXIS_POS_X;
        break;
    case FPV_COORDS_Z_UP_RIGHT_HANDED:
        c.right = FPV_AXIS_NEG_Y; c.up = FPV_AXIS_POS_Z; c.forward = FPV_AXIS_POS_X;
        break;
    case FPV_COORDS_Y_UP_RIGHT_HANDED:
    default:
        c.right = FPV_AXIS_POS_X; c.up = FPV_AXIS_POS_Y; c.forward = FPV_AXIS_NEG_Z;
        break;
    }
    return c;
}

FPV_API fpv_result fpv_shared_data_dir(char *out, uint32_t outSize) {
    std::string dir = envUtf8("LIBFPV_DATA_DIR");
#if defined(__EMSCRIPTEN__)
    if (dir.empty()) return setError(FPV_ERROR_NOT_FOUND, "no shared data folder in a browser: use FPV_WEB_DATA_DIR");
#else
    if (dir.empty() && !userConfigDir().empty()) dir = pathToUtf8(pathFromUtf8(userConfigDir()) / "libfpv");
#endif
    if (dir.empty()) return setError(FPV_ERROR_NOT_FOUND, "no home folder for the shared data folder");
    if (!out || outSize < dir.size() + 1)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "out_size %u is too small, %u bytes needed", outSize,
                        uint32_t(dir.size() + 1));
    memcpy(out, dir.c_str(), dir.size() + 1);
    return FPV_OK;
}

FPV_API void fpv_global_config_defaults(fpv_global_config *c) {
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->struct_size = sizeof(*c);
    c->coords = fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1.0f);
    c->log_level = FPV_LOG_INFO;
    c->configurator_port = 5761;
    c->timing = FPV_TIMING_THREAD;
    c->sim_rate_hz = 1000.0f;
    c->auto_pause_seconds = 0.5f;
}

FPV_API int fpv_is_initialized(void) { return lib().initialized ? 1 : 0; }

FPV_API fpv_result fpv_global_init(const fpv_global_config *cfgIn) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    clearError();
    if (L.initialized) return setError(FPV_ERROR_ALREADY_INITIALIZED, "fpv_global_init called twice");
    if (!cfgIn) return setError(FPV_ERROR_INVALID_ARGUMENT, "config is NULL");
    if (cfgIn->struct_size < offsetof(fpv_global_config, flags) + sizeof(uint32_t))
        return setError(FPV_ERROR_ABI_MISMATCH, "fpv_global_config.struct_size %u is too small (use fpv_global_config_defaults)",
                        cfgIn->struct_size);
    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    memcpy(&cfg, cfgIn, std::min<size_t>(sizeof(cfg), cfgIn->struct_size));
    cfg.struct_size = sizeof(cfg);
    if (!cfg.data_dir || !cfg.data_dir[0]) return setError(FPV_ERROR_INVALID_ARGUMENT, "data_dir is required");
    if (!L.conv.setup(cfg.coords.right, cfg.coords.up, cfg.coords.forward, cfg.coords.units_per_meter))
        return setError(FPV_ERROR_INVALID_ARGUMENT, "coords: right/up/forward must name three different axes and units_per_meter > 0");
    if (cfg.sim_rate_hz <= 0) cfg.sim_rate_hz = 1000.0f;
    if (cfg.configurator_port <= 0) cfg.configurator_port = 5761;
#if defined(__EMSCRIPTEN__)
    cfg.flags |= FPV_INIT_NO_CONFIGURATOR;  // a web page cannot accept connections
#endif

    logSetSink((LogSink)cfg.log, cfg.log_user, (LogLevel)std::clamp((int)cfg.log_level, 0, 3));
#if defined(__EMSCRIPTEN__)
    {
        char *kept = libfpv_web_data_dir();
        std::string dir = cfg.data_dir, keptDir = kept;
        free(kept);
        if (!keptDir.empty() && dir != keptDir && dir.rfind(keptDir + "/", 0) != 0)
            FPV_LOGW("data_dir \"%s\" is not inside \"%s\", the folder this browser keeps: settings will be lost "
                     "on reload (pass FPV_WEB_DATA_DIR, or set Module.libfpvDataDir)",
                     cfg.data_dir, keptDir.c_str());
    }
#endif

    // The data dir pointer is only valid during this call.
    L.dataDir = cfg.data_dir;
    cfg.data_dir = nullptr;
    L.config = cfg;
    fs::path root = pathFromUtf8(L.dataDir);
    std::error_code ec;
    fs::create_directories(root, ec);
    if (!fs::is_directory(root, ec)) return setError(FPV_ERROR_IO, "cannot create data_dir \"%s\"", L.dataDir.c_str());
    L.presetDir = pathToUtf8(root / "presets");
    L.eepromDir = pathToUtf8(root / "eeprom");
    fs::create_directories(pathFromUtf8(L.eepromDir), ec);

    pinLibraryInMemory();

    std::string err;
    if (!materializePresets(err)) FPV_LOGW("presets: %s", err.c_str());
    std::vector<std::string> errors;
    L.presets = loadPresetDirectory(L.presetDir, errors);
    for (auto &e : errors) FPV_LOGW("preset ignored: %s", e.c_str());
    if (L.presets.empty()) L.presets.push_back(builtinDefaultPreset());
    loadSettings();
    L.input.setKeyboardHover(L.settings.keyboardHover);
    L.input.setKeyboardThrottleExpo(L.settings.keyboardThrottleExpo);
    L.input.setKeyboardYawExpo(L.settings.keyboardYawExpo);
    L.presetIndex = 0;
    if (!L.settings.preset.empty()) {
        int idx = findPreset(L.settings.preset);
        if (idx >= 0) L.presetIndex = idx;
    } else {
        int idx = findPreset("freestyler-5");
        if (idx >= 0) L.presetIndex = idx;
    }

    // Betaflight is initialised once per process and survives terminate.
    L.world.setMaterial(0, Material{});
    L.flight.setWorld(&L.world);
    const DronePreset &preset = currentPreset();
    if (!L.flight.betaflightReady()) {
        bfb_ws_set_base_port(cfg.configurator_port - 1);
        bfb_ws_set_enabled(!(cfg.flags & FPV_INIT_NO_CONFIGURATOR));
        std::string eeprom = eepromPathFor(preset);
        bool fresh = !fs::exists(pathFromUtf8(eeprom), ec);
        // Betaflight's init creates a missing EEPROM with its own defaults,
        // so freshness is decided before.
        L.flight.initBetaflight(eeprom, preset);
        g_activeEeprom = eeprom;
        if (fresh) applyFirstRunConfig(preset);
        ensureEssentials(fresh);  // the first-run tune is saved without a reboot
        L.flight.applyPreset(preset);
        warmUpBetaflight();
    } else {
        bfb_ws_set_enabled(!(cfg.flags & FPV_INIT_NO_CONFIGURATOR));
        if (!(cfg.flags & FPV_INIT_NO_CONFIGURATOR)) {
            if (bfb_ws_base_port() != cfg.configurator_port - 1) {
                FPV_LOGW("configurator port cannot change after the first init in a process; keeping %d",
                         bfb_ws_base_port() + 1);
            }
            SimITL::Sim::getInstance().startServices();
        }
        activateEepromFor(preset);
        L.flight.applyPreset(preset);
    }

#if defined(LIBFPV_KAMIKAZE)
    applyKamikazeSettings();
#endif

    RunnerConfig rc;
    rc.threaded = cfg.timing == FPV_TIMING_THREAD;
    rc.simRateHz = cfg.sim_rate_hz;
    rc.autoPauseSeconds = (cfg.flags & FPV_INIT_NO_AUTO_PAUSE) ? 0.0f : cfg.auto_pause_seconds;
    L.runner.configure(rc);
    L.input.setDataDir(L.dataDir);
    L.runner.start();

    if (!(cfg.flags & FPV_INIT_NO_DEVICE_INPUT)) {
        // SDL is initialised and polled on the simulation thread.
        std::string inputErr;
        bool ok = L.runner.call([&] { return L.input.startDevices(inputErr); });
        if (!ok) FPV_LOGW("radio/gamepad input unavailable: %s", inputErr.c_str());
    }

    if (!g_exitHandlerInstalled.exchange(true)) installExitHandler(&shutdownForExit);

    L.initialized = true;
    L.everInitialized = true;
    FPV_LOGI("libfpv %s ready (preset \"%s\", %s timing, configurator %s)", LIBFPV_VERSION_STRING,
             currentPreset().name.c_str(), rc.threaded ? "threaded" : "manual",
             (cfg.flags & FPV_INIT_NO_CONFIGURATOR) ? "off" : ("ws://localhost:" + std::to_string(bfb_ws_base_port() + 1)).c_str());
    return FPV_OK;
}

FPV_API void fpv_global_terminate(void) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return;
    L.runner.call([&] {
        if (L.droneExists) L.flight.despawn();
        if (bfb_config_dirty()) bfb_write_eeprom();
        L.input.stopDevices();
    });
    L.droneExists = false;
    L.drone.magic = 0;
    L.runner.stop();
    SimITL::Sim::getInstance().stop();
    saveSettings();
    L.initialized = false;
    FPV_LOGI("libfpv terminated (Betaflight state kept for a later fpv_global_init)");
    logSetSink(nullptr, nullptr, LogLevel::Error);
}

// --- world ---------------------------------------------------------------------

#define REQUIRE_INIT()                                                                 \
    Lib &L = lib();                                                                    \
    std::lock_guard<std::recursive_mutex> lock_(L.mutex);                              \
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first")

static bool meshToTriangles(const fpv_mesh &m, const Vec3 &scale, bool toInternal, std::vector<Triangle> &out) {
    if (!m.positions || m.vertex_count == 0) return m.triangle_count == 0;
    uint32_t stride = m.vertex_stride ? m.vertex_stride : 12;
    const uint8_t *base = (const uint8_t *)m.positions;
    auto vtx = [&](uint32_t i) {
        const float *p = (const float *)(base + (size_t)i * stride);
        Vec3 h{p[0] * scale.x, p[1] * scale.y, p[2] * scale.z};
        return toInternal ? lib().conv.posToInternal(h) : h;
    };
    out.reserve(out.size() + m.triangle_count);
    for (uint32_t t = 0; t < m.triangle_count; t++) {
        uint32_t i0, i1, i2;
        if (m.indices) {
            i0 = m.indices[t * 3];
            i1 = m.indices[t * 3 + 1];
            i2 = m.indices[t * 3 + 2];
        } else {
            i0 = t * 3;
            i1 = t * 3 + 1;
            i2 = t * 3 + 2;
        }
        if (i0 >= m.vertex_count || i1 >= m.vertex_count || i2 >= m.vertex_count) return false;
        Triangle tri{vtx(i0), vtx(i1), vtx(i2), m.materials ? m.materials[t] : m.material};
        out.push_back(tri);
    }
    return true;
}

FPV_API fpv_result fpv_material_set(uint8_t id, const fpv_material *m) {
    REQUIRE_INIT();
    if (!m) return setError(FPV_ERROR_INVALID_ARGUMENT, "material is NULL");
    Material mat;
    mat.restitution = std::clamp(m->restitution, 0.0f, 1.0f);
    mat.friction = std::max(0.0f, m->friction);
    mat.flags = m->flags;
    L.world.setMaterial(id, mat);
    return FPV_OK;
}

FPV_API fpv_result fpv_static_surfaces_load(const fpv_mesh *meshes, uint32_t count) {
    REQUIRE_INIT();
    if (!meshes && count) return setError(FPV_ERROR_INVALID_ARGUMENT, "meshes is NULL");
    std::vector<Triangle> tris;
    for (uint32_t i = 0; i < count; i++) {
        if (!meshToTriangles(meshes[i], {1, 1, 1}, true, tris))
            return setError(FPV_ERROR_INVALID_ARGUMENT, "mesh %u: index out of range or no positions", i);
    }
    size_t n = tris.size();
    double t0 = wallSeconds();
    L.world.loadStatic(std::move(tris));
    FPV_LOGI("static collision: %zu triangles (%.0f ms)", n, (wallSeconds() - t0) * 1000.0);
    return FPV_OK;
}

FPV_API fpv_result fpv_static_surfaces_clear(void) {
    REQUIRE_INIT();
    L.world.loadStatic({});
    return FPV_OK;
}

static RigidTransform toInternalTransform(const fpv_transform &t) {
    Lib &L = lib();
    Quat q{t.rotation[0], t.rotation[1], t.rotation[2], t.rotation[3]};
    if (qdot(q, q) < 1e-12f) q = Quat{};
    return {L.conv.posToInternal({t.position[0], t.position[1], t.position[2]}), L.conv.rotToInternal(q)};
}

static Vec3 scaleOf(const fpv_transform *t) {
    if (!t) return {1, 1, 1};
    Vec3 s{t->scale[0], t->scale[1], t->scale[2]};
    for (int i = 0; i < 3; i++)
        if (s[i] == 0) s[i] = 1;
    return s;
}

FPV_API fpv_result fpv_surface_object_create(const fpv_mesh *mesh, const fpv_transform *t, uint32_t *outId) {
    REQUIRE_INIT();
    if (!mesh || !outId) return setError(FPV_ERROR_INVALID_ARGUMENT, "mesh and out_object_id are required");
    // Object space: scale is baked in, then converted to internal axes and metres.
    std::vector<Triangle> tris;
    if (!meshToTriangles(*mesh, scaleOf(t), true, tris))
        return setError(FPV_ERROR_INVALID_ARGUMENT, "mesh: index out of range or no positions");
    fpv_transform ident{};
    ident.rotation[3] = 1;
    *outId = L.world.createObject(std::move(tris), toInternalTransform(t ? *t : ident), L.flight.simTime());
    return FPV_OK;
}

FPV_API fpv_result fpv_surface_object_move(uint32_t id, const fpv_transform *t) {
    REQUIRE_INIT();
    if (!t) return setError(FPV_ERROR_INVALID_ARGUMENT, "transform is NULL");
    if (!L.world.moveObject(id, toInternalTransform(*t), L.flight.simTime()))
        return setError(FPV_ERROR_NOT_FOUND, "no surface object %u", id);
    return FPV_OK;
}

FPV_API fpv_result fpv_surface_object_delete(uint32_t id) {
    REQUIRE_INIT();
    if (!L.world.deleteObject(id)) return setError(FPV_ERROR_NOT_FOUND, "no surface object %u", id);
    return FPV_OK;
}

FPV_API void fpv_world_set_raycast_callback(fpv_raycast_fn cb, void *user) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    L.raycast = cb;
    L.raycastUser = user;
    if (!cb) L.world.setProxyPlanes({});
}

// --- drone -----------------------------------------------------------------

#define REQUIRE_DRONE(d)                                                                  \
    REQUIRE_INIT();                                                                       \
    if (!(d) || (d) != &L.drone || !L.droneExists)                                        \
        return setError(FPV_ERROR_INVALID_ARGUMENT, "invalid or deleted drone handle")

FPV_API void fpv_drone_desc_defaults(fpv_drone_desc *d) {
    if (!d) return;
    memset(d, 0, sizeof(*d));
    d->struct_size = sizeof(*d);
    d->rotation[3] = 1.0f;
}

FPV_API fpv_result fpv_drone_create(const fpv_drone_desc *descIn, fpv_drone **out) {
    REQUIRE_INIT();
    if (!out) return setError(FPV_ERROR_INVALID_ARGUMENT, "out_drone is NULL");
    *out = nullptr;
    if (L.droneExists)
        return setError(FPV_ERROR_DRONE_EXISTS,
                        "only one drone can exist per process (Betaflight is a singleton); delete the existing one first");
    fpv_drone_desc desc;
    fpv_drone_desc_defaults(&desc);
    if (descIn) memcpy(&desc, descIn, std::min<size_t>(sizeof(desc), descIn->struct_size ? descIn->struct_size : sizeof(desc)));
    if (desc.preset && desc.preset[0]) {
        fpv_result r = selectPreset(desc.preset, false);
        if (r != FPV_OK) return r;
    }
    Quat rotH{desc.rotation[0], desc.rotation[1], desc.rotation[2], desc.rotation[3]};
    if (qdot(rotH, rotH) < 1e-12f) rotH = Quat{};
    Vec3 pos = L.conv.posToInternal({desc.position[0], desc.position[1], desc.position[2]});
    Quat rot = L.conv.rotToInternal(rotH);
    Vec3 pilotH{desc.pilot_position[0], desc.pilot_position[1], desc.pilot_position[2]};
    Vec3 pilot = length2(pilotH) > 0 ? L.conv.posToInternal(pilotH) : pos;
    const DronePreset preset = currentPreset();
    L.runner.call([&] {
        activateEepromFor(preset);
        L.flight.spawn(preset, {0, 0, 0}, rot, pilot);
        L.flight.teleport(pos + rotate(rot, Vec3{0, L.flight.restHeight() + 0.002f, 0}), rot);
    });
    L.droneExists = true;
    L.spawnPos = pos;
    L.spawnRot = rot;
    L.windProbe.reset();  // the air around the last drone is not this one's
    L.drone.magic = 0xF9D0u;
    L.chaseValid = false;
    L.runner.heartbeat();
    *out = &L.drone;
    return FPV_OK;
}

FPV_API fpv_result fpv_drone_delete(fpv_drone *d) {
    REQUIRE_DRONE(d);
    L.runner.call([&] { L.flight.despawn(); });
    L.droneExists = false;
    L.drone.magic = 0;
    return FPV_OK;
}

FPV_API fpv_result fpv_drone_reset(fpv_drone *d, const float position[3], const float rotation[4]) {
    REQUIRE_DRONE(d);
    if (!position) return setError(FPV_ERROR_INVALID_ARGUMENT, "position is NULL");
    Vec3 pos = L.conv.posToInternal({position[0], position[1], position[2]});
    Quat rot;
    if (rotation) {
        rot = L.conv.rotToInternal({rotation[0], rotation[1], rotation[2], rotation[3]});
    } else {
        // keep the current heading, upright
        Vec3 fwd = rotate(L.flight.snapshot().rot, {0, 0, 1});
        fwd.y = 0;
        rot = lookRotation(length2(fwd) > 1e-6f ? fwd : Vec3{0, 0, 1}, {0, 1, 0});
    }
    L.runner.call([&] {
        L.flight.teleport(pos + rotate(rot, Vec3{0, L.flight.restHeight() + 0.002f, 0}), rot);
        L.flight.resetFlightTime();  // a reset starts a new flight
    });
    L.chaseValid = false;
    L.windProbe.reset();
    return FPV_OK;
}

FPV_API fpv_result fpv_drone_repair(fpv_drone *d) {
    REQUIRE_DRONE(d);
    L.runner.call([&] { L.flight.repair(); });
    return FPV_OK;
}

FPV_API fpv_result fpv_drone_set_preset(fpv_drone *d, const char *preset) {
    REQUIRE_DRONE(d);
    if (!preset) return setError(FPV_ERROR_INVALID_ARGUMENT, "preset is NULL");
    return selectPreset(preset, true);
}

// --- GPS -------------------------------------------------------------------------

FPV_API fpv_result fpv_gps_set_home(const float position[3]) {
    REQUIRE_INIT();
    if (!position) {
        L.flight.setHome(nullptr);
        return FPV_OK;
    }
    Vec3 home = L.conv.posToInternal({position[0], position[1], position[2]});
    if (!finite(home)) return setError(FPV_ERROR_INVALID_ARGUMENT, "home position is not finite");
    L.flight.setHome(&home);
    return FPV_OK;
}

// --- wind ------------------------------------------------------------------------

FPV_API void fpv_wind_defaults(fpv_wind *wind) {
    if (!wind) return;
    std::memset(wind, 0, sizeof(*wind));
    wind->struct_size = sizeof(*wind);
    wind->turbulence = -1.0f;
}

FPV_API fpv_result fpv_wind_set(const fpv_wind *wind) {
    REQUIRE_INIT();
    if (!wind) {
        L.windSet = false;
        L.windAmbient = WindAmbient{};
        return FPV_OK;
    }
    if (wind->struct_size < sizeof(fpv_wind)) return setError(FPV_ERROR_ABI_MISMATCH, "set wind->struct_size = sizeof(fpv_wind)");
    Vec3 v = L.conv.dirToInternal({wind->velocity[0], wind->velocity[1], wind->velocity[2]}) / L.conv.unitsPerMeter();
    if (!finite(v) || !std::isfinite(wind->reference_height) || !std::isfinite(wind->turbulence))
        return setError(FPV_ERROR_INVALID_ARGUMENT, "wind values are not finite");
    WindAmbient a;
    a.velocity = v;
    a.refHeight = wind->reference_height > 0 ? L.conv.lenToInternal(wind->reference_height) : 10.0f;
    a.turbulence = wind->turbulence < 0 ? -1.0f : std::min(wind->turbulence, 1.0f);
    a.noShelter = (wind->flags & FPV_WIND_NO_SHELTER) != 0;
    a.enclosed = (wind->flags & FPV_WIND_ENCLOSED) != 0;
    L.windAmbient = a;
    L.windSet = true;
    return FPV_OK;
}

FPV_API fpv_result fpv_wind_set_sources(const fpv_wind_source *sources, uint32_t count) {
    REQUIRE_INIT();
    if (count && !sources) return setError(FPV_ERROR_INVALID_ARGUMENT, "sources is NULL");
    L.windSources.clear();
    for (uint32_t i = 0; i < count && L.windSources.size() < (size_t)kMaxWindSources; i++) {
        const fpv_wind_source &h = sources[i];
        WindSource w;
        w.pos = L.conv.posToInternal({h.position[0], h.position[1], h.position[2]});
        w.dir = normalize(L.conv.dirToInternal({h.direction[0], h.direction[1], h.direction[2]}), {0, -1, 0});
        w.speed = L.conv.lenToInternal(h.speed);
        w.radius = L.conv.lenToInternal(h.radius);
        w.type = h.type == FPV_WIND_SOURCE_JET ? WindSource::Jet : WindSource::Rotor;
        if (!finite(w.pos) || !finite(w.dir) || !(w.speed >= 0) || !(w.radius > 0) || !std::isfinite(w.speed))
            continue;  // skipped, not fatal: one bad source must not stop the rest
        L.windSources.push_back(w);
    }
    return FPV_OK;
}

FPV_API fpv_result fpv_wind_add_blast(const float position[3], float radius) {
    REQUIRE_INIT();
    if (!position) return setError(FPV_ERROR_INVALID_ARGUMENT, "position is NULL");
    Vec3 p = L.conv.posToInternal({position[0], position[1], position[2]});
    float r = L.conv.lenToInternal(radius);
    if (!finite(p) || !(r > 0) || !std::isfinite(r)) return setError(FPV_ERROR_INVALID_ARGUMENT, "invalid blast");
    if (L.settings.windEnabled) L.flight.addBlast(p, r);
    return FPV_OK;
}

// --- time ------------------------------------------------------------------------

FPV_API void fpv_set_paused(int p) { lib().runner.setPaused(p != 0); }
FPV_API int fpv_is_paused(void) { return lib().runner.paused() || lib().runner.autoPaused(); }
FPV_API void fpv_set_time_scale(float s) {
    if (std::isfinite(s)) lib().runner.setTimeScale(std::clamp(s, 0.05f, 4.0f));
}

FPV_API fpv_result fpv_step(float seconds) {
    REQUIRE_INIT();
    if (L.runner.threaded())
        return setError(FPV_ERROR_INVALID_ARGUMENT, "fpv_step requires timing = FPV_TIMING_MANUAL");
    if (!(seconds >= 0) || seconds > 60) return setError(FPV_ERROR_INVALID_ARGUMENT, "seconds must be 0..60");
    L.runner.stepManual(seconds);
    return FPV_OK;
}

// --- input -------------------------------------------------------------------------

FPV_API fpv_result fpv_input_get_status(fpv_input_status *s) {
    if (!s) return setError(FPV_ERROR_INVALID_ARGUMENT, "status is NULL");
    InputStatus st = lib().input.status();
    uint32_t size = s->struct_size;
    fpv_input_status out{};
    out.struct_size = sizeof(out);
    out.source = (fpv_input_source)st.source;
    snprintf(out.device_name, sizeof(out.device_name), "%s", st.deviceName.c_str());
    out.needs_setup = st.needsSetup;
    out.device_count = st.deviceCount;
    memcpy(s, &out, std::min<size_t>(size ? size : sizeof(out), sizeof(out)));
    return FPV_OK;
}

FPV_API void fpv_input_set_channels(const float *ch, uint32_t count) {
    if (!ch) return;
    lib().input.setHostChannels(ch, count, wallSeconds());
}

FPV_API void fpv_input_clear_channels(void) { lib().input.clearHostChannels(); }

// --- configuration -------------------------------------------------------------

FPV_API fpv_result fpv_cli_exec(const char *commands, char **outOutput) {
    REQUIRE_INIT();
    if (outOutput) *outOutput = nullptr;
    if (!commands) return setError(FPV_ERROR_INVALID_ARGUMENT, "commands is NULL");
    CliFilterResult f = filterCliCommands(commands);
    std::string output;
    for (auto &r : f.refused) output += "# refused: " + r + "\n";
    std::string cliOut;
    int rc = f.script.empty() ? BFB_CLI_OK : runCli(f.script, cliOut);
    output += cliOut;
    if (outOutput) *outOutput = dupString(output);
    if (rc == BFB_CLI_BUSY) return setError(FPV_ERROR_BUSY, "the Betaflight Configurator is using the CLI");
    if (rc != BFB_CLI_OK) return setError(FPV_ERROR_INTERNAL, "CLI session failed");
    return FPV_OK;
}

FPV_API fpv_result fpv_import_betaflight_diff(const char *diff, char **outReport) {
    REQUIRE_INIT();
    if (outReport) *outReport = nullptr;
    if (!diff) return setError(FPV_ERROR_INVALID_ARGUMENT, "diff_text is NULL");
    DiffImport imp = buildDiffImport(diff);
    if (imp.applied.empty() && imp.skipped.empty())
        return setError(FPV_ERROR_PARSE, "no Betaflight CLI lines found (expected the output of `diff all`)");
    if (L.droneExists && L.flight.snapshot().armed) return setError(FPV_ERROR_BUSY, "disarm before importing a configuration");
    std::string cliOut;
    int rc = runCli(imp.script, cliOut);
    if (rc == BFB_CLI_BUSY) return setError(FPV_ERROR_BUSY, "the Betaflight Configurator is using the CLI");
    // features (air mode) only take effect at boot, and Betaflight does not
    // flag them as needing one
    bool reboot = L.runner.call([] { return bfb_reboot_required() != 0 || bfb_features_pending() != 0; });
    if (reboot) {
        L.runner.call([] {
            bfb_reboot();
            warmUpBetaflight();
        });
    }
    std::string report = formatImportReport(imp, cliOut);
    FPV_LOGI("imported Betaflight diff: %zu applied, %zu skipped", imp.applied.size(), imp.skipped.size());
    if (outReport) *outReport = dupString(report);
    return rc == BFB_CLI_OK ? FPV_OK : setError(FPV_ERROR_INTERNAL, "CLI session failed");
}

FPV_API void fpv_set_default_betaflight_diff(const char *diff) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    L.defaultDiff = diff ? diff : "";
}

FPV_API uint32_t fpv_preset_count(void) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    return L.initialized ? (uint32_t)L.presets.size() : 0;
}

FPV_API const char *fpv_preset_name(uint32_t i) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    return (L.initialized && i < L.presets.size()) ? L.presets[i].name.c_str() : nullptr;
}

FPV_API const char *fpv_preset_directory(void) {
    Lib &L = lib();
    return L.initialized ? L.presetDir.c_str() : nullptr;
}

FPV_API fpv_result fpv_configurator_get_info(fpv_configurator_info *info) {
    REQUIRE_INIT();
    if (!info) return setError(FPV_ERROR_INVALID_ARGUMENT, "info is NULL");
    fpv_configurator_info out{};
    out.struct_size = sizeof(out);
    out.enabled = !(L.config.flags & FPV_INIT_NO_CONFIGURATOR);
    out.port = bfb_ws_base_port() + 1;
    out.client_count = bfb_ws_client_count();
    snprintf(out.url, sizeof(out.url), "ws://localhost:%d", out.port);
    uint32_t size = info->struct_size ? info->struct_size : sizeof(out);
    memcpy(info, &out, std::min<size_t>(size, sizeof(out)));
    return FPV_OK;
}

}  // extern "C"
