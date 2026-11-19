// libfpv - the simulation server: Betaflight's configuration per quad, the
// drone and the real-time loop (docs/DECISIONS.md, D37).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "sim/sim_server.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <sstream>

#include "config/bf_config.h"
#include "core/bf_bridge.h"
#include "sim/sim.h"
#include "util/fs.h"
#include "util/log.h"

namespace fs = std::filesystem;

namespace fpv {

namespace {

// One-time upgrades of EEPROMs made by older libfpv versions, in order. The
// last one applied is recorded in a file next to the EEPROM, so a player's
// later edits are never undone.
const char *const kEepromUpgrades[] = {"turtle-2", "air-osd-3"
#if defined(LIBFPV_KAMIKAZE)
                                       , "bomb-osd-4"
#endif
                                       , "crashflip-tune-5"
};
// Index of the upgrade called `name` (the kamikaze one is not in every build).
size_t upgradeIndex(const char *name) {
    for (size_t i = 0; i < std::size(kEepromUpgrades); i++)
        if (std::strcmp(kEepromUpgrades[i], name) == 0) return i;
    return std::size(kEepromUpgrades);
}
const char *const kEepromUpgrade = kEepromUpgrades[std::size(kEepromUpgrades) - 1];
fs::path upgradeMarker(const std::string &eeprom) { return pathFromUtf8(eeprom + ".upgrade"); }

// Features (the simulated GPS of D22, air mode in a preset tune) and filters
// only take effect at boot.
void rebootIfNeeded(bool always) {
    if (always || !bfb_gps_running() || bfb_features_pending()) bfb_reboot();
}

}  // namespace

SimServer::SimServer(ChannelSource &input, CollisionWorld *sharedWorld)
    : m_input(input),
      m_ownWorld(sharedWorld ? nullptr : std::make_unique<CollisionWorld>()),
      m_world(sharedWorld ? sharedWorld : m_ownWorld.get()),
      m_worldShared(sharedWorld != nullptr) {
    m_flight.setWorld(m_world);
}

SimServer::~SimServer() { m_runner.stop(); }

std::string SimServer::eepromPathFor(const DronePreset &p) const {
    return pathToUtf8(pathFromUtf8(m_cfg.eepromDir) / (p.fileName + ".eeprom"));
}

std::string SimServer::cliRun(const std::string &script) {
    char *o = nullptr;
    bfb_cli_run(script.c_str(), &o);
    std::string out = o ? o : "";
    bfb_free(o);
    return out;
}

// libfpv's first-run configuration plus the preset's own tune and the
// game's default diff, applied to the active EEPROM.
void SimServer::applyFirstRunConfig(const DronePreset &p) {
    std::string script = betaflightFirstRunScript(presetNtsc(p));
    if (!p.betaflightDiff.empty()) {
        DiffImport imp = buildDiffImport(p.betaflightDiff, presetNtsc(p));
        // the preset tune replaces Betaflight defaults; keep libfpv's OSD/modes
        script = imp.script + betaflightFirstRunScript(presetNtsc(p));
    }
    if (!m_cfg.defaultDiff.empty()) {
        // the game's setup goes on top: without the import's own reset to
        // defaults, which would drop the preset tune
        std::string game = buildDiffImport(m_cfg.defaultDiff, presetNtsc(p)).script;
        const std::string reset = "defaults nosave\n";
        if (game.rfind(reset, 0) == 0) game.erase(0, reset.size());
        script += game;
    }
    script += "save noreboot\n";
    cliRun(script);
    writeFileAtomic(upgradeMarker(eepromPathFor(p)), kEepromUpgrade);
    FPV_LOGI("created Betaflight configuration for \"%s\" (%s)", p.name.c_str(), eepromPathFor(p).c_str());
}

// Betaflight's video system follows the quad's camera, as on a real flight
// controller set up for it: NTSC shows 13 OSD rows (SimITL patch 0019), and
// Betaflight moves elements below the last row up to it at boot. libfpv's
// own default layout, untouched, is swapped for the one made for the new
// system. True when it changed: Betaflight must reboot.
bool SimServer::syncVideoSystem(const DronePreset &p) {
    bool ntsc = presetNtsc(p);
    std::string out = cliRun("get vcd_video_system");
    const char *want = ntsc ? "NTSC" : "PAL";
    if (out.find(std::string("= ") + want) != std::string::npos) return false;
    std::string script = std::string("set vcd_video_system = ") + want + "\n";
    // is the layout still libfpv's default for the other system?
    std::vector<std::string> other;
    {
        std::istringstream in(betaflightOsdLayout(!ntsc));
        for (std::string line; std::getline(in, line);) other.push_back(line);
    }
    std::string gets;
    for (const std::string &line : other) {
        size_t eq = line.find(" = ");
        if (line.rfind("set ", 0) == 0 && eq != std::string::npos) gets += "get " + line.substr(4, eq - 4) + "\n";
    }
    std::string current = cliRun(gets);
    bool untouched = true;
    for (const std::string &line : other) {
        if (line.rfind("set ", 0) == 0 && current.find(line.substr(4)) == std::string::npos) untouched = false;
    }
    if (untouched) script += betaflightOsdLayout(ntsc);
    FPV_LOGI("Betaflight video system: %s (%s)", want, untouched ? "default OSD layout moved" : "OSD layout kept");
    cliRun(script + "save noreboot");
    return true;
}

// Makes sure libfpv's essentials survived edits made in the Configurator
// (another motor protocol would leave the motors stopped), then reboots
// Betaflight if it has to, or always with `reboot` (a quad's EEPROM was just
// switched to or created).
void SimServer::ensureEssentials(const DronePreset &p, bool reboot) {
    reboot |= syncVideoSystem(p);
    if (cliRun("get motor_pwm_protocol").find("= PWM") == std::string::npos) {
        FPV_LOGW("motor protocol was changed away from PWM; restoring PWM");
        cliRun("set motor_pwm_protocol = PWM\nsave noreboot");
    }
    // Synced PWM makes Betaflight raise pid_process_denom to run the PID loop
    // no faster than the ESCs' 480 Hz, and it keeps that denominator: put
    // both back (an EEPROM made while synced PWM was the default, or a
    // Configurator edit). Takes effect after the reboot below.
    if (cliRun("get use_unsynced_pwm").find("= ON") == std::string::npos) {
        FPV_LOGW("unsynced PWM was off; turning it back on");
        std::string denom = cliRun("get pid_process_denom");
        size_t def = denom.find("Default value: ");
        std::string fix = "set use_unsynced_pwm = ON\n";
        if (def != std::string::npos) fix += "set pid_process_denom = " + std::to_string(std::atoi(denom.c_str() + def + 15)) + "\n";
        cliRun(fix + "save noreboot");
        reboot = true;
    }

    // the simulated GPS (D22). Features only take effect at boot.
    if (!bfb_gps_configured()) {
        FPV_LOGI("enabling Betaflight's GPS feature");
        cliRun("feature GPS\nsave noreboot");
    }

    std::string done;
    readTextFile(upgradeMarker(m_activeEeprom), done);
    size_t next = 0;  // first upgrade not applied yet
    for (size_t i = 0; i < std::size(kEepromUpgrades); i++)
        if (done == kEepromUpgrades[i]) next = i + 1;
    if (next == std::size(kEepromUpgrades)) {
        rebootIfNeeded(reboot);
        return;
    }
    std::string aux = cliRun("aux");
    std::string script;
    if (next == 0) {
        // turtle mode (D21): the AUX4 switch and the crash flip cut-off
        script += betaflightSwitchMigration(aux, {kTurtleSwitch});
        std::string out = cliRun("get crashflip_rate");
        size_t rate = out.find("crashflip_rate = ");
        if (rate != std::string::npos && std::atoi(out.c_str() + rate + 17) == 0) script += "set crashflip_rate = 30\n";
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
        std::string out = cliRun("get osd_custom_msg0_pos");
        size_t pos = out.find("osd_custom_msg0_pos = ");
        if (pos == std::string::npos || !(std::atoi(out.c_str() + pos + 22) & 0x800)) script += betaflightBombOsdLine();
    }
#endif
    if (next <= upgradeIndex("crashflip-tune-5")) {
        // a quad whose own tune sets crashflip_rate (the Air65 II: 0, its
        // real firmware's full turtle power, D21) gets it on an EEPROM made
        // before the tune had it
        std::istringstream tune(p.betaflightDiff);
        for (std::string line; std::getline(tune, line);)
            if (line.rfind("set crashflip_rate", 0) == 0) script += line + "\n";
    }
    if (!script.empty()) {
        FPV_LOGI("upgrading Betaflight configuration for new switches and OSD elements");
        cliRun(script + "save noreboot");
    }
    writeFileAtomic(upgradeMarker(m_activeEeprom), kEepromUpgrade);
    rebootIfNeeded(reboot);
}

// Activates the preset's EEPROM (one virtual flight controller per quad).
void SimServer::activateEepromFor(const DronePreset &p) {
    std::string path = eepromPathFor(p);
    if (path == m_activeEeprom) return;
    m_activeEeprom = path;
    int existed = bfb_switch_eeprom(path.c_str());
    if (existed == 0) applyFirstRunConfig(p);
    // reading an EEPROM only reloads the PID and rate profiles: gyro filters,
    // the mixer and features are set up at boot and would stay the previous
    // quad's
    ensureEssentials(p, true);
    warmUpBetaflight();  // reading an EEPROM suspends RX briefly
}

// Steps the parked simulation until Betaflight's boot-time sensor
// calibration is done (a few simulated seconds, ~50 ms of wall time), so a
// freshly spawned drone arms immediately.
void SimServer::warmUpBetaflight() {
    const float dt = 1.0f / std::max(200.0f, m_cfg.simRateHz);
    float idle[kChannelCount];
    setIdleChannels(idle);
    double t0 = wallSeconds();
    int steps = 0;
    // the flag only appears once the scheduler ran, so always run 0.5 s first
    while ((steps * dt < 0.5f || bfb_settling()) && steps * dt < 10.0f) {
        m_flight.step(dt, idle, false);
        steps++;
    }
    FPV_LOGD("Betaflight warm-up: %.2f s simulated in %.0f ms", steps * dt, (wallSeconds() - t0) * 1000.0);
}

// --- lifecycle ---------------------------------------------------------------------

bool SimServer::start(const ServerConfig &cfg, std::string &error) {
    (void)error;
    m_cfg = cfg;
    const DronePreset &preset = m_cfg.preset;
    // Betaflight is initialised once per process and survives stop().
    if (!m_flight.betaflightReady()) {
        bfb_ws_set_base_port(cfg.configuratorPort - 1);
        bfb_ws_set_enabled(cfg.configurator);
        std::string eeprom = eepromPathFor(preset);
        std::error_code ec;
        bool fresh = !fs::exists(pathFromUtf8(eeprom), ec);
        // Betaflight's init creates a missing EEPROM with its own defaults,
        // so freshness is decided before.
        m_flight.initBetaflight(eeprom, preset);
        m_activeEeprom = eeprom;
        if (fresh) applyFirstRunConfig(preset);
        ensureEssentials(preset, fresh);  // the first-run tune is saved without a reboot
        m_flight.applyPreset(preset);
        warmUpBetaflight();
    } else {
        bfb_ws_set_enabled(cfg.configurator);
        if (cfg.configurator) {
            if (bfb_ws_base_port() != cfg.configuratorPort - 1) {
                FPV_LOGW("configurator port cannot change after the first init in a process; keeping %d",
                         bfb_ws_base_port() + 1);
            }
            SimITL::Sim::getInstance().startServices();
        }
        activateEepromFor(preset);
        m_flight.applyPreset(preset);
    }

    RunnerConfig rc;
    rc.threaded = cfg.threaded;
    rc.simRateHz = cfg.simRateHz;
    rc.autoPauseSeconds = cfg.autoPauseSeconds;
    m_runner.configure(rc);
    m_runner.start();
    return true;
}

bool SimServer::startInput(std::string &error) {
    // SDL is initialised and polled on the simulation thread.
    return m_runner.call([&] { return m_input.startDevices(error); });
}

void SimServer::stop() {
    m_runner.call([&] {
        if (m_flight.active()) m_flight.despawn();
        if (bfb_config_dirty()) bfb_write_eeprom();
        m_input.stopDevices();
    });
    m_runner.stop();
    SimITL::Sim::getInstance().stop();
}

void SimServer::stopForExit() {
    // stop our threads before static destructors run, persist settings,
    // never block for long
    m_runner.callWithTimeout([&] {
        if (bfb_config_dirty()) bfb_write_eeprom();
        m_input.stopDevices();
    }, 1.0);
    m_runner.stop();
    SimITL::Sim::getInstance().stop();
}

// --- Betaflight configuration ------------------------------------------------------

CliResult SimServer::cli(const std::string &script, std::string &output) {
    int rc = BFB_CLI_ERROR;
    m_runner.call([&] {
        char *out = nullptr;
        rc = bfb_cli_run(script.c_str(), &out);
        output = out ? out : "";
        bfb_free(out);
    });
    return rc == BFB_CLI_OK ? CliResult::Ok : rc == BFB_CLI_BUSY ? CliResult::Busy : CliResult::Error;
}

void SimServer::setDefaultDiff(const std::string &diff) {
    m_runner.call([&] { m_cfg.defaultDiff = diff; });
}

void SimServer::selectPreset(const DronePreset &p) {
    m_runner.call([&] {
        activateEepromFor(p);
        // the same quad with another camera
        if (syncVideoSystem(p)) {
            bfb_reboot();
            warmUpBetaflight();
        }
        m_flight.applyPreset(p);
    });
}

void SimServer::resetEeproms(bool all, const DronePreset &p) {
    m_runner.call([&] {
        // keep nothing pending that would be written back to the old file
        if (bfb_config_dirty()) bfb_write_eeprom();
        std::error_code ec;
        if (all) {
            std::vector<fs::path> files;
            for (auto &e : fs::directory_iterator(pathFromUtf8(m_cfg.eepromDir), ec)) files.push_back(e.path());
            for (auto &f : files) fs::remove(f, ec);
        } else {
            std::string path = eepromPathFor(p);
            fs::remove(pathFromUtf8(path), ec);
            fs::remove(upgradeMarker(path), ec);
        }
        m_activeEeprom.clear();
        activateEepromFor(p);
        m_flight.applyPreset(p);
    });
}

void SimServer::finishImport(const DronePreset &p) {
    m_runner.call([&] {
        // features (air mode) only take effect at boot, and Betaflight does
        // not flag them as needing one; an import's `defaults` resets the
        // video system: it follows the camera again
        bool synced = syncVideoSystem(p);
        if (synced || bfb_reboot_required() != 0 || bfb_features_pending() != 0) {
            bfb_reboot();
            warmUpBetaflight();
        }
    });
}

void SimServer::reboot() {
    m_runner.call([] { bfb_reboot(); });
}

std::vector<std::string> SimServer::armingDisableFlagNames() {
    std::vector<std::string> names;
    for (int i = 0; i < 32; i++) names.push_back(bfb_arming_disable_flag_name(i));
    return names;
}

ConfiguratorStatus SimServer::configurator() {
    ConfiguratorStatus s;
    s.port = bfb_ws_base_port() + 1;
    s.clientCount = bfb_ws_client_count();
    return s;
}

// --- the drone --------------------------------------------------------------------------

void SimServer::spawn(const DronePreset &preset, Vec3 pos, Quat rot, Vec3 pilot) {
    m_runner.call([&] {
        activateEepromFor(preset);
        m_flight.setCargo({});  // a new drone carries nothing
        m_flight.spawn(preset, {0, 0, 0}, rot, pilot);
        m_flight.teleport(pos + rotate(rot, Vec3{0, m_flight.restHeight() + 0.002f, 0}), rot);
    });
}

void SimServer::despawn() {
    m_runner.call([&] { m_flight.despawn(); });
}

void SimServer::reset(Vec3 pos, Quat rot) {
    m_runner.call([&] {
        m_flight.teleport(pos + rotate(rot, Vec3{0, m_flight.restHeight() + 0.002f, 0}), rot);
        m_flight.resetFlightTime();  // a reset starts a new flight
    });
}

void SimServer::repair() {
    m_runner.call([&] { m_flight.repair(); });
}

void SimServer::damageProps(int motor, float amount) {
    m_runner.call([&] { m_flight.damageProps(motor, amount); });
}

void SimServer::setPilotPosition(Vec3 pos) {
    m_runner.call([&] { m_flight.setPilotPosition(pos); });
}

void SimServer::push(Vec3 velocity, bool replace) {
    m_runner.call([&] { m_flight.push(velocity, replace); });
}

void SimServer::setCargo(const Cargo &cargo) {
    m_runner.call([&] { m_flight.setCargo(cargo); });
}

void SimServer::setPortals(const std::vector<PortalLink> &links) { m_flight.setPortals(links); }

void SimServer::setHome(bool set, Vec3 home) { m_flight.setHome(set ? &home : nullptr); }

void SimServer::setWind(const WindState &state) { m_flight.setWind(state); }

void SimServer::addBlast(Vec3 pos, float radius) { m_flight.addBlast(pos, radius); }

void SimServer::setSimOptions(bool batteryDrain, bool damage, const RcLinkConfig &rcLink) {
    m_flight.setBatteryDrain(batteryDrain);
    m_flight.setDamage(damage);
    m_flight.setRcLinkConfig(rcLink);
}

void SimServer::setHostObstruction(const HostObstruction &o) {
    m_flight.setHostVideoObstruction(o.videoLossDb, o.videoK);
    m_flight.setHostRcObstruction(o.rcLossDb);
}

// --- the collision world --------------------------------------------------------------------

void SimServer::worldSetMaterial(uint8_t id, const Material &m) {
    if (!m_worldShared) m_world->setMaterial(id, m);
}

void SimServer::worldLoadStatic(const std::vector<Triangle> &tris) {
    if (!m_worldShared) m_world->loadStatic(tris);
}

void SimServer::worldCreateObject(uint32_t id, const std::vector<Triangle> &localTris, const RigidTransform &t,
                                  double simTime) {
    if (!m_worldShared) m_world->createObject(localTris, t, simTime, id);
}

void SimServer::worldMoveObject(uint32_t id, const RigidTransform &t, double simTime) {
    if (!m_worldShared) m_world->moveObject(id, t, simTime);
}

void SimServer::worldDeleteObject(uint32_t id) {
    if (!m_worldShared) m_world->deleteObject(id);
}

void SimServer::worldSetProxyPlanes(const std::vector<Contact> &planes) {
    if (!m_worldShared) m_world->setProxyPlanes(planes);
}

// --- the warhead ----------------------------------------------------------------------------

#if defined(LIBFPV_KAMIKAZE)
void SimServer::setWarhead(const WarheadConfig &config, bool installed, float payloadKg, float payloadDrop) {
    m_flight.warhead().setConfig(config);
    m_runner.call([&] {
        m_flight.setBombInstalled(installed);
        m_flight.setPayload(payloadKg, payloadDrop);
    });
}

bool SimServer::pollDetonation(DetonationEvent &out) { return m_flight.warhead().pollDetonation(out); }

bool SimServer::detonate(bool evenIfSafe) {
    return m_runner.call([&] { return m_flight.detonate(evenIfSafe); });
}
#endif

std::unique_ptr<Server> createLocalServer(ChannelSource &input, CollisionWorld *world) {
    return std::make_unique<SimServer>(input, world);
}

}  // namespace fpv
