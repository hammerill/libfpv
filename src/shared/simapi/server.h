// libfpv - the simulation server: everything the client (the C API, the
// presentation, the overlay) asks of the simulation, Betaflight and SimITL.
// The client only ever reaches the simulation through this interface
// (docs/DECISIONS.md, D37). Default builds run the server in the game's
// process (SimServer, direct calls); split builds run it in libfpv-server
// and carry the same calls between the processes.
//
// Every argument and result is a plain value: no callbacks and no pointers
// into the other side, so each call can be carried as a message.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "config/preset.h"
#include "fpv_math.h"
#include "input/channels.h"
#include "rc/rc_link.h"
#include "simapi/portal.h"
#include "simapi/state.h"
#include "world/collision.h"
#include "world/wind.h"
#if defined(LIBFPV_KAMIKAZE)
#include "simapi/kamikaze_state.h"
#endif

namespace fpv {

// Betaflight's CLI, as the server reports it.
enum class CliResult { Ok = 0, Busy = 1, Error = 2 };  // Busy: the Configurator is in CLI mode

struct ServerConfig {
    std::string eepromDir;      // one Betaflight EEPROM per quad: <dir>/<preset file name>.eeprom
    DronePreset preset;         // the quad flown first
    std::string defaultDiff;    // fpv_set_default_betaflight_diff, applied to new EEPROMs
    bool configurator = true;   // Betaflight Configurator over WebSocket
    int configuratorPort = 5761;
    bool threaded = true;       // the real-time loop; false = manual stepping (fpv_step)
    float simRateHz = 1000.0f;
    float autoPauseSeconds = 0.5f;  // <= 0 disables
};

struct ConfiguratorStatus {
    int port = 0;
    int clientCount = 0;
};

// What the client tells the simulation about the game's own raycast (frame.cpp):
// the video and RC links' obstruction, dB lost; < 0 = none (no host raycast).
struct HostObstruction {
    float videoLossDb = -1.0f, videoK = 10.0f;
    float rcLossDb = -1.0f;
};

class Server {
public:
    virtual ~Server() = default;

    // --- lifecycle ------------------------------------------------------------
    // Boots Betaflight (once per server: it cannot be torn down) or resumes
    // it with the given quad, then starts the real-time loop. False when the
    // server cannot be reached (split builds: libfpv-server missing).
    virtual bool start(const ServerConfig &cfg, std::string &error) = 0;
    // fpv_global_terminate: deletes the drone, saves Betaflight's settings
    // and stops the loop. Betaflight stays booted for the next start().
    virtual void stop() = 0;
    // Process exit with libfpv still running: like stop(), but never waits
    // long for the simulation thread.
    virtual void stopForExit() = 0;
    // Starts the radio/gamepad devices of the channel source on the
    // simulation thread, where they are polled.
    virtual bool startInput(std::string &error) = 0;

    // --- Betaflight configuration ------------------------------------------------
    virtual CliResult cli(const std::string &script, std::string &output) = 0;
    virtual void setDefaultDiff(const std::string &diff) = 0;
    // Flies `preset` on its own EEPROM: switches to it (created with libfpv's
    // first-run setup if new), follows its camera's video system and
    // applies its physics.
    virtual void selectPreset(const DronePreset &preset) = 0;
    // Erases EEPROMs (every quad's with `all`, else `preset`'s) and flies
    // `preset` on a fresh one.
    virtual void resetEeproms(bool all, const DronePreset &preset) = 0;
    // After an imported diff: follows the camera's video system again and
    // reboots Betaflight if features or settings need it.
    virtual void finishImport(const DronePreset &preset) = 0;
    virtual void reboot() = 0;
    // Betaflight's names of the arming disable flags, by bit.
    virtual std::vector<std::string> armingDisableFlagNames() = 0;
    virtual ConfiguratorStatus configurator() = 0;

    // --- the drone ------------------------------------------------------------------
    // Spawns `preset` resting on its lowest point at pos (internal frame).
    virtual void spawn(const DronePreset &preset, Vec3 pos, Quat rot, Vec3 pilot) = 0;
    virtual void despawn() = 0;
    // Puts the drone back at pos and starts a new flight (the OSD timer).
    virtual void reset(Vec3 pos, Quat rot) = 0;
    virtual void repair() = 0;
    virtual void damageProps(int motor, float amount) = 0;
    virtual void setPilotPosition(Vec3 pos) = 0;
    virtual void push(Vec3 velocity, bool replace) = 0;
    virtual void setCargo(const Cargo &cargo) = 0;
    virtual void setPortals(const std::vector<PortalLink> &links) = 0;
    virtual void setHome(bool set, Vec3 home) = 0;
    virtual void setWind(const WindState &state) = 0;
    virtual void addBlast(Vec3 pos, float radius) = 0;
    virtual void setSimOptions(bool batteryDrain, bool damage, const RcLinkConfig &rcLink) = 0;
    virtual void setHostObstruction(const HostObstruction &o) = 0;

    // --- the collision world ------------------------------------------------------------
    // The client keeps its own copy for its rays (chase camera, wind,
    // hearing) and mirrors every change here, ids and all. A server in the
    // client's process shares the client's copy and ignores these.
    virtual void worldSetMaterial(uint8_t id, const Material &m) = 0;
    virtual void worldLoadStatic(const std::vector<Triangle> &tris) = 0;
    virtual void worldCreateObject(uint32_t id, const std::vector<Triangle> &localTris, const RigidTransform &t,
                                   double simTime) = 0;
    // `lead`, `leadAlt`: how far past simTime the pose is for (CollisionWorld::moveObject)
    virtual void worldMoveObject(uint32_t id, const RigidTransform &t, double simTime, double lead,
                                 double leadAlt) = 0;
    virtual void worldDeleteObject(uint32_t id) = 0;
    virtual void worldSetProxyPlanes(const std::vector<Contact> &planes) = 0;

    // --- time ---------------------------------------------------------------------------
    virtual bool threaded() = 0;
    virtual float stepSeconds() = 0;
    virtual double simTime() = 0;
    virtual void setPaused(bool paused) = 0;
    virtual bool paused() = 0;
    virtual bool autoPaused() = 0;
    virtual void setTimeScale(float scale) = 0;
    // The host's frames: a long gap auto-pauses the simulation.
    virtual void heartbeat() = 0;
    // Browsers: runs the simulation up to now (no thread). No-op elsewhere.
    virtual void pump() = 0;
    // Manual timing: advances the simulation by `seconds`.
    virtual void stepManual(float seconds) = 0;

    // --- state ---------------------------------------------------------------------------
    // The latest two snapshots and the wallSeconds() each was taken at.
    virtual void snapshots(FlightSnapshot &prev, double &prevWall, FlightSnapshot &cur, double &curWall) = 0;
    // Impacts since the last call, oldest first.
    virtual void takeImpacts(std::vector<ImpactEvent> &out) = 0;

#if defined(LIBFPV_KAMIKAZE)
    // --- the warhead ------------------------------------------------------------------------
    virtual void setWarhead(const WarheadConfig &config, bool installed, float payloadKg, float payloadDrop) = 0;
    virtual bool pollDetonation(DetonationEvent &out) = 0;
    virtual bool detonate(bool evenIfSafe) = 0;
#endif
};

// The server of default builds, in the client's process: steps the
// simulation on its own thread with channels from `input`, colliding against
// the client's own collision world.
std::unique_ptr<Server> createLocalServer(ChannelSource &input, CollisionWorld *world);

// The server of split builds: starts libfpv-server and carries every call
// to it; the simulation asks `input` for each step's channels through it.
std::unique_ptr<Server> createRemoteServer(ChannelSource &input);

}  // namespace fpv
