// libfpv - the simulation server itself: Betaflight, SimITL's flight model,
// libfpv's rigid body and the real-time loop, behind the Server interface.
// Default builds create it in the game's process; split builds run it in
// libfpv-server (docs/DECISIONS.md, D37).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <memory>
#include <string>

#include "input/channels.h"
#include "sim/flight.h"
#include "sim/runner.h"
#include "simapi/server.h"
#include "world/collision.h"

namespace fpv {

class SimServer final : public Server {
public:
    // `input` gives the channels each step. With `sharedWorld` the server
    // collides against the client's own collision world (same process) and
    // ignores the world* calls; without it, it keeps its own copy.
    SimServer(ChannelSource &input, CollisionWorld *sharedWorld);
    ~SimServer() override;

    bool start(const ServerConfig &cfg, std::string &error) override;
    void stop() override;
    void stopForExit() override;
    bool startInput(std::string &error) override;

    CliResult cli(const std::string &script, std::string &output) override;
    void setDefaultDiff(const std::string &diff) override;
    void selectPreset(const DronePreset &preset) override;
    void resetEeproms(bool all, const DronePreset &preset) override;
    void finishImport(const DronePreset &preset) override;
    void reboot() override;
    std::vector<std::string> armingDisableFlagNames() override;
    ConfiguratorStatus configurator() override;

    void spawn(const DronePreset &preset, Vec3 pos, Quat rot, Vec3 pilot) override;
    void despawn() override;
    void reset(Vec3 pos, Quat rot) override;
    void repair() override;
    void damageProps(int motor, float amount) override;
    void setPilotPosition(Vec3 pos) override;
    void push(Vec3 velocity, bool replace) override;
    void setCargo(const Cargo &cargo) override;
    void setPortals(const std::vector<PortalLink> &links) override;
    void setHome(bool set, Vec3 home) override;
    void setWind(const WindState &state) override;
    void addBlast(Vec3 pos, float radius) override;
    void setSimOptions(bool batteryDrain, bool damage, const RcLinkConfig &rcLink) override;
    void setHostObstruction(const HostObstruction &o) override;

    void worldSetMaterial(uint8_t id, const Material &m) override;
    void worldLoadStatic(const std::vector<Triangle> &tris) override;
    void worldCreateObject(uint32_t id, const std::vector<Triangle> &localTris, const RigidTransform &t,
                           double simTime) override;
    void worldMoveObject(uint32_t id, const RigidTransform &t, double simTime) override;
    void worldDeleteObject(uint32_t id) override;
    void worldSetProxyPlanes(const std::vector<Contact> &planes) override;

    bool threaded() override { return m_runner.threaded(); }
    float stepSeconds() override { return m_runner.stepSeconds(); }
    double simTime() override { return m_flight.simTime(); }
    void setPaused(bool paused) override { m_runner.setPaused(paused); }
    bool paused() override { return m_runner.paused(); }
    bool autoPaused() override { return m_runner.autoPaused(); }
    void setTimeScale(float scale) override { m_runner.setTimeScale(scale); }
    void heartbeat() override { m_runner.heartbeat(); }
    void pump() override { m_runner.pump(); }
    void stepManual(float seconds) override { m_runner.stepManual(seconds); }

    void snapshots(FlightSnapshot &prev, double &prevWall, FlightSnapshot &cur, double &curWall) override {
        m_runner.snapshots(prev, prevWall, cur, curWall);
    }
    void takeImpacts(std::vector<ImpactEvent> &out) override { m_runner.takeImpacts(out); }

#if defined(LIBFPV_KAMIKAZE)
    void setWarhead(const WarheadConfig &config, bool installed, float payloadKg, float payloadDrop) override;
    bool pollDetonation(DetonationEvent &out) override;
    bool detonate(bool evenIfSafe) override;
#endif

private:
    // Betaflight's configuration, all on the simulation thread
    std::string eepromPathFor(const DronePreset &p) const;
    void applyFirstRunConfig(const DronePreset &p);
    bool syncVideoSystem(const DronePreset &p);
    void ensureEssentials(const DronePreset &p, bool reboot);
    void activateEepromFor(const DronePreset &p);
    void warmUpBetaflight();
    std::string cliRun(const std::string &script);  // the output; simulation thread

    ChannelSource &m_input;
    std::unique_ptr<CollisionWorld> m_ownWorld;  // without a shared one
    CollisionWorld *m_world;
    bool m_worldShared;
    Flight m_flight;
    Runner m_runner{m_flight, m_input};
    ServerConfig m_cfg;
    std::string m_activeEeprom;
};

}  // namespace fpv
