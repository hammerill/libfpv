// libfpv - one drone: SimITL flight model + libfpv rigid body and collisions.
// Runs on the simulation thread only. Internal frame: X right, Y up,
// Z forward, metres (SimITL's frame).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "config/preset.h"
#include "fpv_math.h"
#include "input/channels.h"
#include "network/packets.h"
#include "simapi/portal.h"
#include "simapi/state.h"
#include "world/collision.h"
#include "world/drone_shape.h"
#include "world/wind.h"
#include "video/video_link.h"
#include "rc/rc_link.h"
#if defined(LIBFPV_KAMIKAZE)
#include "sim/kamikaze.h"
#endif

namespace fpv {

class Flight {
public:
    // One-time Betaflight initialisation (Betaflight cannot be torn down).
    void initBetaflight(const std::string &eepromPath, const DronePreset &parkedPreset);
    bool betaflightReady() const { return m_bfReady; }

    void setWorld(CollisionWorld *world) { m_world = world; }

    // Drone lifecycle. Without a drone, Betaflight keeps running "parked" so
    // the Configurator and the overlay keep working.
    void spawn(const DronePreset &preset, Vec3 pos, Quat rot, Vec3 pilotPos);
    void despawn();
    bool active() const { return m_active; }
    void teleport(Vec3 pos, Quat rot);
    // Velocity change (simulation thread): added, or set with `replace`.
    void push(Vec3 velocity, bool replace);
    // Linked portals (any thread); used from the next step.
    void setPortals(std::vector<PortalLink> links);
    // The drone's cargo (simulation thread): mass and inertia at once, also in flight.
    void setCargo(const Cargo &cargo);
    void repair();
    // Adds `amount` (0..1) to a prop's damage, motor -1 = all, and to the
    // video transmitter's (all of it for motor -1, a quarter for one prop).
    // Simulation thread; nothing with damage off.
    void damageProps(int motor, float amount);
    // The player's simulation options (any thread): the battery keeps the
    // charge it has (it still sags; a respawn brings a full one), props, motors and the VTX take no damage.
    void setBatteryDrain(bool on) { m_batteryDrain = on; }
    void setDamage(bool on) { m_damage = on; }
    // Zeroes Betaflight's total armed time, the OSD flight timer.
    void resetFlightTime();
    void applyPreset(const DronePreset &preset);
    const DronePreset &preset() const { return m_preset; }

    // One simulation step of dt seconds with Betaflight channel values
    // (AETR + AUX1..6, -1..1). `radioLost` puts Betaflight into failsafe.
    void step(float dt, const float channels[kChannelCount], bool radioLost);

    const FlightSnapshot &snapshot() const { return m_snap; }
    // Impacts accumulated since the last call.
    void takeImpacts(std::vector<ImpactEvent> &out);

    // GPS home point (internal frame). Without one, Betaflight sets home
    // where the drone arms. Any thread.
    void setHome(const Vec3 *home);

    // Wind (any thread): the latest state from the host's probes, and a
    // blast wave that starts at the next step.
    void setWind(WindState state);
    void addBlast(Vec3 pos, float radius);

#if defined(LIBFPV_KAMIKAZE)
    Warhead &warhead() { return m_warhead; }
    // Host-requested detonation (simulation thread).
    bool detonate(bool evenIfSafe);
    // The payload (warhead) carried under the frame: kg and metres below
    // the frame centre, < 0 = the preset's or automatic. Applies at once,
    // also in flight. Simulation thread.
    void setPayload(float massKg, float dropM);
    // Installs or removes the bomb (and its weight) at once. Simulation thread.
    void setBombInstalled(bool on);
#endif

    double simTime() const { return m_simTime; }
    Vec3 pilotPosition() const { return m_pilotPos; }
    void setPilotPosition(Vec3 p) { m_pilotPos = p; }  // simulation thread
    // The video link's obstruction as the game's own raycast sees it (host
    // thread, frame.cpp): the loss in dB and K factor, combined with what the
    // simulation's collision world blocks. lossDb < 0: none (no host raycast).
    void setHostVideoObstruction(float lossDb, float kFactor) {
        m_hostVideoK = kFactor;
        m_hostVideoLoss = lossDb;
    }
    // The same for the RC link (D36), at its band: loss in dB, < 0 = none.
    void setHostRcObstruction(float lossDb) { m_hostRcLoss = lossDb; }
    // The player's RC link setting (any thread), used from the next step.
    void setRcLinkConfig(const RcLinkConfig &cfg);
    // SimITL's receiver asks for each packet (simulation thread).
    bool rcPacketArrives();
    // Lowest point of the collision shape below the centre of mass (metres).
    float restHeight() const;

private:
    using Sphere = ShapeSphere;

    void buildShape();
    float shapeBottom() const;
    void stopMotors();
    void rcRespawnGrace();  // 1 s without RC packet loss
    void syncPropDirection();
    void resolveContacts(float dt);
    void propStrikes(float dt);
    void updateGroundEffect(float out[4]);
    void updateVideo(float dt);
    // the RC link around Betaflight's step (D36)
    void updateRcLink();
    void pushRcLinkStats();
    Vec3 updateWind(float dt);
    void recordImpact(const Contact &c, float speed, float impulse, int motor);
    // drops the contacts in a portal's hole from m_contacts (from `first` on)
    void dropPortalHoleContacts(size_t first);
    void passPortals(Vec3 before);  // a crossing since `before` takes the drone through
    bool hanging() const { return m_cargo.mass > 0 && m_cargo.line > 0; }
    Vec3 hookPoint() const;  // where the line hangs from (world)
    void stepCargo(float dt);  // a hanging cargo: its own flight, then the line
    DronePreset withExtraMass(const DronePreset &bare);
    void refreshMass();  // re-applies cargo and payload to the live physics
    Mat3 worldInvInertia() const;

    CollisionWorld *m_world = nullptr;
    bool m_bfReady = false;
    bool m_active = false;
    DronePreset m_preset;
    std::vector<Sphere> m_shape;

    // authoritative rigid body state
    Vec3 m_pos, m_vel, m_angVel;
    // What m_pos could not take of the moves so far (compensated summation):
    // far from the origin a float position only has 0.24 mm steps (2048 to
    // 4096 m), finer than a step at 1 kHz below about 0.12 m/s, so slow moves
    // were rounded away or up to whole steps.
    Vec3 m_posCarry;
    double m_contactTime = 0;  // when resolveContacts meets the world (the substep's end)
    void movePos(Vec3 d);
    Quat m_rot;
    float m_propDamage[4] = {0, 0, 0, 0};
    // The video transmitter (D34): bent antenna, cracked connector, a cooked
    // amplifier. Costs up to kVtxDamageLossDb of the link budget; repaired
    // with the drone.
    float m_vtxDamage = 0;
    void damageVtx(float amount);
    std::atomic<bool> m_batteryDrain{true}, m_damage{true};
    void keepBattery(double capacity);  // FPV_SIM_NO_BATTERY_DRAIN
    void keepMotors();   // FPV_SIM_NO_DAMAGE
    double m_simTime = 0;
    double m_lastGroundContact = -1;
    double m_lastMotorStrike[6] = {-1, -1, -1, -1, -1, -1};  // frame, motors 0..3, cargo
    Vec3 m_pilotPos;
    std::mutex m_homeMutex;
    bool m_hasHome = false;
    Vec3 m_home;
    std::mutex m_windMutex;
    WindState m_windIn;
    bool m_windChanged = false;
    std::vector<WindBlast> m_blastsIn;
    WindState m_wind;  // simulation thread
    WindClock m_windClock;
    std::vector<WindBlast> m_blasts;

    StateInput m_input{};
    std::vector<Contact> m_contacts;
    std::vector<ImpactEvent> m_impacts;
    FlightSnapshot m_snap;
    std::mutex m_portalMutex;
    std::vector<PortalLink> m_portalsIn;
    bool m_portalsChanged = false;
    std::vector<PortalLink> m_portals;  // simulation thread
    uint32_t m_teleports = 0, m_portalPasses = 0;
    int m_lastPortal = -1;
    Cargo m_cargo;
    // a hanging cargo's own body (world, simulation thread)
    Vec3 m_cargoPos, m_cargoVel;
    float m_lineNow = 0;  // the line's length now (the winch reels it to m_cargo.line)
    bool m_lineTaut = false;
    DronePreset m_basePreset;  // without the cargo and payload
    float m_mass = 0.5f;
    Vec3 m_invInertiaBody{550, 310, 550};
    uint32_t m_videoTick = 0;
    int m_yawReversed = -1;
    // the video link's obstruction probe (D34): a few rays per 20 ms cycle
    struct VideoProbe {
        bool centre = false;    // the direct ray is blocked
        float tNear = 0, tFar = 0;  // the obstacle's start from each end
        int offsets = 0;        // blocked paths through the first Fresnel zone
        float lossDb = 0;       // smoothed
        float kFactor = 10;     // smoothed
    } m_videoProbe;
    float m_videoTarget[2] = {0, 10};  // loss dB, K, from the last full cycle
    std::atomic<float> m_hostVideoLoss{-1.0f}, m_hostVideoK{10.0f};
    // the RC link (D36)
    std::mutex m_rcMutex;
    RcLinkConfig m_rcIn;
    bool m_rcChanged = true;
    RcLink m_rcLink;               // simulation thread
    float m_rcLossTarget = 0, m_rcLossDb = 0;  // obstruction, dB
    float m_rcPathGain = -60, m_rcK = 10, m_rcDoppler = 0;  // from the last step's geometry
    std::atomic<float> m_hostRcLoss{-1.0f};
    RcLinkInputs m_rcInputs;       // for this step's packets
    double m_rcLastPacket = -1;    // sim time of the last packet slot
    double m_rcImmuneUntil = -1;   // respawn grace: no packet lost before then
    // Respawn grace for the props: a repaired or moved drone takes no damage
    // before then, so what surrounded it before (the water it was repaired
    // in, a host's stale damage, a world object still moving to the new
    // spot) never breaks the new props.
    double m_damageGraceUntil = -1;
    bool canDamage() const { return m_damage && m_simTime >= m_damageGraceUntil; }
    bool m_rcPushed = false;       // Betaflight shows our link statistics
#if defined(LIBFPV_KAMIKAZE)
    Warhead m_warhead;
    Vec3 m_impactVel;  // velocity before the current contacts resolved
    void onDetonated();
    float m_payloadKg = -1, m_payloadDropM = -1;  // requested
    float m_payloadMass = 0;                      // effective
    Vec3 m_payloadPos;                            // body frame
#endif
};

}  // namespace fpv
