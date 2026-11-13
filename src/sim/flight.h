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
#include "input/input.h"
#include "network/packets.h"
#include "sim/portal.h"
#include "world/collision.h"
#include "world/wind.h"
#include "video/video_link.h"
#if defined(LIBFPV_KAMIKAZE)
#include "sim/kamikaze.h"
#endif

namespace fpv {

struct ImpactEvent {
    Vec3 point, normal;
    float speed = 0;     // m/s, relative normal speed
    float impulse = 0;   // N*s
    uint8_t material = 0;
    uint32_t objectId = 0;
    int motorIndex = -1;
};

struct MotorSnapshot {
    float rpm = 0, thrust = 0, current = 0, temperature = 0, command = 0, propDamage = 0;
    uint32_t flags = 0;
};

// Everything the host-facing side needs, copied out after every step.
struct FlightSnapshot {
    double simTime = 0;
    bool active = false;  // a drone exists
    Vec3 pos, vel, angVel;
    Quat rot;
    std::array<MotorSnapshot, 4> motors{};
    float batVoltage = 0, batVoltageSag = 0, batCurrent = 0, mahDrawn = 0;
    bool armed = false, failsafe = false, beeper = false, onGround = false, crashed = false;
    bool beeperSwitch = false;  // the beeper sounds for the pilot's switch (through the motors)
    bool modeAngle = false, modeHorizon = false, turtle = false, osdDisabled = false;
    uint32_t armingDisableFlags = 0;
    float altitude = -1;
    Vec3 wind;  // the air's velocity at the drone
    bool cargo = false;          // something is carried
    bool cargoHanging = false;   // on a line
    bool cargoLineTaut = false;  // the line pulls (it lifts or drags the cargo)
    Vec3 cargoPos, cargoVel;     // its centre, world
    float videoNoise = 0, videoBreakup = 0, videoSignal = 1;
    // the link budget (D34): mean received power, Rician K, fade rate
    float videoRxDbm = -40, videoK = 10, videoDoppler = 0;
    std::array<uint8_t, 16 * 30> osd{};
    uint32_t osdRevision = 0;
    std::array<float, kChannelCount> channels{};
    // Counts the jumps (portal passes, teleports): no interpolation across one.
    uint32_t teleports = 0;
    int lastPortal = -1;  // link * 2 + the side entered, of the last portal pass
    uint32_t portalPasses = 0;
#if defined(LIBFPV_KAMIKAZE)
    WarheadSnapshot warhead;
    float payloadMass = 0, totalMass = 0;  // kg
#endif
};

// Something the drone carries (fpv_drone_set_cargo), body frame.
struct Cargo {
    float mass = 0;   // kg, 0 = none
    Vec3 offset;      // its centre, metres from the drone's centre (on a line: where it is at the pickup)
    float radius = 0; // collision sphere, 0 = none
    float line = 0;   // > 0: hangs on a line this long (m) from under the frame, reeled in to it
};

// Impacts of the cargo's sphere report this motor index.
constexpr int kCargoMotorIndex = -2;

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
    // Adds `amount` (0..1) to a prop's damage, motor -1 = all. Simulation
    // thread; nothing with damage off.
    void damageProps(int motor, float amount);
    // The player's simulation options (any thread): the battery keeps the
    // charge it has (it still sags; a respawn brings a full one), props and motors take no damage.
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
    // Lowest point of the collision shape below the centre of mass (metres).
    float restHeight() const;

private:
    struct Sphere {
        Vec3 local;
        float radius;
        int motor;  // -1 body, 0..3 motor bell
    };

    void buildShape();
    float shapeBottom() const;
    void stopMotors();
    void syncPropDirection();
    void resolveContacts(float dt);
    void propStrikes(float dt);
    void updateGroundEffect(float out[4]);
    void updateVideo(float dt);
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
    void movePos(Vec3 d);
    Quat m_rot;
    float m_propDamage[4] = {0, 0, 0, 0};
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
