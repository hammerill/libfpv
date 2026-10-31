// libfpv - one drone: SimITL flight model + libfpv rigid body and collisions.
// Runs on the simulation thread only. Internal frame: X right, Y up,
// Z forward, metres (SimITL's frame).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "config/preset.h"
#include "fpv_math.h"
#include "input/input.h"
#include "network/packets.h"
#include "world/collision.h"
#include "world/wind.h"
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
    bool modeAngle = false, modeHorizon = false, turtle = false, osdDisabled = false;
    uint32_t armingDisableFlags = 0;
    float altitude = -1;
    Vec3 wind;  // the air's velocity at the drone
    float videoNoise = 0, videoBreakup = 0, videoSignal = 1;
    std::array<uint8_t, 16 * 30> osd{};
    uint32_t osdRevision = 0;
    std::array<float, kChannelCount> channels{};
#if defined(LIBFPV_KAMIKAZE)
    WarheadSnapshot warhead;
    float payloadMass = 0, totalMass = 0;  // kg
#endif
};

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
    void repair();
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
    void updateVideo();
    Vec3 updateWind(float dt);
    void recordImpact(const Contact &c, float speed, float impulse, int motor);
    Mat3 worldInvInertia() const;

    CollisionWorld *m_world = nullptr;
    bool m_bfReady = false;
    bool m_active = false;
    DronePreset m_preset;
    std::vector<Sphere> m_shape;

    // authoritative rigid body state
    Vec3 m_pos, m_vel, m_angVel;
    Quat m_rot;
    float m_propDamage[4] = {0, 0, 0, 0};
    double m_simTime = 0;
    double m_lastGroundContact = -1;
    double m_lastMotorStrike[5] = {-1, -1, -1, -1, -1};
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
    float m_mass = 0.5f;
    Vec3 m_invInertiaBody{550, 310, 550};
    uint32_t m_videoTick = 0;
    int m_yawReversed = -1;
    float m_videoOcclusion = 0;
#if defined(LIBFPV_KAMIKAZE)
    Warhead m_warhead;
    Vec3 m_impactVel;  // velocity before the current contacts resolved
    void onDetonated();
    DronePreset withPayload(const DronePreset &bare);
    void refreshPayload();  // re-applies the payload to the live physics
    DronePreset m_basePreset;                     // without the payload
    float m_payloadKg = -1, m_payloadDropM = -1;  // requested
    float m_payloadMass = 0;                      // effective
    Vec3 m_payloadPos;                            // body frame
#endif
};

}  // namespace fpv
