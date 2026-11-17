// libfpv - the simulation's state as the client sees it: the snapshot taken
// after every step, impacts and the cargo. Shared by the client and the
// simulation server (docs/DECISIONS.md, D37).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <array>
#include <cstdint>

#include "fpv_math.h"
#include "input/channels.h"
#include "rc/rc_link.h"
#if defined(LIBFPV_KAMIKAZE)
#include "simapi/kamikaze_state.h"
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
    float vtxDamage = 0;  // 0 = intact, 1 = what is left of it after hard hits, fire, blasts
    // the RC link (D36), when simulated
    bool rcLinkOn = false;
    RcLinkStatus rcLink;
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

}  // namespace fpv
