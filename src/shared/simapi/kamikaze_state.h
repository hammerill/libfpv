// libfpv - kamikaze builds (LIBFPV_KAMIKAZE): the warhead's state, settings
// and detonations as the client sees them. The warhead itself runs in the
// simulation (sim/kamikaze.h).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstdint>

#include "fpv_math.h"

namespace fpv {

// Igniting: the fuse is lit (impact or explode switch), the blast comes
// after the ignition delay; nothing stops it. Values match fpv_warhead_state.
enum class WarheadState { Safe = 0, Arming = 1, Armed = 2, Detonated = 3, Igniting = 4 };
enum class DetonationCause { Impact = 0, Switch = 1, Host = 2 };

struct WarheadConfig {
    float armingDelay = 0.0f;    // seconds from switch on (motors armed) to live
    float minImpactSpeed = 0.25f;  // m/s, the contact fuse threshold (0.1..30)
    float ignitionDelay = 0.15f;   // s from the boom command to the blast (0..1), 0 = at once
    bool osd = true;             // the state in Betaflight's custom message 1 element
};

// Internal frame (metres, X right, Y up, Z forward).
struct DetonationEvent {
    double simTime = 0;
    DetonationCause cause = DetonationCause::Impact;
    Vec3 position, velocity, normal;  // normal is zero unless it was an impact
    uint8_t material = 0;
    uint32_t objectId = 0;
    int motorIndex = -1;
};

struct WarheadSnapshot {
    bool installed = false;      // the drone carries the bomb (an overlay/host choice, per session)
    WarheadState state = WarheadState::Safe;
    float armingRemaining = 0;
    float ignitionRemaining = 0; // seconds to the blast, while Igniting
    bool bombSwitch = false, explodeSwitch = false;
    bool blocksArming = false;   // the motors cannot arm (bomb switch on, or destroyed)
    bool needsCycle = false;     // the bomb switch must go off before it can arm
    double refusedUntil = -1;    // explode switch flipped while not armed: "NOT ARMED" until then
    double stateSince = 0;       // sim time of the last state change
    uint32_t detonations = 0;
};

}  // namespace fpv
