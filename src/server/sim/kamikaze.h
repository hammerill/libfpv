// libfpv - kamikaze builds (LIBFPV_KAMIKAZE): the warhead carried by the
// drone. Bomb arm switch on AUX7, explode switch on AUX8. The flight side
// runs on the simulation thread; detonations are queued for the host.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "fpv_math.h"
#include "input/channels.h"
#include "simapi/kamikaze_state.h"

namespace fpv {

class Warhead {
public:
    // Simulation thread -------------------------------------------------
    // Before Betaflight sees the channels: holds AUX1 (arm) low while the
    // motors must not arm, until the pilot cycles the arm switch, like
    // Betaflight's own ARMSWITCH safety.
    void filterChannels(float ch[kChannelCount], bool fcArmed);
    // After the flight step: switch logic, arming countdown, explode switch,
    // the ignition's countdown. Returns true when it detonated in this step.
    bool update(float dt, double simTime, const float ch[kChannelCount], bool fcArmed, Vec3 pos, Vec3 vel);
    // A contact from the collision solver; lights a live warhead's fuse.
    // Returns true only when it detonated at once (no ignition delay).
    bool impact(double simTime, Vec3 pos, Vec3 vel, Vec3 point, Vec3 normal, float speed, uint8_t material,
                uint32_t objectId, int motor);
    // Host request (fpv_kamikaze_detonate): at once, also while igniting.
    // Returns false if not detonated.
    bool detonate(double simTime, Vec3 pos, Vec3 vel, bool evenIfSafe);
    // A new drone or a repaired one: fresh warhead, safe; the bomb switch
    // must be seen off before it can arm again.
    void reset(double simTime);
    // Installs or removes the bomb. Without it the bomb switches do nothing;
    // a destroyed drone stays destroyed until repaired.
    void setInstalled(bool on, double simTime);
    // The state line of the OSD ("BOMB_SAFE", "ARMING 3", "!ACTIVE!"...), "" when
    // there is nothing to show (no bomb, blink off, igniting, destroyed).
    void statusText(char *out, size_t size, double simTime, bool fcArmed);

    bool installed() const { return m_snap.installed; }
    bool detonated() const { return m_snap.state == WarheadState::Detonated; }
    bool igniting() const { return m_snap.state == WarheadState::Igniting; }
    const WarheadSnapshot &snapshot() const { return m_snap; }

    // Any thread ------------------------------------------------------------
    void setConfig(const WarheadConfig &c);
    WarheadConfig config();
    bool pollDetonation(DetonationEvent &out);

private:
    void setState(WarheadState s, double simTime);
    void fire(const DetonationEvent &e);
    // The boom command: the blast now (no delay) or after the ignition delay.
    // Returns true when it detonated now.
    bool ignite(const DetonationEvent &e, Vec3 dronePos);

    std::mutex m_mutex;  // config and the event queue
    WarheadConfig m_config;
    std::deque<DetonationEvent> m_events;

    WarheadSnapshot m_snap;
    bool m_armHold = false;      // AUX1 held low until the arm switch goes off
    bool m_explodePrev = true;   // edge detection; true = must be seen off first
    DetonationEvent m_pending;   // the blast the lit fuse leads to
    Vec3 m_ignitedAt;            // the drone's position when it was lit
};

}  // namespace fpv
