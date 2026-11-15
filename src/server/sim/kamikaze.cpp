// libfpv - kamikaze builds: the warhead (DECISIONS D24).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "sim/kamikaze.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "util/log.h"

namespace fpv {

namespace {
// Betaflight's mode ranges use 1700..2100 for "on"; same threshold here.
bool switchOn(float v) { return v > 0.4f; }
}  // namespace

void Warhead::setConfig(const WarheadConfig &c) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config = c;
}

WarheadConfig Warhead::config() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_config;
}

bool Warhead::pollDetonation(DetonationEvent &out) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_events.empty()) return false;
    out = m_events.front();
    m_events.pop_front();
    return true;
}

void Warhead::setState(WarheadState s, double simTime) {
    if (m_snap.state == s) return;
    m_snap.state = s;
    m_snap.stateSince = simTime;
}

void Warhead::reset(double simTime) {
    setState(WarheadState::Safe, simTime);
    m_snap.armingRemaining = 0;
    m_snap.ignitionRemaining = 0;
    m_snap.needsCycle = true;
    m_snap.refusedUntil = -1;
    m_explodePrev = true;
}

void Warhead::setInstalled(bool on, double simTime) {
    if (m_snap.installed == on) return;
    m_snap.installed = on;
    // a new bomb, or none: safe, and a switch left on must be cycled first
    if (!detonated()) reset(simTime);
}

void Warhead::statusText(char *out, size_t size, double simTime, bool fcArmed) {
    const WarheadSnapshot &w = m_snap;
    bool blink = std::fmod(simTime, 1.0) < 0.6;  // slow enough for Betaflight's OSD refresh
    const char *text = "";
    char arming[16];
    if (!w.installed || !config().osd || detonated() || w.state == WarheadState::Igniting) {
        text = "";
    } else if (simTime < w.refusedUntil) {
        if (blink) text = "NOT ARMED";
    } else if (w.state == WarheadState::Safe) {
        if (!w.bombSwitch || (fcArmed && !w.needsCycle)) text = "BOMB_SAFE";
        else if (blink) text = "BOMB SW ON";
    } else if (w.state == WarheadState::Arming) {
        std::snprintf(arming, sizeof arming, "ARMING %d", (int)std::ceil(w.armingRemaining));
        text = arming;
    } else if (w.state == WarheadState::Armed) {
        text = "!ACTIVE!";
    }
    // centred in the width of the longest word, so it stays put as it changes
    const int kWidth = 10;
    int n = (int)std::strlen(text);
    int pad = n ? std::max(0, (kWidth - n) / 2) : 0;
    std::snprintf(out, size, "%*s%s", pad, "", text);
}

void Warhead::filterChannels(float ch[kChannelCount], bool fcArmed) {
    bool armSwitch = switchOn(ch[kAux1]);
    // without the bomb its switches do nothing (a wreck stays a wreck)
    bool present = m_snap.installed || detonated();
    m_snap.bombSwitch = present && switchOn(ch[kAux7]);
    m_snap.explodeSwitch = present && switchOn(ch[kAux8]);
    // Takeoff with a live (or arming) bomb is what the bomb switch guards
    // against: the motors only arm with it off. A destroyed drone never arms.
    m_snap.blocksArming = !fcArmed && (m_snap.bombSwitch || detonated());
    if (detonated()) m_armHold = true;  // the flight controller is gone
    if (m_snap.blocksArming && armSwitch) m_armHold = true;
    if (!armSwitch && !m_snap.blocksArming) m_armHold = false;
    if (m_armHold) ch[kAux1] = -1.0f;
}

bool Warhead::update(float dt, double simTime, const float ch[kChannelCount], bool fcArmed, Vec3 pos, Vec3 vel) {
    if (detonated() || !m_snap.installed) return false;
    bool bomb = switchOn(ch[kAux7]), explode = switchOn(ch[kAux8]);
    bool explodeEdge = explode && !m_explodePrev;
    m_explodePrev = explode;
    if (m_snap.state == WarheadState::Igniting) {
        // the fuse is lit: the switches change nothing any more
        m_snap.ignitionRemaining -= dt;
        if (m_snap.ignitionRemaining > 0) return false;
        DetonationEvent e = m_pending;
        e.simTime = simTime;
        // the blast is where the drone is now: an impact point moves with it
        e.position = m_pending.position + (pos - m_ignitedAt);
        fire(e);
        return true;
    }
    if (!bomb) m_snap.needsCycle = false;
    float delay = config().armingDelay;

    switch (m_snap.state) {
    case WarheadState::Safe:
        if (bomb && fcArmed && !m_snap.needsCycle) {
            if (delay > 0) {
                setState(WarheadState::Arming, simTime);
                m_snap.armingRemaining = delay;
            } else {
                setState(WarheadState::Armed, simTime);  // no arming delay: live at once
                FPV_LOGD("warhead live");
            }
        }
        break;
    case WarheadState::Arming:
        if (!bomb) {
            setState(WarheadState::Safe, simTime);
        } else if (!fcArmed) {
            // motors disarmed before it was live: abort, switch must cycle
            setState(WarheadState::Safe, simTime);
            m_snap.needsCycle = true;
        } else {
            m_snap.armingRemaining -= dt;
            if (m_snap.armingRemaining <= 0) {
                m_snap.armingRemaining = 0;
                setState(WarheadState::Armed, simTime);
                FPV_LOGD("warhead live");
            }
        }
        break;
    case WarheadState::Armed:
        // stays live when the motors disarm: only the bomb switch safes it
        if (!bomb) setState(WarheadState::Safe, simTime);
        break;
    case WarheadState::Detonated:
    case WarheadState::Igniting: break;
    }
    if (m_snap.state != WarheadState::Arming) m_snap.armingRemaining = 0;

    if (explodeEdge) {
        if (m_snap.state == WarheadState::Armed) {
            DetonationEvent e;
            e.simTime = simTime;
            e.cause = DetonationCause::Switch;
            e.position = pos;
            e.velocity = vel;
            return ignite(e, pos);
        }
        m_snap.refusedUntil = simTime + 1.5;
    }
    return false;
}

bool Warhead::impact(double simTime, Vec3 pos, Vec3 vel, Vec3 point, Vec3 normal, float speed, uint8_t material,
                     uint32_t objectId, int motor) {
    if (m_snap.state != WarheadState::Armed) return false;
    if (speed < config().minImpactSpeed) return false;
    DetonationEvent e;
    e.simTime = simTime;
    e.cause = DetonationCause::Impact;
    e.position = point;
    e.velocity = vel;
    e.normal = normal;
    e.material = material;
    e.objectId = objectId;
    e.motorIndex = motor;
    return ignite(e, pos);
}

bool Warhead::detonate(double simTime, Vec3 pos, Vec3 vel, bool evenIfSafe) {
    if (detonated() || !m_snap.installed) return false;
    if (m_snap.state != WarheadState::Armed && m_snap.state != WarheadState::Igniting && !evenIfSafe) return false;
    DetonationEvent e;
    e.simTime = simTime;
    e.cause = DetonationCause::Host;
    e.position = pos;
    e.velocity = vel;
    fire(e);
    return true;
}

bool Warhead::ignite(const DetonationEvent &e, Vec3 dronePos) {
    float delay = std::clamp(config().ignitionDelay, 0.0f, 1.0f);
    if (delay <= 0) {
        fire(e);
        return true;
    }
    setState(WarheadState::Igniting, e.simTime);
    m_snap.armingRemaining = 0;
    m_snap.ignitionRemaining = delay;
    m_pending = e;
    m_ignitedAt = dronePos;
    FPV_LOGD("warhead ignited, %.0f ms to the blast", delay * 1000);
    return false;
}

void Warhead::fire(const DetonationEvent &e) {
    setState(WarheadState::Detonated, e.simTime);
    m_snap.armingRemaining = 0;
    m_snap.ignitionRemaining = 0;
    m_snap.detonations++;
    static const char *kCause[] = {"impact", "explode switch", "host"};
    FPV_LOGI("warhead detonated (%s)", kCause[(int)e.cause]);
    std::lock_guard<std::mutex> lock(m_mutex);
    m_events.push_back(e);
    if (m_events.size() > 16) m_events.pop_front();
}

}  // namespace fpv
