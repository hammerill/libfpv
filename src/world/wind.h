// libfpv - wind: the weather's wind made local (height, shelter behind
// obstacles, updrafts, enclosure), turbulence, air movers (rotor
// downwash, jets) and blast waves. Internal frame: X right, Y up, Z
// forward, metres, m/s.
//
// The host thread probes the world around the drone a few times a second
// (WindProbe) and hands the result to the simulation thread as a
// WindState; the simulation samples the air velocity from it at the
// centre of mass and at each motor hub, every step (sampleWind).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>

#include "fpv_math.h"

namespace fpv {

constexpr int kMaxWindSources = 32;
constexpr int kWindOctaves = 5;

struct WindSource {
    enum Type : uint32_t { Rotor = 0, Jet = 1 };
    Vec3 pos, dir;  // dir: unit, where the air is blown
    float speed = 0, radius = 1;
    uint32_t type = Rotor;
    float groundDist = -1;  // along dir to the ground (the wake turns there), < 0 = none near
};

struct WindBlast {
    Vec3 pos;
    float radius = 1;
    double start = 0;  // simulation time it went off
};

// What the simulation needs to evaluate the air anywhere near the drone.
struct WindState {
    bool on = false;
    Vec3 mean;               // local mean wind at the drone, m/s
    Vec3 convection;         // what carries the eddies along (the free wind, also over a sheltering wall)
    float sigma = 0;         // turbulence, along-wind standard deviation, m/s
    float lengthScale = 20;  // largest eddies, m
    std::vector<WindSource> sources;
};

// The turbulence field's own time: eddies are carried along by the mean
// wind (frozen turbulence) and slowly change shape. Integrated per step, so
// the field stays continuous when the mean wind changes.
struct WindClock {
    Vec3 drift;
    float evolve[kWindOctaves] = {};
    void advance(const WindState &s, float dt);
};

// The air velocity at p (m/s).
Vec3 sampleWind(const WindState &s, const WindClock &clock, const std::vector<WindBlast> &blasts, Vec3 p, double t);

// True when a blast no longer moves any air (to drop it).
bool blastOver(const WindBlast &b, double t);

// The host's weather wind in internal units.
struct WindAmbient {
    Vec3 velocity;          // at refHeight, m/s
    float refHeight = 10;   // m
    float turbulence = -1;  // < 0 automatic
    bool noShelter = false, enclosed = false;
};

// Host thread: how the surroundings change the weather's wind at the
// drone, from rays into the world. Smoothed over about half a second so
// passing a pole or flying through a doorway doesn't step the wind.
class WindProbe {
public:
    // A ray against the world: true on a hit, with distance and normal.
    using Raycast = std::function<bool(Vec3 origin, Vec3 dir, float maxDist, float &t, Vec3 &normal)>;

    void reset() { m_valid = false; }
    // Probes when due (about 10 times a second) and returns the state for
    // the simulation. `strength` scales the weather's wind (the player's
    // setting); the sources are kept as given.
    WindState update(float dt, Vec3 dronePos, const WindAmbient &ambient, float strength,
                     const std::vector<WindSource> &sources, const Raycast &ray);

    // For the overlay, from the last probes: wind lost to obstacles and
    // walls (0..1), how enclosed the drone is (0..1).
    float shelter() const { return m_valid ? std::max(0.0f, 1.0f - m_retained) : 0.0f; }
    float enclosure() const { return m_valid ? m_enclosed : 0.0f; }

private:
    struct Raw {
        float retained = 1;  // mean wind kept, < 0 = reversed (just behind a wall)
        float turbBoost = 0;
        float updraft = 0;   // fraction of the free wind turned upwards (windward faces)
        float enclosed = 0;  // 0 open .. 1 inside
        float hEff = 10;     // above the open ground around, m
        float hLocal = 10;   // above the surface right below, m
    };
    Raw measure(Vec3 p, Vec3 windDir, const Raycast &ray) const;

    bool m_valid = false;
    float m_timer = 0, m_sinceProbe = 0;
    float m_retained = 1, m_turbBoost = 0, m_updraft = 0, m_enclosed = 0, m_hEff = 10, m_hLocal = 10;
    Vec3 m_lastDir{0, 0, 1};
};

}  // namespace fpv
