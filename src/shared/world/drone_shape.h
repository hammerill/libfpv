// libfpv - the drone's collision shape: the spheres the flight model collides
// with the world, built from the preset. Shared by the flight model (server)
// and the overlay's collision view (client), so both see the same shape.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <vector>

#include "config/preset.h"
#include "fpv_math.h"

namespace fpv {

struct ShapeSphere {
    Vec3 local;  // body frame, metres from the centre of mass
    float radius;
    int motor;  // -1 body, 0..3 motor bell (or its duct), kCargoMotorIndex the cargo
    bool fine = false;  // detail on a larger part: not sized for substeps
};

// The prop disc's half thickness: a contact within it of a prop's plane
// strikes the prop.
constexpr float kPropDiscHalfThickness = 0.012f;
// Detail spheres (ShapeSphere::fine): duct rims and prop tips, as rings of
// small spheres
constexpr float kRimSphereRadius = 0.004f;
constexpr int kRimSpheres = 8;
constexpr int kPropTipSpheres = 12;
constexpr float kPropNutHeight = 0.006f;  // motor top above the prop disc
// A hit faster than this (relative normal speed, m/s) damages the video
// transmitter: the antenna bends, the connector cracks. 1/kVtxHitScale of
// its health per m/s above it.
constexpr float kVtxHitSpeed = 5.0f, kVtxHitScale = 40.0f;

// The preset's own spheres (body, motors or ducts, prop tips, canopy) into
// `out`, which is cleared first. The flight model adds what the drone
// carries (the cargo) itself.
void buildDroneShape(const DronePreset &p, std::vector<ShapeSphere> &out);

}  // namespace fpv
