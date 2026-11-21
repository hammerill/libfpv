// libfpv - the drone's collision shape (drone_shape.h).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "world/drone_shape.h"

#include <algorithm>
#include <cmath>

namespace fpv {

// A sphere at the centre of mass and one under each motor bell. Their
// bottoms are level, so the drone rests on five points and ground friction
// can stop it from spinning. One more sphere on top of each motor (bell and
// prop nut, just above the prop disc): upside down the drone rests on those
// with its props clear of the ground, and in turtle mode it pivots on an arm
// like a real quad instead of rolling on the body. Then the canopy around
// the FPV camera: upside down the quad rests on it and the motor tops, and
// the lens can never get closer to a surface than the camera's near plane
// (ensureCameraClearance).
void buildDroneShape(const DronePreset &p, std::vector<ShapeSphere> &out) {
    out.clear();
    const CollisionSettings &c = p.collision;
    const bool ducted = p.physics.ductThrustGain > 0;
    out.push_back({{0, 0, 0}, c.bodyRadius, -1});
    for (int i = 0; i < 4; i++) {
        const Vec3 &m = p.physics.quadMotorPos[i];
        float bottom = -c.bodyRadius;  // level with the body's bottom
        float top = std::max(c.bodyRadius, m.y + kPropDiscHalfThickness + kPropNutHeight);
        if (ducted) {
            // A duct is a ring: a flat rim at its bottom and its top with an
            // edge the quad tips and lands on, round a core that keeps
            // walls out of it (D38)
            out.push_back({{m.x, 0.5f * (bottom + top), m.z}, c.motorRadius, i});
            float ring = c.motorRadius - kRimSphereRadius;
            for (float y : {bottom + kRimSphereRadius, top - kRimSphereRadius})
                for (int k = 0; k < kRimSpheres; k++) {
                    float a = 6.2831853f * k / kRimSpheres;
                    out.push_back({{m.x + ring * std::cos(a), y, m.z + ring * std::sin(a)}, kRimSphereRadius, i, true});
                }
            continue;
        }
        out.push_back({{m.x, std::min(m.y, bottom + c.motorRadius), m.z}, c.motorRadius, i});
        out.push_back({{m.x, top - c.motorRadius, m.z}, c.motorRadius, i});
        // the prop's tips: a quad landing tilted or tipping over touches
        // down on them first, as far out as they reach (D38)
        float ring = c.propRadius - kRimSphereRadius;
        for (int k = 0; k < kPropTipSpheres; k++) {
            float a = 6.2831853f * k / kPropTipSpheres;
            out.push_back({{m.x + ring * std::cos(a), m.y, m.z + ring * std::sin(a)}, kRimSphereRadius, i, true});
        }
    }
    out.push_back({c.canopyOffset, c.canopyRadius, -1});
}

}  // namespace fpv
