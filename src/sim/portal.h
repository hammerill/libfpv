// libfpv - linked portals (fpv_portals_set): two openings in the world that
// are one. A body crossing one leaves the other with its pose, velocity and
// spin carried over, and the surface around an opening doesn't block it.
// Internal frame (metres, X right, Y up, Z forward).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cmath>
#include <vector>

#include "fpv_math.h"

namespace fpv {

struct Portal {
    Vec3 pos;   // centre of the opening, on the surface
    Quat rot;   // local +Z out of the surface (the side bodies enter from), +Y the opening's up
    float halfWidth = 0, halfHeight = 0;

    Vec3 toLocal(Vec3 p) const { return rotate(conj(rot), p - pos); }
    bool insideOpening(Vec3 local, float margin = 0) const {
        return std::fabs(local.x) < halfWidth - margin && std::fabs(local.y) < halfHeight - margin;
    }
};

// How deep behind an opening's surface its hole reaches: the wall it is on.
// Contacts in that box are with the wall around the hole, which a body in
// the opening passes through.
constexpr float kPortalHoleDepth = 0.6f;
constexpr float kPortalHoleFront = 0.05f;  // and this far in front (the surface's own contacts)

struct PortalLink {
    Portal a, b;
    const Portal &side(int s) const { return s ? b : a; }
};

// The rigid motion that takes what enters `from` out of `to`: in at the
// front of one, out at the front of the other, turned half way around the
// opening's up (so left stays left, as when walking through a doorway).
struct PortalTransform {
    Quat rot;
    Vec3 fromPos, toPos;
    Vec3 point(Vec3 p) const { return toPos + rotate(rot, p - fromPos); }
    Vec3 dir(Vec3 d) const { return rotate(rot, d); }
    Quat orient(Quat q) const { return normalize(rot * q); }
};

inline PortalTransform portalTransform(const Portal &from, const Portal &to) {
    const Quat halfTurn{0, 1, 0, 0};  // 180 degrees about local +Y
    PortalTransform t;
    t.rot = normalize(to.rot * halfTurn * conj(from.rot));
    t.fromPos = from.pos;
    t.toPos = to.pos;
    return t;
}

// The opening that point `p` lies in the hole of (behind the surface, within
// the wall), as link * 2 + side, or -1.
inline int portalHoleAt(const std::vector<PortalLink> &links, Vec3 p) {
    for (size_t i = 0; i < links.size(); i++) {
        for (int s = 0; s < 2; s++) {
            const Portal &o = links[i].side(s);
            Vec3 l = o.toLocal(p);
            if (l.z < kPortalHoleFront && l.z > -kPortalHoleDepth && o.insideOpening(l)) return (int)(i * 2 + s);
        }
    }
    return -1;
}

}  // namespace fpv
