// libfpv - collision world: static triangles, moving surface objects and
// temporary planes from the host raycast callback. Internal units (metres,
// libfpv frame).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "fpv_math.h"

namespace fpv {

struct Material {
    float restitution = 0.2f;
    float friction = 0.3f;  // smooth floors; rough ground is about 0.6 (D38)
    uint32_t flags = 0;
};

enum : uint32_t {
    kMatKillsProps = 1u << 0,
    kMatNoCollision = 1u << 1,
    kMatTransparentToVideo = 1u << 2,
};

struct Triangle {
    Vec3 a, b, c;
    uint8_t material = 0;
};

// Bounding volume hierarchy over a triangle soup.
class TriangleBVH {
public:
    void build(std::vector<Triangle> tris);
    bool empty() const { return m_tris.empty(); }
    size_t size() const { return m_tris.size(); }

    template <typename F>
    void queryAabb(Vec3 lo, Vec3 hi, F &&fn) const;  // fn(const Triangle&)

    // Closest hit along a ray within maxDist. Returns false if none.
    bool raycast(Vec3 origin, Vec3 dir, float maxDist, float &outT, Vec3 &outNormal, uint8_t &outMat,
                 uint32_t materialSkipMask, const Material *materials) const;

private:
    struct Node {
        Vec3 lo, hi;
        uint32_t left = 0;   // child index, or first triangle for leaves
        uint32_t count = 0;  // > 0 for leaves
    };
    std::vector<Node> m_nodes;
    std::vector<Triangle> m_tris;
    void buildNode(uint32_t node, std::vector<uint32_t> &idx, const std::vector<Vec3> &centroids, uint32_t begin,
                   uint32_t end, const std::vector<Triangle> &src);
};

struct Contact {
    Vec3 point;          // on the surface
    Vec3 normal;         // from surface towards the query centre
    float depth = 0;     // penetration
    Vec3 surfaceVel;     // velocity of the surface at the point
    Vec3 surfaceAngVel;  // angular velocity of the surface
    uint8_t material = 0;
    uint32_t objectId = 0;
    // A raycast hit seen through a portal, brought back to this side: it is
    // where it seems, even inside a portal's hole.
    bool throughPortal = false;
};

struct RigidTransform {
    Vec3 pos;
    Quat rot;
};

class CollisionWorld {
public:
    CollisionWorld();

    // --- writer side (API thread) -------------------------------------
    void setMaterial(uint8_t id, const Material &m);
    void loadStatic(std::vector<Triangle> tris);
    // `id` 0 takes the next free id; a mirror (the simulation server's copy)
    // passes the id the client's world gave.
    uint32_t createObject(std::vector<Triangle> localTris, const RigidTransform &t, double simTime, uint32_t id = 0);
    // The pose is the host's for `lead` s past simTime, or, if the host
    // calls things the other way round, for `leadAlt`: a host that moves its
    // surfaces before it steps or ticks gives the poses for the end of that
    // step or frame (lead), one that moves them after, for the time already
    // reached (leadAlt). Each object finds out which from how smoothly its
    // moves line up with either (D39).
    bool moveObject(uint32_t id, const RigidTransform &t, double simTime, double lead = 0, double leadAlt = 0);
    bool deleteObject(uint32_t id);
    bool hasObject(uint32_t id) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_objects.count(id) > 0;
    }
    // Planes from the host raycast callback: (point, normal, material).
    void setProxyPlanes(std::vector<Contact> planes);

    // --- reader side (simulation thread) ------------------------------
    // Collects contacts for a sphere at time simTime. Returns count added.
    size_t sphereContacts(Vec3 centre, float radius, double simTime, std::vector<Contact> &out) const;
    bool raycast(Vec3 origin, Vec3 dir, float maxDist, double simTime, float &outT, Vec3 &outNormal,
                 uint8_t &outMat, uint32_t &outObject, bool forVideo = false) const;
    Material material(uint8_t id) const { return m_materials[id]; }

    // --- debug view (the overlay's collision view; mutex() held) ------
    struct DebugTriangle {
        Vec3 a, b, c;  // world
        uint8_t material = 0;
        uint32_t objectId = 0;  // 0 = static
    };
    // Appends the triangles that may touch the box lo..hi, objects at their
    // pose for simTime, up to `limit` of them. Returns false if it stopped
    // at the limit.
    bool debugTriangles(Vec3 lo, Vec3 hi, double simTime, size_t limit, std::vector<DebugTriangle> &out) const;
    // The raycast callback's planes (setProxyPlanes), as they are now.
    const std::vector<Contact> &proxyPlanes() const { return m_proxyPlanes; }

    std::mutex &mutex() const { return m_mutex; }
    size_t staticTriangleCount() const { return m_static ? m_static->size() : 0; }

private:
    // The host's moves, newest last: when each came and the pose it gave.
    struct MoveSample {
        double t, tAlt;  // when it is for: by `lead`, by `leadAlt`
        Vec3 pos;
        Quat rot;
        double at(bool alt) const { return alt ? tAlt : t; }
    };
    // Where an object is at a time, and how fast it moves there.
    struct ObjectMotion {
        RigidTransform pose;
        Vec3 rel;  // pose.pos from the object's anchor, before rounding
        Vec3 linVel, angVel;  // world, at pose.pos
        bool moving = false;
    };
    struct Object {
        TriangleBVH bvh;
        Vec3 localLo, localHi;
        float boundRadius = 0;  // around local origin
        // The surface follows a smooth path through the host's moves (D39):
        // the motion fitted to the recent moves (fitPos/fitRot at t0, moving
        // at fitVel/fitAngVel until holdAt), plus what was left of the
        // previous path when the last move came (errPos/errVel,
        // errRot/errAngVel at blendFrom), faded out over kBlendTime with a
        // velocity that stays continuous.
        // Positions are kept from the anchor (a recent pose): far from the
        // origin a float has 0.24 mm steps at 2.5 km, and rounding the path
        // there at every move made a slow deck carry a quad a third too far.
        Vec3 anchor;
        std::vector<MoveSample> moves;  // the last kOrderMoves, positions from the anchor
        bool alt = false;               // the moves' times are by leadAlt
        double t0 = 0, holdAt = 0;  // the fitted line's time; it stops there
        double blendFrom = 0;       // when the blend began (the last move's arrival)
        Vec3 fitPos, fitVel, fitAngVel;
        Quat fitRot;
        Vec3 errPos, errVel, errRot, errAngVel;
        // the motion at the last time asked (every sphere of a substep asks
        // for the same); under the world's mutex
        mutable double cacheTime = -1e300;
        mutable ObjectMotion cache;
    };
    ObjectMotion objectMotion(const Object &o, double simTime) const;
    RigidTransform objectPose(const Object &o, double simTime) const { return objectMotion(o, simTime).pose; }

    mutable std::mutex m_mutex;
    Material m_materials[256];
    std::shared_ptr<TriangleBVH> m_static;
    std::unordered_map<uint32_t, std::unique_ptr<Object>> m_objects;
    uint32_t m_nextId = 1;
    std::vector<Contact> m_proxyPlanes;
    // the host moves its surfaces after it steps or ticks (moveObject's
    // leadAlt), and the tally that says so
    bool m_alt = false;
    float m_altEvidence = 0;
};

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection).
Vec3 closestPointOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c);

template <typename F>
void TriangleBVH::queryAabb(Vec3 lo, Vec3 hi, F &&fn) const {
    if (m_nodes.empty()) return;
    uint32_t stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp) {
        const Node &n = m_nodes[stack[--sp]];
        if (n.hi.x < lo.x || n.lo.x > hi.x || n.hi.y < lo.y || n.lo.y > hi.y || n.hi.z < lo.z || n.lo.z > hi.z)
            continue;
        if (n.count) {
            for (uint32_t i = 0; i < n.count; i++) fn(m_tris[n.left + i]);
        } else if (sp < 62) {
            stack[sp++] = n.left;
            stack[sp++] = n.left + 1;
        }
    }
}

}  // namespace fpv
