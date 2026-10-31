// libfpv - collision world: static triangles, moving surface objects and
// temporary planes from the host raycast callback. Internal units (metres,
// libfpv frame).
// SPDX-License-Identifier: GPL-3.0-or-later
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
    float friction = 0.6f;
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
    uint32_t createObject(std::vector<Triangle> localTris, const RigidTransform &t, double simTime);
    bool moveObject(uint32_t id, const RigidTransform &t, double simTime);
    bool deleteObject(uint32_t id);
    // Planes from the host raycast callback: (point, normal, material).
    void setProxyPlanes(std::vector<Contact> planes);

    // --- reader side (simulation thread) ------------------------------
    // Collects contacts for a sphere at time simTime. Returns count added.
    size_t sphereContacts(Vec3 centre, float radius, double simTime, std::vector<Contact> &out) const;
    bool raycast(Vec3 origin, Vec3 dir, float maxDist, double simTime, float &outT, Vec3 &outNormal,
                 uint8_t &outMat, uint32_t &outObject, bool forVideo = false) const;
    Material material(uint8_t id) const { return m_materials[id]; }

    std::mutex &mutex() const { return m_mutex; }
    size_t staticTriangleCount() const { return m_static ? m_static->size() : 0; }

private:
    struct Object {
        TriangleBVH bvh;
        Vec3 localLo, localHi;
        float boundRadius = 0;   // around local origin
        RigidTransform from, to; // interpolated between moves
        double t0 = 0, duration = 0;
        Vec3 linVel, angVel;     // estimated from the last move
    };
    RigidTransform objectPose(const Object &o, double simTime) const;

    mutable std::mutex m_mutex;
    Material m_materials[256];
    std::shared_ptr<TriangleBVH> m_static;
    std::unordered_map<uint32_t, std::unique_ptr<Object>> m_objects;
    uint32_t m_nextId = 1;
    std::vector<Contact> m_proxyPlanes;
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
