// libfpv - collision world implementation.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "world/collision.h"

#include <algorithm>
#include <limits>

namespace fpv {

Vec3 closestPointOnTriangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    Vec3 bp = p - b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    Vec3 cp = p - c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

// Möller-Trumbore, two-sided.
static bool rayTriangle(Vec3 o, Vec3 d, const Triangle &t, float &outT) {
    Vec3 e1 = t.b - t.a, e2 = t.c - t.a;
    Vec3 p = cross(d, e2);
    float det = dot(e1, p);
    if (std::fabs(det) < 1e-14f) return false;
    float inv = 1.0f / det;
    Vec3 s = o - t.a;
    float u = dot(s, p) * inv;
    if (u < 0 || u > 1) return false;
    Vec3 q = cross(s, e1);
    float v = dot(d, q) * inv;
    if (v < 0 || u + v > 1) return false;
    float tt = dot(e2, q) * inv;
    if (tt < 0) return false;
    outT = tt;
    return true;
}

static bool rayAabb(Vec3 o, Vec3 invD, Vec3 lo, Vec3 hi, float maxT) {
    float tmin = 0, tmax = maxT;
    for (int i = 0; i < 3; i++) {
        float t1 = (lo[i] - o[i]) * invD[i], t2 = (hi[i] - o[i]) * invD[i];
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    return true;
}

// --- BVH -----------------------------------------------------------------

void TriangleBVH::build(std::vector<Triangle> tris) {
    m_nodes.clear();
    m_tris.clear();
    // drop degenerate and non-finite triangles
    tris.erase(std::remove_if(tris.begin(), tris.end(),
                              [](const Triangle &t) {
                                  return !finite(t.a) || !finite(t.b) || !finite(t.c) ||
                                         length2(cross(t.b - t.a, t.c - t.a)) < 1e-14f;
                              }),
               tris.end());
    if (tris.empty()) return;
    std::vector<uint32_t> idx(tris.size());
    std::vector<Vec3> centroids(tris.size());
    for (uint32_t i = 0; i < tris.size(); i++) {
        idx[i] = i;
        centroids[i] = (tris[i].a + tris[i].b + tris[i].c) / 3.0f;
    }
    m_nodes.reserve(tris.size() / 2 + 2);
    m_tris.reserve(tris.size());
    m_nodes.push_back({});
    buildNode(0, idx, centroids, 0, (uint32_t)idx.size(), tris);
}

void TriangleBVH::buildNode(uint32_t node, std::vector<uint32_t> &idx, const std::vector<Vec3> &centroids,
                            uint32_t begin, uint32_t end, const std::vector<Triangle> &src) {
    const float inf = std::numeric_limits<float>::max();
    Vec3 lo{inf, inf, inf}, hi{-inf, -inf, -inf}, clo = lo, chi = hi;
    for (uint32_t i = begin; i < end; i++) {
        const Triangle &t = src[idx[i]];
        lo = vmin(lo, vmin(t.a, vmin(t.b, t.c)));
        hi = vmax(hi, vmax(t.a, vmax(t.b, t.c)));
        clo = vmin(clo, centroids[idx[i]]);
        chi = vmax(chi, centroids[idx[i]]);
    }
    m_nodes[node].lo = lo;
    m_nodes[node].hi = hi;

    const uint32_t kLeafSize = 4;
    Vec3 ext = chi - clo;
    int axis = ext.x > ext.y ? (ext.x > ext.z ? 0 : 2) : (ext.y > ext.z ? 1 : 2);
    if (end - begin <= kLeafSize || ext[axis] <= 0) {
        m_nodes[node].left = (uint32_t)m_tris.size();
        m_nodes[node].count = end - begin;
        for (uint32_t i = begin; i < end; i++) m_tris.push_back(src[idx[i]]);
        return;
    }
    uint32_t mid = begin + (end - begin) / 2;
    std::nth_element(idx.begin() + begin, idx.begin() + mid, idx.begin() + end,
                     [&](uint32_t a, uint32_t b) { return centroids[a][axis] < centroids[b][axis]; });
    uint32_t left = (uint32_t)m_nodes.size();
    m_nodes.push_back({});
    m_nodes.push_back({});
    m_nodes[node].left = left;
    m_nodes[node].count = 0;
    buildNode(left, idx, centroids, begin, mid, src);
    buildNode(left + 1, idx, centroids, mid, end, src);
}

bool TriangleBVH::raycast(Vec3 o, Vec3 d, float maxDist, float &outT, Vec3 &outNormal, uint8_t &outMat,
                          uint32_t skipMask, const Material *materials) const {
    if (m_nodes.empty()) return false;
    Vec3 invD{1.0f / (std::fabs(d.x) > 1e-20f ? d.x : 1e-20f), 1.0f / (std::fabs(d.y) > 1e-20f ? d.y : 1e-20f),
              1.0f / (std::fabs(d.z) > 1e-20f ? d.z : 1e-20f)};
    float best = maxDist;
    const Triangle *hit = nullptr;
    uint32_t stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp) {
        const Node &n = m_nodes[stack[--sp]];
        if (!rayAabb(o, invD, n.lo, n.hi, best)) continue;
        if (n.count) {
            for (uint32_t i = 0; i < n.count; i++) {
                const Triangle &t = m_tris[n.left + i];
                if (materials && (materials[t.material].flags & skipMask)) continue;
                float tt;
                if (rayTriangle(o, d, t, tt) && tt < best) {
                    best = tt;
                    hit = &t;
                }
            }
        } else if (sp < 62) {
            stack[sp++] = n.left;
            stack[sp++] = n.left + 1;
        }
    }
    if (!hit) return false;
    outT = best;
    Vec3 nrm = normalize(cross(hit->b - hit->a, hit->c - hit->a));
    outNormal = dot(nrm, d) > 0 ? -nrm : nrm;
    outMat = hit->material;
    return true;
}

// --- World ---------------------------------------------------------------

CollisionWorld::CollisionWorld() {}

void CollisionWorld::setMaterial(uint8_t id, const Material &m) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_materials[id] = m;
}

void CollisionWorld::loadStatic(std::vector<Triangle> tris) {
    auto bvh = std::make_shared<TriangleBVH>();
    bvh->build(std::move(tris));  // outside the lock: can take a while
    std::lock_guard<std::mutex> lock(m_mutex);
    m_static = bvh->empty() ? nullptr : bvh;
}

uint32_t CollisionWorld::createObject(std::vector<Triangle> localTris, const RigidTransform &t, double simTime) {
    auto obj = std::make_unique<Object>();
    const float inf = std::numeric_limits<float>::max();
    obj->localLo = {inf, inf, inf};
    obj->localHi = {-inf, -inf, -inf};
    for (auto &tr : localTris) {
        for (Vec3 v : {tr.a, tr.b, tr.c}) {
            obj->localLo = vmin(obj->localLo, v);
            obj->localHi = vmax(obj->localHi, v);
            obj->boundRadius = std::max(obj->boundRadius, length(v));
        }
    }
    obj->bvh.build(std::move(localTris));
    obj->from = obj->to = t;
    obj->t0 = simTime;
    std::lock_guard<std::mutex> lock(m_mutex);
    uint32_t id = m_nextId++;
    if (m_nextId == 0) m_nextId = 1;
    m_objects[id] = std::move(obj);
    return id;
}

RigidTransform CollisionWorld::objectPose(const Object &o, double simTime) const {
    if (o.duration <= 0) return o.to;
    float a = (float)std::clamp((simTime - o.t0) / o.duration, 0.0, 1.0);
    return {lerp(o.from.pos, o.to.pos, a), slerp(o.from.rot, o.to.rot, a)};
}

bool CollisionWorld::moveObject(uint32_t id, const RigidTransform &t, double simTime) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_objects.find(id);
    if (it == m_objects.end()) return false;
    Object &o = *it->second;
    RigidTransform cur = objectPose(o, simTime);
    // The next move is expected about as far in the future as this one was
    // after the previous; interpolate over that interval.
    double interval = simTime - o.t0;
    o.from = cur;
    o.to = t;
    o.t0 = simTime;
    o.duration = (interval > 1e-4 && interval < 0.25) ? interval : 0.0;
    if (o.duration > 0) {
        o.linVel = (t.pos - cur.pos) / (float)o.duration;
        Quat dq = normalize(t.rot * conj(cur.rot));
        if (dq.w < 0) dq = {-dq.x, -dq.y, -dq.z, -dq.w};
        float s = std::sqrt(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z);
        float angle = 2.0f * std::atan2(s, dq.w);
        o.angVel = s > 1e-9f ? Vec3{dq.x, dq.y, dq.z} / s * (angle / (float)o.duration) : Vec3{};
    } else {
        o.linVel = {};
        o.angVel = {};
    }
    return true;
}

bool CollisionWorld::deleteObject(uint32_t id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_objects.erase(id) > 0;
}

void CollisionWorld::setProxyPlanes(std::vector<Contact> planes) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_proxyPlanes = std::move(planes);
}

// Caller must hold mutex().
size_t CollisionWorld::sphereContacts(Vec3 c, float r, double simTime, std::vector<Contact> &out) const {
    size_t before = out.size();
    auto addTri = [&](const Triangle &t, Vec3 centre, const auto &toWorldPoint, const auto &toWorldDir,
                      const auto &velAt, Vec3 angVel, uint32_t objectId) {
        if (m_materials[t.material].flags & kMatNoCollision) return;
        Vec3 q = closestPointOnTriangle(centre, t.a, t.b, t.c);
        Vec3 d = centre - q;
        float dist2 = length2(d);
        if (dist2 >= r * r) return;
        float dist = std::sqrt(dist2);
        Vec3 n;
        if (dist > 1e-6f) {
            n = d / dist;
        } else {
            n = normalize(cross(t.b - t.a, t.c - t.a));
        }
        Contact ct;
        ct.point = toWorldPoint(q);
        ct.normal = toWorldDir(n);
        ct.depth = r - dist;
        ct.material = t.material;
        ct.objectId = objectId;
        ct.surfaceVel = velAt(ct.point);
        ct.surfaceAngVel = angVel;
        out.push_back(ct);
    };
    auto ident = [](Vec3 v) { return v; };
    auto zeroVel = [](Vec3) { return Vec3{}; };

    Vec3 rr{r, r, r};
    if (m_static) {
        m_static->queryAabb(c - rr, c + rr, [&](const Triangle &t) { addTri(t, c, ident, ident, zeroVel, Vec3{}, 0); });
    }
    for (auto &kv : m_objects) {
        const Object &o = *kv.second;
        RigidTransform pose = objectPose(o, simTime);
        if (length(c - pose.pos) > o.boundRadius + r) continue;
        Quat inv = conj(pose.rot);
        Vec3 lc = rotate(inv, c - pose.pos);
        auto toWorldP = [&](Vec3 v) { return pose.pos + rotate(pose.rot, v); };
        auto toWorldD = [&](Vec3 v) { return rotate(pose.rot, v); };
        bool moving = o.duration > 0 && simTime < o.t0 + o.duration;
        auto vel = [&](Vec3 p) { return moving ? o.linVel + cross(o.angVel, p - pose.pos) : Vec3{}; };
        o.bvh.queryAabb(lc - rr, lc + rr, [&](const Triangle &t) { addTri(t, lc, toWorldP, toWorldD, vel, moving ? o.angVel : Vec3{}, kv.first); });
    }
    for (const Contact &p : m_proxyPlanes) {
        // A proxy plane is a disc of 0.75 m around the host's hit point.
        float h = dot(c - p.point, p.normal);
        if (h >= r || h < -r) continue;
        Vec3 onPlane = c - p.normal * h;
        if (length2(onPlane - p.point) > 0.75f * 0.75f) continue;
        if (m_materials[p.material].flags & kMatNoCollision) continue;
        Contact ct = p;
        ct.point = onPlane;
        ct.depth = r - h;
        out.push_back(ct);
    }
    return out.size() - before;
}

// Caller must hold mutex().
bool CollisionWorld::raycast(Vec3 o, Vec3 d, float maxDist, double simTime, float &outT, Vec3 &outN,
                             uint8_t &outMat, uint32_t &outObj, bool forVideo) const {
    uint32_t skip = forVideo ? kMatTransparentToVideo : kMatNoCollision;
    bool any = false;
    float best = maxDist;
    if (m_static) {
        float t;
        Vec3 n;
        uint8_t m;
        if (m_static->raycast(o, d, best, t, n, m, skip, m_materials)) {
            any = true;
            best = t;
            outN = n;
            outMat = m;
            outObj = 0;
        }
    }
    for (auto &kv : m_objects) {
        const Object &ob = *kv.second;
        RigidTransform pose = objectPose(ob, simTime);
        // cheap reject: distance from the ray to the object's bounding sphere
        Vec3 oc = pose.pos - o;
        float along = std::clamp(dot(oc, d), 0.0f, best);
        if (length(oc - d * along) > ob.boundRadius) continue;
        Quat inv = conj(pose.rot);
        float t;
        Vec3 n;
        uint8_t m;
        if (ob.bvh.raycast(rotate(inv, o - pose.pos), rotate(inv, d), best, t, n, m, skip, m_materials)) {
            any = true;
            best = t;
            outN = rotate(pose.rot, n);
            outMat = m;
            outObj = kv.first;
        }
    }
    if (!forVideo) {
        for (const Contact &p : m_proxyPlanes) {
            float denom = dot(d, p.normal);
            if (denom >= -1e-6f) continue;
            float t = dot(p.point - o, p.normal) / denom;
            if (t < 0 || t >= best) continue;
            if (length2(o + d * t - p.point) > 0.75f * 0.75f) continue;
            any = true;
            best = t;
            outN = p.normal;
            outMat = p.material;
            outObj = p.objectId;
        }
    }
    if (any) outT = best;
    return any;
}

}  // namespace fpv
