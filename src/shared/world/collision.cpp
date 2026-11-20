// libfpv - collision world implementation.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
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

uint32_t CollisionWorld::createObject(std::vector<Triangle> localTris, const RigidTransform &t, double simTime,
                                      uint32_t id) {
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
    obj->anchor = t.pos;
    obj->moves.assign(1, MoveSample{simTime, simTime, Vec3{}, t.rot});
    obj->t0 = obj->holdAt = obj->blendFrom = simTime;
    obj->fitRot = t.rot;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (id == 0) {
        id = m_nextId++;
        if (m_nextId == 0) m_nextId = 1;
    }
    m_objects[id] = std::move(obj);
    return id;
}

namespace {
// The host's moves come once a frame of its own, read against the simulation's
// clock a few milliseconds early or late: at 36 m/s, 3 ms is 11 cm. A
// surface going from one move to the next in its interval (the first
// approach) stopped dead whenever the next move came late, then jumped, and
// its velocity was off by the jitter over the interval: a quad sitting on a
// car roof was kicked off at 20 km/h. The surface now follows the line fitted
// through the last kFitWindow of moves, moved on at the fitted velocity, and
// each new move bends the path over to its new line within kBlendTime, the
// velocity continuous (D39).
constexpr double kFitWindow = 0.06;    // s of moves the velocity is fitted to
constexpr float kReanchorDistance = 50.0f;  // m from the anchor
constexpr size_t kOrderMoves = 12;
constexpr float kAltEvidenceSwitch = 6.0f, kAltEvidenceMax = 30.0f;  // log residual ratios, summed    // moves kept: to tell which times they are for
constexpr double kBlendTime = 0.05;    // s to take up a new move
constexpr double kTeleportGap = 0.5;   // s without moves: the next one starts afresh
constexpr double kSameTime = 1e-4;     // s: moves closer than this replace each other
// with no new move, it goes on at its velocity for a few of the host's intervals, then stops
constexpr double kCoastIntervals = 4.0, kCoastMin = 0.1, kCoastMax = 0.3;
constexpr float kTeleportDistance = 1.0f;  // m off the path (plus 0.2 s of its speed): a teleport
constexpr float kTeleportAngle = 0.5f;     // rad off the path

// rotation vector of q (shortest way), and back
Vec3 rotationVector(Quat q) {
    if (q.w < 0) q = {-q.x, -q.y, -q.z, -q.w};
    Vec3 v{q.x, q.y, q.z};
    float s = length(v);
    if (s < 1e-9f) return v * 2.0f;
    return v * (2.0f * std::atan2(s, q.w) / s);
}
Quat fromRotationVector(Vec3 r) {
    float a = length(r);
    if (a < 1e-9f) return normalize(Quat{r.x * 0.5f, r.y * 0.5f, r.z * 0.5f, 1.0f});
    return axisAngle(r / a, a);
}
}  // namespace

CollisionWorld::ObjectMotion CollisionWorld::objectMotion(const Object &o, double simTime) const {
    if (simTime == o.cacheTime) return o.cache;
    ObjectMotion m;
    m.moving = simTime < o.holdAt;
    float tm = (float)(std::min(simTime, o.holdAt) - o.t0);  // < 0 before a manual host's stamp: fine, a line
    Vec3 pos = o.fitPos + o.fitVel * tm;
    Quat rot = fromRotationVector(o.fitAngVel * tm) * o.fitRot;
    if (m.moving) {
        m.linVel = o.fitVel;
        m.angVel = o.fitAngVel;
    }
    double tau = simTime - o.blendFrom;
    if (tau >= 0 && tau < kBlendTime) {
        // cubic Hermite from the old path's offset and rate to none
        float s = (float)(tau / kBlendTime), T = (float)kBlendTime;
        float h00 = (2 * s - 3) * s * s + 1, h10 = ((s - 2) * s + 1) * s * T;
        float d00 = (6 * s - 6) * s / T, d10 = (3 * s - 4) * s + 1;
        pos += o.errPos * h00 + o.errVel * h10;
        m.linVel += o.errPos * d00 + o.errVel * d10;
        rot = fromRotationVector(o.errRot * h00 + o.errAngVel * h10) * rot;
        m.angVel += o.errRot * d00 + o.errAngVel * d10;
    }
    m.rel = pos;
    m.pose = {o.anchor + pos, normalize(rot)};
    o.cacheTime = simTime;
    o.cache = m;
    return m;
}

namespace {
// Sum of squared residuals of the least squares parabola through (t, y),
// t relative to the newest: how far the moves are from a smooth path.
float parabolaResidual(const double *t, const Vec3 *y, size_t n) {
    double S[5] = {0, 0, 0, 0, 0};
    double Y[3][3] = {};
    for (size_t i = 0; i < n; i++) {
        double p = 1;
        for (int k = 0; k < 5; k++, p *= t[i]) S[k] += p;
        for (int c = 0; c < 3; c++) {
            double v = y[i][c];
            Y[c][0] += v;
            Y[c][1] += v * t[i];
            Y[c][2] += v * t[i] * t[i];
        }
    }
    // normal equations [S0 S1 S2; S1 S2 S3; S2 S3 S4] x = Y, by Cramer
    auto det3 = [](double a, double b, double c, double d, double e, double f, double g, double h, double k) {
        return a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
    };
    double D = det3(S[0], S[1], S[2], S[1], S[2], S[3], S[2], S[3], S[4]);
    if (std::fabs(D) < 1e-30) return 0;
    double res = 0;
    for (int c = 0; c < 3; c++) {
        double y0 = Y[c][0], y1 = Y[c][1], y2 = Y[c][2];
        double a0 = det3(y0, S[1], S[2], y1, S[2], S[3], y2, S[3], S[4]) / D;
        double a1 = det3(S[0], y0, S[2], S[1], y1, S[3], S[2], y2, S[4]) / D;
        double a2 = det3(S[0], S[1], y0, S[1], S[2], y1, S[2], S[3], y2) / D;
        for (size_t i = 0; i < n; i++) {
            double e = y[i][c] - (a0 + a1 * t[i] + a2 * t[i] * t[i]);
            res += e * e;
        }
    }
    return (float)res;
}
}  // namespace

bool CollisionWorld::moveObject(uint32_t id, const RigidTransform &t, double simTime, double lead, double leadAlt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_objects.find(id);
    if (it == m_objects.end()) return false;
    Object &o = *it->second;
    struct Uncache {  // the path changes below
        const Object &o;
        ~Uncache() { o.cacheTime = -1e300; }
    } uncache{o};
    const double arrived = simTime;
    // the times the pose may be for
    const double tA = simTime + std::clamp(lead, -0.1, 0.25), tB = simTime + std::clamp(leadAlt, -0.1, 0.25);
    auto restart = [&] {
        o.anchor = t.pos;
        o.moves.assign(1, MoveSample{tA, tB, Vec3{}, t.rot});
        o.t0 = o.holdAt = o.blendFrom = arrived;
        o.fitPos = Vec3{};
        o.fitRot = t.rot;
        o.fitVel = o.fitAngVel = o.errPos = o.errVel = o.errRot = o.errAngVel = Vec3{};
    };
    if (o.moves.empty() || tA - o.moves.back().t > kTeleportGap || tA < o.moves.back().t ||
        tB < o.moves.back().tAlt) {
        restart();
        return true;
    }
    // where the path is now, and where it would be when the pose is for
    ObjectMotion now = objectMotion(o, arrived), then = objectMotion(o, o.alt ? tB : tA);
    float speed = length(now.linVel);
    if (length(t.pos - then.pose.pos) > kTeleportDistance + 0.2f * speed ||
        length(rotationVector(t.rot * conj(then.pose.rot))) > kTeleportAngle + 0.2f * length(now.angVel)) {
        restart();
        return true;
    }
    // a new anchor once the object went far from it, the path moved with it
    if (length(t.pos - o.anchor) > kReanchorDistance) {
        Vec3 shift = t.pos - o.anchor;
        o.anchor = t.pos;
        for (MoveSample &mv : o.moves) mv.pos -= shift;
        o.fitPos -= shift;
        now.rel -= shift;
    }
    Vec3 rel = t.pos - o.anchor;
    if (tA - o.moves.back().t < kSameTime) {
        // the same instant (a paused simulation, two moves in a frame): the later wins
        o.moves.back().pos = rel;
        o.moves.back().rot = t.rot;
    } else {
        o.moves.push_back({tA, tB, rel, t.rot});
        if (o.moves.size() > kOrderMoves) o.moves.erase(o.moves.begin());
    }

    // Which times the moves are for: the ones they line up smoothly with.
    // With even frames both do (and it matters little which); with uneven
    // ones, the wrong frame's times put each move off its path by its speed
    // times the difference. The order is the host's, the same for every
    // object, so every moving object adds to one tally (m_altEvidence, how
    // much smoother the other times were), and it changes only when that is
    // clear.
    if (o.moves.size() >= 4) {
        size_t n = o.moves.size();
        double ta[kOrderMoves], tb[kOrderMoves];
        Vec3 y[kOrderMoves];
        const MoveSample &last = o.moves.back();
        float lever = std::max(0.1f, o.boundRadius);
        float resA = 0, resB = 0;
        for (int part = 0; part < 2; part++) {  // positions, then turns (at the object's size)
            for (size_t i = 0; i < n; i++) {
                const MoveSample &mv = o.moves[i];
                ta[i] = mv.t - last.t;
                tb[i] = mv.tAlt - last.tAlt;
                y[i] = part == 0 ? mv.pos - last.pos : rotationVector(mv.rot * conj(last.rot)) * lever;
            }
            resA += parabolaResidual(ta, y, n);
            resB += parabolaResidual(tb, y, n);
        }
        if (std::max(resA, resB) > 1e-10f) {
            float r = std::log((resA + 1e-12f) / (resB + 1e-12f));
            m_altEvidence = std::clamp(m_altEvidence + std::clamp(r, -2.0f, 2.0f), -kAltEvidenceMax, kAltEvidenceMax);
            if (m_altEvidence > kAltEvidenceSwitch) m_alt = true;
            if (m_altEvidence < -kAltEvidenceSwitch) m_alt = false;
        }
    }
    o.alt = m_alt;

    // least squares line through the moves of the last kFitWindow (at least
    // two), relative to the newest (positions far from the origin keep their
    // precision in the differences)
    const bool alt = o.alt;
    const MoveSample &last = o.moves.back();
    const double tLast = last.at(alt);
    size_t first = o.moves.size() - std::min<size_t>(o.moves.size(), 2);
    while (first > 0 && tLast - o.moves[first - 1].at(alt) <= kFitWindow) first--;
    size_t n = o.moves.size() - first;
    double st = 0, stt = 0;
    Vec3 sp{}, stp{}, sr{}, str{};
    for (size_t i = first; i < o.moves.size(); i++) {
        const MoveSample &mv = o.moves[i];
        double dt = mv.at(alt) - tLast;
        Vec3 dp = mv.pos - last.pos;
        Vec3 dr = rotationVector(mv.rot * conj(last.rot));
        st += dt;
        stt += dt * dt;
        sp += dp;
        sr += dr;
        stp += dp * (float)dt;
        str += dr * (float)dt;
    }
    double den = (double)n * stt - st * st;
    Vec3 vel{}, angVel{}, offP = sp / (float)n, offR = sr / (float)n;
    if (den > 1e-12) {
        vel = (stp * (float)n - sp * (float)st) / (float)den;
        angVel = (str * (float)n - sr * (float)st) / (float)den;
        offP = (sp - vel * (float)st) / (float)n;
        offR = (sr - angVel * (float)st) / (float)n;
    }
    o.t0 = tLast;
    o.fitPos = last.pos + offP;
    o.fitRot = normalize(fromRotationVector(offR) * last.rot);
    o.fitVel = vel;
    o.fitAngVel = angVel;
    // the line where the blend begins
    o.blendFrom = arrived;
    float back = (float)(arrived - o.t0);
    Vec3 linePos = o.fitPos + vel * back;
    Quat lineRot = fromRotationVector(angVel * back) * o.fitRot;
    o.errPos = now.rel - linePos;
    o.errVel = now.linVel - vel;
    o.errRot = rotationVector(now.pose.rot * conj(lineRot));
    o.errAngVel = now.angVel - angVel;
    double interval = n > 1 ? (tLast - o.moves[first].at(alt)) / (double)(n - 1) : 0.0;
    // coasting from when the move came: one made after the tick comes a frame
    // after its time, and a 40 ms frame then stopped the surface dead
    o.holdAt = std::max(tLast, arrived) + std::clamp(interval * kCoastIntervals, kCoastMin, kCoastMax);
    if (den <= 1e-12) o.holdAt = arrived;  // nothing to tell a velocity from yet
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
        ObjectMotion om = objectMotion(o, simTime);
        const RigidTransform &pose = om.pose;
        if (length(c - pose.pos) > o.boundRadius + r) continue;
        Quat inv = conj(pose.rot);
        Vec3 lc = rotate(inv, c - pose.pos);
        auto toWorldP = [&](Vec3 v) { return pose.pos + rotate(pose.rot, v); };
        auto toWorldD = [&](Vec3 v) { return rotate(pose.rot, v); };
        auto vel = [&](Vec3 p) { return om.linVel + cross(om.angVel, p - pose.pos); };
        o.bvh.queryAabb(lc - rr, lc + rr, [&](const Triangle &t) { addTri(t, lc, toWorldP, toWorldD, vel, om.angVel, kv.first); });
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
