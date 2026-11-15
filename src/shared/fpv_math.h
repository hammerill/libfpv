// libfpv - small vector/quaternion library and host coordinate conversion.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fpv {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravity = 9.81f;  // SimITL hard-codes this value

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    float &operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, Vec3 a) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator/(Vec3 a, float s) { return {a.x / s, a.y / s, a.z / s}; }
inline Vec3 &operator+=(Vec3 &a, Vec3 b) { a = a + b; return a; }
inline Vec3 &operator-=(Vec3 &a, Vec3 b) { a = a - b; return a; }
inline Vec3 &operator*=(Vec3 &a, float s) { a = a * s; return a; }
inline Vec3 mul(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length2(Vec3 a) { return dot(a, a); }
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a, Vec3 fallback = {0, 1, 0}) {
    float l = length(a);
    return l > 1e-12f ? a / l : fallback;
}
inline Vec3 vmin(Vec3 a, Vec3 b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 vmax(Vec3 a, Vec3 b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }
inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }
inline bool finite(Vec3 a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }

// Row-major 3x3 matrix, v' = M v. Columns are the images of the basis axes.
struct Mat3 {
    Vec3 r[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Vec3 col(int i) const { return {r[0][i], r[1][i], r[2][i]}; }
    static Mat3 fromCols(Vec3 a, Vec3 b, Vec3 c) {
        Mat3 m;
        m.r[0] = {a.x, b.x, c.x};
        m.r[1] = {a.y, b.y, c.y};
        m.r[2] = {a.z, b.z, c.z};
        return m;
    }
    static Mat3 diag(Vec3 d) {
        Mat3 m;
        m.r[0] = {d.x, 0, 0};
        m.r[1] = {0, d.y, 0};
        m.r[2] = {0, 0, d.z};
        return m;
    }
};
inline Vec3 operator*(const Mat3 &m, Vec3 v) { return {dot(m.r[0], v), dot(m.r[1], v), dot(m.r[2], v)}; }
inline Mat3 transpose(const Mat3 &m) { return Mat3::fromCols(m.r[0], m.r[1], m.r[2]); }
inline Mat3 operator*(const Mat3 &a, const Mat3 &b) {
    return Mat3::fromCols(a * b.col(0), a * b.col(1), a * b.col(2));
}
inline float det(const Mat3 &m) { return dot(m.r[0], cross(m.r[1], m.r[2])); }

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
    Quat() = default;
    constexpr Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
};
inline Quat operator*(Quat a, Quat b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quat conj(Quat q) { return {-q.x, -q.y, -q.z, q.w}; }
inline float qdot(Quat a, Quat b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
inline Quat normalize(Quat q) {
    float l = std::sqrt(qdot(q, q));
    if (!(l > 1e-12f) || !std::isfinite(l)) return {};
    return {q.x / l, q.y / l, q.z / l, q.w / l};
}
inline Quat axisAngle(Vec3 axis, float angle) {
    axis = normalize(axis);
    float s = std::sin(angle * 0.5f);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(angle * 0.5f)};
}
inline Vec3 rotate(Quat q, Vec3 v) {
    Vec3 u{q.x, q.y, q.z};
    Vec3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}
inline Mat3 toMat3(Quat q) {
    return Mat3::fromCols(rotate(q, {1, 0, 0}), rotate(q, {0, 1, 0}), rotate(q, {0, 0, 1}));
}
inline Quat toQuat(const Mat3 &m) {
    // Shepperd's method on the rotation part of m
    float t = m.r[0][0] + m.r[1][1] + m.r[2][2];
    Quat q;
    if (t > 0) {
        float s = std::sqrt(t + 1.0f) * 2.0f;
        q = {(m.r[2][1] - m.r[1][2]) / s, (m.r[0][2] - m.r[2][0]) / s, (m.r[1][0] - m.r[0][1]) / s, 0.25f * s};
    } else if (m.r[0][0] > m.r[1][1] && m.r[0][0] > m.r[2][2]) {
        float s = std::sqrt(1.0f + m.r[0][0] - m.r[1][1] - m.r[2][2]) * 2.0f;
        q = {0.25f * s, (m.r[0][1] + m.r[1][0]) / s, (m.r[0][2] + m.r[2][0]) / s, (m.r[2][1] - m.r[1][2]) / s};
    } else if (m.r[1][1] > m.r[2][2]) {
        float s = std::sqrt(1.0f + m.r[1][1] - m.r[0][0] - m.r[2][2]) * 2.0f;
        q = {(m.r[0][1] + m.r[1][0]) / s, 0.25f * s, (m.r[1][2] + m.r[2][1]) / s, (m.r[0][2] - m.r[2][0]) / s};
    } else {
        float s = std::sqrt(1.0f + m.r[2][2] - m.r[0][0] - m.r[1][1]) * 2.0f;
        q = {(m.r[0][2] + m.r[2][0]) / s, (m.r[1][2] + m.r[2][1]) / s, 0.25f * s, (m.r[1][0] - m.r[0][1]) / s};
    }
    return normalize(q);
}
inline Quat slerp(Quat a, Quat b, float t) {
    float d = qdot(a, b);
    if (d < 0) {
        b = {-b.x, -b.y, -b.z, -b.w};
        d = -d;
    }
    if (d > 0.9995f) {
        return normalize(Quat{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
                              a.w + (b.w - a.w) * t});
    }
    float th = std::acos(d), s = std::sin(th);
    float wa = std::sin((1 - t) * th) / s, wb = std::sin(t * th) / s;
    return {a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb};
}
// Rotation whose local Z looks along `forward` with local Y as close to `up` as possible.
inline Quat lookRotation(Vec3 forward, Vec3 up) {
    Vec3 f = normalize(forward, {0, 0, 1});
    Vec3 r = cross(up, f);
    if (length2(r) < 1e-8f) r = cross(Vec3{0, 0, 1}, f);
    if (length2(r) < 1e-8f) r = {1, 0, 0};
    r = normalize(r);
    Vec3 u = cross(f, r);
    return toQuat(Mat3::fromCols(r, u, f));
}

// Converts between the host world convention and libfpv's internal frame
// (X right, Y up, Z forward, metres; the same as SimITL).
class CoordConverter {
public:
    // axes: 0..5 = +X,-X,+Y,-Y,+Z,-Z (fpv_axis); returns false if not a basis.
    bool setup(int right, int up, int forward, float unitsPerMeter) {
        int axes[3] = {right, up, forward};
        bool used[3] = {false, false, false};
        for (int i = 0; i < 3; i++) {
            if (axes[i] < 0 || axes[i] > 5) return false;
            int a = axes[i] / 2;
            if (used[a]) return false;
            used[a] = true;
        }
        if (!(unitsPerMeter > 0) || !std::isfinite(unitsPerMeter)) return false;
        // Column i of H2I... build the host->internal matrix row by row:
        // internal component i = sign * host component of axis i.
        for (int i = 0; i < 3; i++) {
            Vec3 row{0, 0, 0};
            row[axes[i] / 2] = (axes[i] % 2) ? -1.0f : 1.0f;
            m_h2i.r[i] = row;
        }
        m_i2h = transpose(m_h2i);
        m_reflect = det(m_h2i) < 0;
        m_upm = unitsPerMeter;
        return true;
    }

    float unitsPerMeter() const { return m_upm; }

    Vec3 dirToInternal(Vec3 h) const { return m_h2i * h; }
    Vec3 dirToHost(Vec3 i) const { return m_i2h * i; }
    Vec3 posToInternal(Vec3 h) const { return m_h2i * h / m_upm; }
    Vec3 posToHost(Vec3 i) const { return m_i2h * (i * m_upm); }
    float lenToInternal(float h) const { return h / m_upm; }
    float lenToHost(float i) const { return i * m_upm; }

    // Rotations are conjugated by the change of basis: R_i = C R_h C^T. This
    // stays a proper rotation even when C is a reflection (handedness flip).
    Quat rotToInternal(Quat h) const { return toQuat(m_h2i * toMat3(normalize(h)) * m_i2h); }
    Quat rotToHost(Quat i) const { return toQuat(m_i2h * toMat3(normalize(i)) * m_h2i); }

    // Angular velocity is a pseudo-vector: it flips sign under reflection.
    Vec3 angVelToInternal(Vec3 h) const { return dirToInternal(h) * (m_reflect ? -1.0f : 1.0f); }
    Vec3 angVelToHost(Vec3 i) const { return dirToHost(i) * (m_reflect ? -1.0f : 1.0f); }

private:
    Mat3 m_h2i, m_i2h;
    bool m_reflect = false;
    float m_upm = 1.0f;
};

}  // namespace fpv
