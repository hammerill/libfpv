// libfpv - wind (see wind.h and docs/DECISIONS.md D30).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "world/wind.h"

#include <cmath>
#include <cstring>

namespace fpv {

namespace {

constexpr float kRoughness = 0.3f;     // z0, m: suburbs and the edges of towns
constexpr float kProbeInterval = 0.1f;  // s between shelter probes
constexpr float kSmoothing = 0.6f;      // s, time constant of the probed values
constexpr float kGroundReach = 400.0f;  // m, the deepest ground looked for
constexpr float kUpwindReach = 80.0f;   // m, obstacles farther upwind are ignored
constexpr float kWindwardReach = 15.0f; // m, a wall this close downwind slows and lifts the air
constexpr float kEnclosureReach = 30.0f;
constexpr float kSourceReach = 150.0f;  // m, sources farther from the drone are dropped
// eddy sizes of the turbulence octaves, m (fixed: their weights follow the
// length scale, so a changing scale never jumps the field). Each divides
// the largest, so the clock can wrap without a seam.
constexpr float kOctaveSize[kWindOctaves] = {60.0f, 20.0f, 7.5f, 3.0f, 1.2f};
// blast wave
constexpr float kBlastFrontSpeed = 100.0f;  // m/s, slower than a real shock: a push you can feel
constexpr float kBlastPeak = 40.0f;         // m/s at the explosion's radius
constexpr float kBlastReach = 12.0f;        // radii

// --- gradient noise (Perlin's improved noise) ----------------------------------

struct Perm {
    uint8_t p[512];
    Perm() {
        uint8_t a[256];
        for (int i = 0; i < 256; i++) a[i] = (uint8_t)i;
        uint32_t s = 0x9E3779B9u;
        for (int i = 255; i > 0; i--) {
            s = s * 1664525u + 1013904223u;
            int j = (int)((s >> 8) % (uint32_t)(i + 1));
            std::swap(a[i], a[j]);
        }
        for (int i = 0; i < 512; i++) p[i] = a[i & 255];
    }
};
const Perm &perm() {
    static const Perm p;
    return p;
}

float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
float grad(int h, float x, float y, float z) {
    h &= 15;
    float u = h < 8 ? x : y, v = h < 4 ? y : (h == 12 || h == 14) ? x : z;
    return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}
float mix(float a, float b, float t) { return a + (b - a) * t; }

float noise3(float x, float y, float z) {
    const uint8_t *p = perm().p;
    float fx = std::floor(x), fy = std::floor(y), fz = std::floor(z);
    int X = (int)fx & 255, Y = (int)fy & 255, Z = (int)fz & 255;
    x -= fx, y -= fy, z -= fz;
    float u = fade(x), v = fade(y), w = fade(z);
    int A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z;
    int B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;
    return mix(mix(mix(grad(p[AA], x, y, z), grad(p[BA], x - 1, y, z), u),
                   mix(grad(p[AB], x, y - 1, z), grad(p[BB], x - 1, y - 1, z), u), v),
               mix(mix(grad(p[AA + 1], x, y, z - 1), grad(p[BA + 1], x - 1, y, z - 1), u),
                   mix(grad(p[AB + 1], x, y - 1, z - 1), grad(p[BB + 1], x - 1, y - 1, z - 1), u), v),
               w);
}

// The noise's standard deviation, measured once, so octave sums can be
// scaled to a unit standard deviation.
float noiseStd() {
    static const float s = [] {
        double sum = 0, sum2 = 0;
        const int n = 20000;
        uint32_t r = 12345;
        auto rnd = [&] {
            r = r * 1664525u + 1013904223u;
            return (float)(r >> 8) / 65536.0f;  // 0..256
        };
        for (int i = 0; i < n; i++) {
            float v = noise3(rnd(), rnd(), rnd());
            sum += v;
            sum2 += (double)v * v;
        }
        double m = sum / n;
        return (float)std::sqrt(std::max(1e-6, sum2 / n - m * m));
    }();
    return s;
}

// Each component of the eddies at p: unit standard deviation, spatially
// coherent (two points a rotor apart see nearly the same gust, a small
// difference makes the torque), weighted like von Karman's spectrum: the
// biggest eddies near the length scale, smaller ones weaker by (l/L)^(1/3).
Vec3 eddies(const WindClock &c, Vec3 p, float lengthScale) {
    static const Vec3 kComponentOffset[3] = {{0, 0, 0}, {31.7f, 47.3f, 11.9f}, {73.1f, 5.3f, 59.7f}};
    Vec3 q = p - c.drift;
    Vec3 out;
    float wsum = 0;
    for (int k = 0; k < kWindOctaves; k++) {
        float l = kOctaveSize[k];
        float rel = l / std::max(lengthScale, 1.0f);
        float w = std::cbrt(rel) / std::sqrt(1.0f + rel * rel);
        Vec3 base = q / l + Vec3{0, c.evolve[k], 0};
        for (int i = 0; i < 3; i++) {
            Vec3 b = base + kComponentOffset[i] + Vec3{(float)k * 17.0f, 0, 0};
            out[i] += w * noise3(b.x, b.y, b.z);
        }
        wsum += w * w;
    }
    return out / (noiseStd() * std::sqrt(std::max(wsum, 1e-6f)));
}

// Contracting rotor wake: radius and centreline speed at distance s below
// the disc (momentum theory: twice the induced velocity at 0.71 R, then a
// spreading turbulent column).
void rotorColumn(float R, float v0, float s, float &radius, float &speed) {
    if (s < 0) {
        radius = R;
        speed = v0 * std::max(0.0f, 1.0f + s / R);  // drawn in from above
    } else if (s < R) {
        radius = R * (1.0f - 0.29f * s / R);
        speed = v0 * (1.0f + s / R);
    } else {
        radius = R * (0.71f + 0.1f * (s - R) / R);
        speed = 2.0f * v0 * 0.71f * R / radius;
    }
}

Vec3 rotorWash(const WindSource &w, Vec3 p) {
    float R = std::max(w.radius, 0.05f);
    Vec3 x = p - w.pos;
    float s = dot(x, w.dir);
    if (s < -R || s > 40.0f * R) return {};
    float g = w.groundDist;
    if (g > 0 && s > g + 0.5f) return {};  // under the ground
    Vec3 radial = x - w.dir * s;
    float r = length(radial);
    float rc, vc;
    rotorColumn(R, w.speed, s, rc, vc);
    Vec3 v = w.dir * (vc * std::exp(-2.0f * r * r / (rc * rc)));
    if (g > 0) {
        // the wake lands and spreads out along the ground (a wall jet)
        float rg, vg;
        rotorColumn(R, w.speed, g, rg, vg);
        float hg = std::max(0.0f, g - s);  // above the ground, along the axis
        float turn = std::clamp(1.0f - hg / (1.5f * rg), 0.0f, 1.0f);
        v = v * (1.0f - turn);
        if (r > 1e-3f && r < 12.0f * R) {
            float vr = r < rg ? vg * r / rg : vg * rg / r;
            float thickness = 0.3f * R + 0.1f * r;
            v += radial / r * (vr * std::exp(-hg / thickness));
        }
    }
    return v;
}

Vec3 jetWash(const WindSource &w, Vec3 p) {
    float R = std::max(w.radius, 0.02f);
    Vec3 x = p - w.pos;
    float s = dot(x, w.dir);
    if (s < 0 || s > 80.0f * R) return {};
    float r = length(x - w.dir * s);
    float rc = R + 0.1f * s;                                 // spreads at about 6 degrees
    float vc = w.speed * std::min(1.0f, 6.0f * R / std::max(s, 1e-3f));  // past the potential core
    return w.dir * (vc * std::exp(-r * r / (rc * rc)));
}

Vec3 blastWind(const WindBlast &b, Vec3 p, double t) {
    float age = (float)(t - b.start);
    if (age < 0) return {};
    Vec3 x = p - b.pos;
    float r = length(x);
    float R = std::max(b.radius, 0.1f);
    if (r > kBlastReach * R || r < 1e-3f) return {};
    float front = kBlastFrontSpeed * age;
    if (r > front) return {};  // not here yet
    float behind = front - r, thickness = 0.25f * front + 3.0f;
    float v = kBlastPeak * std::pow(R / std::max(r, 0.5f * R), 1.5f) * std::exp(-behind / thickness);
    return x / r * v;
}

Vec3 rotY(Vec3 v, float a) {
    float c = std::cos(a), s = std::sin(a);
    return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
}

constexpr float kDeg = 3.14159265f / 180.0f;

}  // namespace

void WindClock::advance(const WindState &s, float dt) {
    if (!(dt > 0)) return;
    drift += s.convection * dt;
    // eddies change shape over about their own turnover time, l / sigma
    float sigma = std::max(s.sigma, 0.3f);
    // (the noise repeats every 256 cells: wrapping there keeps float
    // precision without a seam)
    for (int k = 0; k < kWindOctaves; k++) evolve[k] = std::fmod(evolve[k] + dt * 0.5f * sigma / kOctaveSize[k], 256.0f);
    const float period = 256.0f * kOctaveSize[0];
    for (int i = 0; i < 3; i++) {
        if (std::fabs(drift[i]) > 4 * period) drift[i] = std::fmod(drift[i], period);
    }
}

bool blastOver(const WindBlast &b, double t) {
    float reach = kBlastReach * std::max(b.radius, 0.1f);
    float age = (float)(t - b.start);
    return kBlastFrontSpeed * age > reach * 1.3f + 30.0f;
}

Vec3 sampleWind(const WindState &s, const WindClock &c, const std::vector<WindBlast> &blasts, Vec3 p, double t) {
    Vec3 v;
    bool needEddies = s.on && (s.sigma > 0 || !s.sources.empty());
    Vec3 e = needEddies ? eddies(c, p, s.lengthScale) : Vec3{};
    if (s.on) {
        v = s.mean;
        if (s.sigma > 0) {
            // along-wind gusts strongest, across a little less, vertical about half
            Vec3 h{s.mean.x, 0, s.mean.z};
            Vec3 along = normalize(h, {0, 0, 1});
            Vec3 across = cross(Vec3{0, 1, 0}, along);
            v += along * (e.x * s.sigma) + across * (e.z * 0.8f * s.sigma) + Vec3{0, e.y * 0.55f * s.sigma, 0};
        }
        for (const WindSource &w : s.sources) {
            Vec3 a = w.type == WindSource::Jet ? jetWash(w, p) : rotorWash(w, p);
            // downwash and jets are very turbulent: about a quarter of their speed
            float l = length(a);
            if (l > 0) v += a + e * (0.25f * l);
        }
    }
    for (const WindBlast &b : blasts) v += blastWind(b, p, t);
    return v;
}

WindProbe::Raw WindProbe::measure(Vec3 p, Vec3 w, const Raycast &ray) const {
    Raw r;
    const Vec3 up{0, 1, 0}, down{0, -1, 0};
    float t;
    Vec3 n;

    // The height that sets the wind's strength is over the open ground
    // around, not over a roof just below: the lowest of five ground probes.
    float hLocal = ray(p, down, kGroundReach, t, n) ? t : kGroundReach;
    float hLow = hLocal;
    static const float kAround[4][2] = {{25, 0}, {-25, 0}, {0, 25}, {0, -25}};
    for (const auto &o : kAround) {
        if (ray(p + Vec3{o[0], 0, o[1]}, down, kGroundReach, t, n)) hLow = std::max(hLow, t);
    }
    r.hEff = hLow;
    r.hLocal = hLocal;

    // Upwind: horizontal rays at a fan of headings and a ladder of heights.
    // At each heading the wall at the drone's height (or just above) is the
    // obstacle, and the highest rung still hitting about as close gives how
    // far it reaches above the drone. Behind it the wind drops to about a
    // fifth (a little backwards right behind the wall) and comes back over
    // some ten obstacle heights downwind, gustier than in the open.
    static const float kAzimuth[] = {-30, -15, 0, 15, 30};
    static const float kRung[] = {0, 3, 10, 25, 60, 140};
    Vec3 upwind = -w;
    float retained = 0, boost = 0;
    for (float az : kAzimuth) {
        Vec3 d = rotY(upwind, az * kDeg);
        float wall = -1, top = 0;
        for (float rung : kRung) {
            bool hit = ray(p + up * rung, d, kUpwindReach, t, n) && std::fabs(n.y) < 0.7f;
            if (rung <= 3) {
                if (hit && (wall < 0 || t < wall)) wall = t;
                if (hit) top = rung;
            } else if (wall >= 0) {
                if (hit && t < wall + 25.0f) top = rung;
                else break;  // the obstacle ends below this rung
            }
        }
        float keep = 1, gust = 0;
        if (wall >= 0) {
            float dz = top + 1.5f;  // the rung below the first open one: about the top
            float height = std::max(2.0f, std::min(hLocal, 60.0f) + dz);
            float x = wall / height;
            float cover = std::clamp(dz / (0.3f * height + 1.0f), 0.0f, 1.0f);
            float deficit = x < 1 ? 0.85f + 0.25f * (1 - x) : 0.85f * std::exp(-(x - 1) / 5);
            keep = 1 - cover * deficit;
            // calmer in the cavity right behind the wall, gustiest where
            // the flow comes back down (about one height behind it)
            gust = cover * 0.8f * (x < 1 ? 0.5f + 0.5f * x : std::exp(-(x - 1) / 6));
        }
        retained += keep;
        boost += gust;
    }
    const float nAz = (float)(sizeof(kAzimuth) / sizeof(kAzimuth[0]));
    retained /= nAz;
    boost /= nAz;

    // Downwind: a wall just ahead stops the air and turns it upwards
    // (updrafts along windward faces), or sideways where it is short.
    float windward = 0, lift = 0;
    for (float az : {-20.0f, 0.0f, 20.0f}) {
        Vec3 d = rotY(w, az * kDeg);
        if (!ray(p, d, kWindwardReach, t, n) || std::fabs(n.y) > 0.7f) continue;
        float block = 1 - t / kWindwardReach;
        bool tall = ray(p + up * 6.0f, d, kWindwardReach + 3.0f, t, n) && std::fabs(n.y) < 0.7f;
        windward += block / 3;
        if (tall) lift += 0.6f * block / 3;
    }

    // Enclosure: a roof overhead and walls nearly all round (a hall, a
    // tunnel, a car park) let little of the weather in.
    float enclosed = 0;
    if (ray(p, up, 60.0f, t, n)) {
        int walls = 0;
        for (int i = 0; i < 8; i++) {
            if (ray(p, rotY({0, 0, 1}, (float)i * 45.0f * kDeg), kEnclosureReach, t, n)) walls++;
        }
        enclosed = std::clamp((walls / 8.0f - 0.5f) * 2.0f, 0.0f, 1.0f);
    }

    r.retained = retained * (1 - 0.6f * windward) * (1 - 0.95f * enclosed);
    r.turbBoost = boost * (1 - enclosed);
    r.updraft = lift * (1 - enclosed);
    r.enclosed = enclosed;
    return r;
}

WindState WindProbe::update(float dt, Vec3 p, const WindAmbient &a, float strength,
                            const std::vector<WindSource> &sources, const Raycast &ray) {
    WindState s;
    Vec3 base = a.velocity * strength;
    Vec3 baseH{base.x, 0, base.z};
    float speedH = length(baseH);
    if (speedH > 0.05f && !a.enclosed) {
        Vec3 dir = baseH / speedH;
        float profile = 1, intensity = a.turbulence >= 0 ? a.turbulence : 0.2f;
        if (!a.noShelter) {
            m_timer -= dt;
            m_sinceProbe += dt;
            // re-probe at once when the wind turns more than about 15 degrees
            if (!m_valid || m_timer <= 0 || dot(dir, m_lastDir) < 0.966f) {
                Raw raw = measure(p, dir, ray);
                float k = m_valid ? 1.0f - std::exp(-m_sinceProbe / kSmoothing) : 1.0f;
                m_retained += (raw.retained - m_retained) * k;
                m_turbBoost += (raw.turbBoost - m_turbBoost) * k;
                m_updraft += (raw.updraft - m_updraft) * k;
                m_enclosed += (raw.enclosed - m_enclosed) * k;
                m_hEff += (raw.hEff - m_hEff) * k;
                m_hLocal += (raw.hLocal - m_hLocal) * k;
                m_valid = true;
                m_timer = kProbeInterval;
                m_sinceProbe = 0;
                m_lastDir = dir;
            }
            // the logarithmic wind profile from the open ground, and the
            // gustiness that comes with it (1 / ln(z / z0))
            float h = std::max(m_hEff, 2 * kRoughness);
            float ref = std::max(a.refHeight, 2 * kRoughness);
            profile = std::clamp(std::log(h / kRoughness) / std::log(ref / kRoughness), 0.3f, 2.2f);
            // just over a roof well above the ground, the air that came over
            // its edge has not come back down yet: about half the wind on it
            if (m_hLocal < 3.0f && m_hEff > m_hLocal + 5.0f) profile *= 0.5f + 0.5f * m_hLocal / 3.0f;
            if (a.turbulence < 0) intensity = std::clamp(1.0f / std::log(std::max(h, 1.0f) / kRoughness), 0.1f, 0.35f);
        } else {
            m_valid = false;
        }
        float free = speedH * profile;
        float retained = m_valid ? m_retained : 1.0f;
        float boost = m_valid ? m_turbBoost : 0.0f;
        float lift = m_valid ? m_updraft : 0.0f;
        float enclosed = m_valid ? m_enclosed : 0.0f;
        s.mean = dir * (free * retained) + Vec3{0, free * lift + base.y, 0};
        // a wake's eddies are shed by the faster air going past the obstacle
        s.convection = dir * (free * (0.5f + 0.5f * std::clamp(retained, 0.0f, 1.0f)));
        s.sigma = intensity * free * (1 + boost) * (1 - 0.9f * enclosed);
        s.lengthScale = std::clamp(0.6f * (m_valid ? m_hEff : 10.0f) + 5.0f, 5.0f, 80.0f);
        s.on = true;
    } else {
        m_valid = false;
    }
    for (const WindSource &src : sources) {
        if (s.sources.size() >= (size_t)kMaxWindSources) break;
        if (!(src.speed > 0) || length(src.pos - p) > kSourceReach + 40.0f * src.radius) continue;
        WindSource w = src;
        if (w.type == WindSource::Rotor) {
            // where the wake lands: the first surface past the aircraft's
            // own body (within 0.6 radii of the hub)
            float from = 0.6f * w.radius, t;
            Vec3 n;
            w.groundDist = ray(w.pos + w.dir * from, w.dir, 40.0f * w.radius, t, n) ? from + t : -1.0f;
        }
        s.sources.push_back(w);
    }
    if (!s.sources.empty()) s.on = true;
    return s;
}

}  // namespace fpv
