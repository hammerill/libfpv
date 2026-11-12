// libfpv - the video link: link budget and the goggles' feed (D34).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "video/video_link.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "config/preset.h"

namespace fpv {

namespace {

float smoothstep(float e0, float e1, float x) {
    float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3 - 2 * t);
}

uint8_t byteOf(float v) { return (uint8_t)std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f); }

// Analog timing
struct AnalogTiming {
    int activeLines, totalLines;  // per field
    double lineS, activeS;        // a line, its visible part
    float fieldRate;
    double subcarrierHz;
};
AnalogTiming timing(int system) {
    if (system == FPV_VIDEO_PAL) return {288, 313, 64e-6, 52e-6, 50.0f, 4.43361875e6};
    return {240, 263, 63.556e-6, 52.66e-6, 59.94f, 3.579545e6};
}

// The video baseband an FPV transmitter passes (the audio subcarriers sit at
// 6.0-6.5 MHz) and the colour bandwidth a goggle's decoder keeps.
constexpr double kLumaBandwidthHz = 5.0e6;
constexpr double kChromaBandwidthHz = 1.0e6;
// A goggle's video decoder samples the active line 720 times (BT.601).
constexpr int kAnalogSamples = 720;
// Click cells per line in the shader (fpv_video_gl3.h): a click is about
// two decoder samples wide.
constexpr float kClickCells = 360.0f;
// IF bandwidth for Rice's click rate
constexpr double kIfBandwidthHz = 18e6;

}  // namespace

VideoSpec videoSpec(const DronePreset &p) {
    VideoSpec s;
    const VtxSettings &v = p.vtx;
    const CameraSettings &c = p.camera;
    s.digital = v.digital;
    s.aspect = c.aspect > 0.5f ? c.aspect : 4.0f / 3.0f;
    s.powerDbm = 10.0f * std::log10(std::max(v.powerMw, 0.01f));
    s.powerNoise = std::clamp(v.powerNoise, 0.0f, 1.0f);
    s.minVoltage = v.minVoltage;
    s.sharpening = std::clamp(c.sharpening, 0.0f, 1.0f);
    if (s.digital) {
        s.system = FPV_VIDEO_DIGITAL;
        s.rasterH = std::clamp(v.height, 240, 2160);
        s.rasterW = std::max(2, (int)std::lround(s.rasterH * s.aspect / 2.0f) * 2);
        s.fieldRate = std::clamp(v.fps, 10.0f, 240.0f);
        s.freezes = v.freezes;
        s.sensitivityDbm = v.sensitivityDbm;
        s.lumaCycles = s.rasterW * 0.5f;
        s.chromaCycles = s.rasterW * 0.25f;  // 4:2:0
        s.subcarrierCycles = 0;
    } else {
        s.system = c.system == FPV_VIDEO_PAL ? FPV_VIDEO_PAL : FPV_VIDEO_NTSC;
        AnalogTiming t = timing(s.system);
        s.rasterW = kAnalogSamples;
        s.rasterH = t.activeLines * 2;
        s.fieldRate = t.fieldRate;
        // TVL counts lines across a width equal to the picture's height
        float camera = std::max(100.0f, c.tvl) * s.aspect * 0.5f;
        float link = (float)(kLumaBandwidthHz * t.activeS);
        float decoder = kAnalogSamples * 0.5f;
        s.lumaCycles = 1.0f / std::sqrt(1 / (camera * camera) + 1 / (link * link) + 1 / (decoder * decoder));
        s.chromaCycles = (float)(kChromaBandwidthHz * t.activeS);
        s.subcarrierCycles = (float)(t.subcarrierHz * t.activeS);
    }
    return s;
}

// --- link budget -------------------------------------------------------------

float freeSpaceLossDb(float distM) {
    float d = std::max(distM, 0.05f);
    return 20.0f * std::log10(4 * kPi * d / kVideoWavelength);
}

float groundLossDb(float distM, float txHeightM, float rxHeightM) {
    float bp = 4.0f * std::max(txHeightM, 0.05f) * std::max(rxHeightM, 0.05f) / kVideoWavelength;
    if (distM <= bp) return 0.0f;
    return 0.7f * 20.0f * std::log10(distM / bp);
}

float txAntennaGainDb(Vec3 d) {
    d = normalize(d, {0, 0, -1});
    // up and 30 degrees back
    Vec3 axis{0, 0.866f, -0.5f};
    float c = dot(d, axis);
    float s2 = std::max(1 - c * c, 0.0f);
    return kVideoTxGainDbi + 10.0f * std::log10(std::max(s2, 0.03f));
}

float rxAntennaGainDb(float elevation) {
    float e = std::fabs(elevation);
    // patch: ~65 degree beam, tilted to the horizon
    float patch = kVideoRxPatchDbi - 12.0f * (e / 1.13f) * (e / 1.13f);
    float c = std::cos(e);
    float omni = kVideoRxOmniDbi + 10.0f * std::log10(std::max(c * c, 0.05f));
    return std::max(patch, omni);
}

float fmClicksPerLine(float cnrDb) {
    double rho = std::pow(10.0, cnrDb / 10.0);
    double rate = kIfBandwidthHz / (2.0 * std::sqrt(3.0)) * std::erfc(std::sqrt(std::max(rho, 0.0)));
    return (float)(rate * timing(FPV_VIDEO_NTSC).activeS);
}

// --- the feed ----------------------------------------------------------------

uint32_t VideoFeed::rnd() {
    // xorshift32
    uint32_t x = m_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return m_rng = x ? x : 0x9e3779b9u;
}

float VideoFeed::gauss() {
    float u1 = std::max(uniform(), 1e-7f), u2 = uniform();
    return std::sqrt(-2.0f * std::log(u1)) * std::cos(2 * kPi * u2);
}

void VideoFeed::reset(uint32_t seed) {
    *this = VideoFeed();
    m_rng = seed ? seed * 2654435761u : 1;
    for (Branch &b : m_branch) {
        for (int n = 0; n < 8; n++) {
            b.alpha[n] = 2 * kPi * (n + uniform()) / 8.0f;
            b.phase[n] = 2 * kPi * uniform();
            b.doppler[n] = 0;
        }
    }
}

// Jakes' sum of sinusoids: eight scattered paths arriving from all around,
// each shifted by the Doppler of its angle; the direct path on top with
// K times their power. Normalised to a mean power of 1.
float VideoFeed::branchFadeDb(int branch, double t, float k) const {
    const Branch &b = m_branch[branch & 1];
    float re = 0, im = 0;
    for (int n = 0; n < 8; n++) {
        float a = b.phase[n] + (float)(t * 2 * kPi) * b.doppler[n];
        re += std::cos(a);
        im += std::sin(a);
    }
    float s = std::sqrt(1.0f / (8.0f * (k + 1)));
    float los = std::sqrt(k / (k + 1));
    re = los + re * s;
    im = im * s;
    return 10.0f * std::log10(std::max(re * re + im * im, 1e-5f));
}

void VideoFeed::advanceFades(float dt) {
    for (Branch &b : m_branch) {
        for (int n = 0; n < 8; n++) b.phase[n] = std::fmod(b.phase[n] + 2 * kPi * b.doppler[n] * dt, 2 * kPi);
    }
}

void VideoFeed::update(float dt, const VideoSpec &s, const VideoInputs &in, fpv_video_feed &o) {
    std::memset(&o, 0, sizeof o);
    if (!m_started) {
        reset(0x51f15eedu);
        m_started = true;
    }
    dt = std::clamp(dt, 0.0f, 0.25f);
    m_time += dt;
    m_impact = std::max(m_impact * std::exp(-dt / 0.15f), std::clamp(in.impact, 0.0f, 1.0f));

    o.flags = (in.noise ? FPV_VIDEO_FEED_NOISE : 0u) | (in.resolution ? FPV_VIDEO_FEED_RESOLUTION : 0u);
    if (o.flags && !s.digital) o.flags |= FPV_VIDEO_FEED_OSD_IN_SIGNAL;
    o.system = (uint32_t)s.system;
    o.picture_aspect = s.aspect;
    o.raster_width = (uint32_t)s.rasterW;
    o.raster_height = (uint32_t)s.rasterH;
    o.luma_cycles = s.lumaCycles;
    o.chroma_cycles = s.chromaCycles;
    o.subcarrier_cycles = s.subcarrierCycles;
    o.sharpening = s.sharpening;
    o.osd_style = (uint32_t)in.osdStyle;
    o.rssi_dbm = in.lost ? -120.0f : in.link.rxDbm;
    o.cnr_db = o.rssi_dbm - kVideoNoiseFloorDbm;

    uint64_t field = (uint64_t)std::floor(m_time * s.fieldRate);
    float fieldDt = 1.0f / s.fieldRate;
    // Doppler of each path for this field
    float fd = std::max(in.link.dopplerHz, 1.0f);  // props and the pilot's head move too
    bool fresh = field != m_field || !m_haveField;
    if (fresh) {
        uint64_t elapsed = m_haveField ? std::min<uint64_t>(field - m_field, 1000) : 1;
        advanceFades(fieldDt * (float)elapsed);  // at the old speed, to this field's start
        for (Branch &b : m_branch)
            for (int n = 0; n < 8; n++) b.doppler[n] = fd * std::cos(b.alpha[n]);
        // slow shadowing: log-normal, 2.5 dB, 0.4 s
        float a = std::exp(-fieldDt * (float)elapsed / 0.4f);
        m_shadowDb = m_shadowDb * a + std::sqrt(1 - a * a) * 2.5f * gauss();
        m_field = field;
        m_haveField = true;
        if (s.digital)
            digitalFrame(s, in, fieldDt, o);
        else
            analogField(s, in, fieldDt, o);
        m_last = o;
    } else {
        // same field: same lines, same noise
        uint32_t flags = o.flags;
        o = m_last;
        o.flags = flags;
        o.osd_style = (uint32_t)in.osdStyle;
    }
    o.field = (uint32_t)m_field;
}

void VideoFeed::analogField(const VideoSpec &s, const VideoInputs &in, float fieldDt, fpv_video_feed &o) {
    const AnalogTiming t = timing(s.system);
    o.field_parity = (uint32_t)(m_field & 1);
    o.seed = (float)(m_field % 4096) * 0.618034f + uniform();
    o.line_count = (uint32_t)t.totalLines;
    const bool noise = in.noise;

    // the VTX resets when the battery sags below what it runs on
    float deficit = s.minVoltage > 0 ? std::clamp((s.minVoltage - in.cellVoltage) / 0.25f, 0.0f, 1.0f) : 0.0f;
    if (m_brownout > 0) {
        m_brownout -= fieldDt;
    } else if (deficit > 0 && uniform() < 0.04f * deficit) {
        m_brownout = 0.4f + 0.6f * uniform();
    }
    bool dark = in.lost || m_brownout > 0;
    float mean = dark ? -40.0f : in.link.rxDbm + m_shadowDb - kVideoNoiseFloorDbm;
    float k = std::max(in.link.kFactor, 0.0f);

    // the field's average, for the colour killer and vertical sync
    float fieldCnr = mean + branchFadeDb(m_selected, 0.5 * fieldDt, k);
    if (m_colourKilled ? fieldCnr > 6.0f : fieldCnr < 2.5f) m_colourKilled = !m_colourKilled;
    if (deficit > 0.3f && uniform() < 0.3f * deficit) m_colourKilled = true;  // sagging camera loses its burst
    // vertical sync: lost well below the threshold, the picture rolls until it locks again
    if (m_vLocked && (fieldCnr < -4.0f || dark) && uniform() < 0.3f) {
        m_vLocked = false;
        m_rollSpeed = (uniform() < 0.5f ? -1.0f : 1.0f) * (0.02f + 0.2f * uniform());
    } else if (!m_vLocked && fieldCnr > -1.0f && !dark && uniform() < 0.4f) {
        m_vLocked = true;
    }
    if (!noise) m_vLocked = true;
    if (m_vLocked) {
        m_roll = 0;
    } else {
        m_rollSpeed += 0.01f * gauss();
        m_roll = m_roll + m_rollSpeed;
        m_roll -= std::floor(m_roll);
    }
    o.roll = m_roll;
    // a hard hit jolts the camera cable: the frame jumps for a field
    if (noise && m_vLocked && m_impact > 0.4f && uniform() < m_impact * 0.5f) o.roll = (gauss() * 0.04f);
    if (o.roll < 0) o.roll += 1.0f;
    // NTSC: the decoder's hue drifts on a weak burst
    m_hue = m_hue * 0.95f + 0.05f * gauss() * 0.6f * smoothstep(10.0f, 0.0f, fieldCnr);

    // motor and ESC noise on the VTX's power
    float load = std::pow(std::clamp(in.throttle, 0.0f, 1.0f), 1.2f);
    float esc = noise ? s.powerNoise * (0.012f + 0.06f * load) : 0.0f;
    const double pwmHz = 48000.0;
    o.interference[0] = esc;
    o.interference[1] = (float)(pwmHz * t.activeS);
    double step = pwmHz * t.lineS;
    o.interference[2] = (float)(step - std::floor(step));
    m_pwmPhase = std::fmod(m_pwmPhase + (float)(pwmHz * fieldDt) + 0.37f * uniform(), 1.0f);
    o.interference[3] = m_pwmPhase;
    float hum = noise ? s.powerNoise * 0.03f * load : 0.0f;

    // the lines
    for (int l = 0; l < t.totalLines; l++) {
        uint8_t *row = o.lines + l * 8;
        if (!noise) {
            float shift = 0;
            uint16_t sh = (uint16_t)std::lround((shift + 0.25f) / 0.5f * 65535.0f);
            row[0] = 0, row[1] = 0, row[2] = 255, row[3] = 255;
            row[4] = (uint8_t)(sh >> 8), row[5] = (uint8_t)(sh & 255), row[6] = 128, row[7] = 128;
            continue;
        }
        double tl = l * t.lineS;
        // diversity: the stronger antenna, switched with a little hysteresis
        float f0 = branchFadeDb(0, tl, k), f1 = branchFadeDb(1, tl, k);
        float cur = m_selected ? f1 : f0, other = m_selected ? f0 : f1;
        bool switched = false;
        if (other > cur + 2.0f) {
            m_selected ^= 1;
            cur = other;
            switched = true;
        }
        float c = mean + cur;
        float clicks = fmClicksPerLine(c) * (1 + 3 * m_impact);
        float density = std::min(clicks / kClickCells, 0.6f);
        float grain = 1.5f * std::pow(10.0f, -(c + 26.0f) / 20.0f) + 0.32f * smoothstep(9.0f, -6.0f, c) + 0.06f * m_impact;
        float picture = smoothstep(-9.0f, 1.0f, c);
        float chroma = m_colourKilled ? 0.0f : smoothstep(1.0f, 9.0f, c);
        float hue = m_hue + gauss() * 0.3f * smoothstep(12.0f, 0.0f, c);
        // horizontal sync: a click on the sync pulse or a weak line kicks
        // the line's start; the PLL pulls back over the next lines
        float kick = std::min(1.0f, clicks * 0.09f + 0.25f * smoothstep(3.0f, -6.0f, c) + 0.15f * m_impact);
        // the antenna switch glitches the line, visibly only on a weak signal
        float glitch = switched ? smoothstep(22.0f, 8.0f, c) : 0.0f;
        if (glitch > 0) kick = std::max(kick, 0.6f * glitch);
        if (uniform() < kick) m_hShift += gauss() * (0.006f + 0.05f * smoothstep(4.0f, -8.0f, c));
        m_hShift *= 0.86f;
        float shift = m_hShift + gauss() * 0.0012f * smoothstep(14.0f, 2.0f, c);
        shift = std::clamp(shift, -0.24f, 0.24f);
        float lumaOffset = 0;
        for (int m = 0; m < 4; m++) {
            double ph = m_humPhase[m] + in.motorHz[m] * tl;
            lumaOffset += hum * 0.5f * (float)std::sin(2 * kPi * (ph - std::floor(ph)));
        }
        if (glitch > 0) lumaOffset += glitch * (uniform() < 0.5f ? 0.08f : -0.06f);
        // blanking lines keep the signal's picture value: black with a signal, snow without
        uint16_t sh = (uint16_t)std::lround((shift + 0.25f) / 0.5f * 65535.0f);
        row[0] = byteOf(grain / 0.5f);
        row[1] = byteOf(std::cbrt(density));
        row[2] = byteOf(chroma);
        row[3] = byteOf(picture);
        row[4] = (uint8_t)(sh >> 8);
        row[5] = (uint8_t)(sh & 255);
        row[6] = byteOf(lumaOffset + 0.5f);
        hue = std::remainder(hue, 2 * kPi);
        row[7] = byteOf(hue / (2 * kPi) + 0.5f);
    }
    for (int m = 0; m < 4; m++) {
        double ph = m_humPhase[m] + in.motorHz[m] * fieldDt;
        m_humPhase[m] = (float)(ph - std::floor(ph));
    }
}

void VideoFeed::digitalFrame(const VideoSpec &s, const VideoInputs &in, float frameDt, fpv_video_feed &o) {
    (void)frameDt;
    o.field_parity = 0;
    o.seed = (float)(m_field % 4096) * 0.618034f + uniform();
    o.line_count = 0;
    if (!in.noise) return;
    float k = std::max(in.link.kFactor, 0.0f);
    float fade = std::max(branchFadeDb(0, 0, k), branchFadeDb(1, 0, k));  // diversity
    float margin = in.lost ? -60.0f : in.link.rxDbm + m_shadowDb + fade - s.sensitivityDbm;
    float deficit = s.minVoltage > 0 ? std::clamp((s.minVoltage - in.cellVoltage) / 0.25f, 0.0f, 1.0f) : 0.0f;
    if (deficit > 0) margin -= 20.0f * deficit;
    float errors = 1.0f / (1.0f + std::exp(margin / 1.2f));
    if (m_impact > 0.3f) errors = std::max(errors, 0.05f * m_impact);
    if (s.freezes) {
        // DJI / Walksnail: the bitrate drops first (mush), then the frame
        // stutters and holds
        o.softness = smoothstep(10.0f, 0.0f, margin);
        o.block_errors = errors * 0.25f;
        if (m_freezeFrames > 0) {
            m_freezeFrames--;
        } else if (uniform() < smoothstep(1.0f, -4.0f, margin)) {
            m_freezeFrames = 2 + (int)(uniform() * 10);
        }
        o.frozen = m_freezeFrames > 0 ? 1u : 0u;
    } else {
        // HDZero: no buffering, the picture breaks into blocks, then holds
        o.softness = 0;
        o.block_errors = errors;
        o.frozen = margin < -8.0f ? 1u : 0u;
    }
}

}  // namespace fpv
