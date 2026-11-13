// libfpv - the RC control link (D36).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "rc/rc_link.h"

#include <algorithm>
#include <cmath>

namespace fpv {

namespace {

// fpv_rc_link_mode order. Sensitivity limits and RFMD from expresslrs.org
// "Signal Health" (ELRS 4.x table); the SNR a LoRa packet needs from the
// spreading factor each rate uses (SX1280 / SX127x datasheets: SF5 -2.5 dB,
// each step -2.5 dB); dynamic power SNR thresholds from "Dynamic Transmit
// Power" (500/250/150 Hz and 900 MHz 200 Hz given there, the slower rates
// follow the same spacing below their SNR floor).
const RcLinkMode kModes[FPV_RC_LINK_MODE_COUNT] = {
    {"2.4 GHz F1000 (race)", RcBand::Ghz24, 1000, -104, 0, true, 0, 0, 33},
    {"2.4 GHz 500 Hz (freestyle)", RcBand::Ghz24, 500, -105, -2.5f, false, 5.0f, 9.5f, 29},
    {"2.4 GHz 250 Hz", RcBand::Ghz24, 250, -108, -5.0f, false, 3.0f, 9.5f, 27},
    {"2.4 GHz 150 Hz", RcBand::Ghz24, 150, -112, -7.5f, false, 0.0f, 8.5f, 24},
    {"2.4 GHz 50 Hz (long range)", RcBand::Ghz24, 50, -115, -10.0f, false, -2.5f, 6.0f, 21},
    {"900 MHz 200 Hz", RcBand::Mhz900, 200, -112, -5.0f, false, 1.0f, 3.0f, 5},
    {"900 MHz 100 Hz", RcBand::Mhz900, 100, -117, -7.5f, false, -1.5f, 2.0f, 2},
    {"900 MHz 50 Hz (long range)", RcBand::Mhz900, 50, -120, -10.0f, false, -4.0f, 1.0f, 1},
    {"900 MHz 25 Hz (long range)", RcBand::Mhz900, 25, -123, -12.5f, false, -6.5f, 0.0f, 0},
};

// FLRC dynamic power: RSSI thresholds, the same for every FLRC rate
constexpr float kFlrcRssiUp = -89.0f, kFlrcRssiDown = -83.0f;
// The highest SNR the radios report (LoRa's SNR estimate saturates)
constexpr float kSnrCeiling = 13.0f;
// An ELRS receiver gives up the connection after this long without a
// packet and then hops searching for the transmitter's sync packets.
constexpr float kDisconnectS = 1.0f;
// Back in range, finding the sync again takes about this long of good link.
constexpr float kResyncS = 0.3f;

float smoothstep(float e0, float e1, float x) {
    float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3 - 2 * t);
}

}  // namespace

const RcLinkMode &rcLinkMode(int mode) { return kModes[std::clamp(mode, 0, FPV_RC_LINK_MODE_COUNT - 1)]; }

int rcPowerLevel(int mw) {
    int level = 0;
    for (int i = 0; i < kRcPowerLevelCount; i++)
        if (mw >= kRcPowerLevels[i]) level = i;
    return level;
}

float rcObstructionDb(RcBand band, bool centre, int offsets, float depth) {
    float f = (centre ? 0.5f : 0.0f) + 0.125f * (float)offsets;
    depth = std::max(depth, 0.0f);
    // a wall: drywall 2-4 dB, brick 3-8, concrete 10-22 at 2.4 GHz; about
    // half at 900 MHz. Along the depth (rooms, foliage): ITU-R P.833's
    // 0.34 dB/m at 2.1 GHz and 0.17 dB/m at 950 MHz, capped like its A_m.
    if (band == RcBand::Ghz24) return f * std::min(8.0f + 0.35f * depth, 35.0f);
    return f * std::min(4.0f + 0.17f * depth, 25.0f);
}

float rcRxAntennaGainDb(RcBand band, Vec3 d) {
    d = normalize(d, {0, 0, -1});
    // a dipole at the back of the frame, pointing back and down
    Vec3 axis{0, -0.5f, -0.866f};
    float c = dot(d, axis);
    float s2 = std::max(1 - c * c, 0.05f);
    // the frame, battery and stack between the antenna and a pilot ahead
    float shadow = (band == RcBand::Ghz24 ? 8.0f : 3.0f) * smoothstep(0.0f, 0.8f, d.z);
    return kRcRxGainDbi + 10.0f * std::log10(s2) - shadow;
}

float RcLink::packetSuccess(float marginDb) {
    // LoRa / FLRC packet error rate vs signal: a steep waterfall about 2 dB
    // wide around the sensitivity limit (84 % there, 99 % 2 dB above)
    return 1.0f / (1.0f + std::exp(-(marginDb + 1.0f) / 0.6f));
}

uint32_t RcLink::rnd() {
    uint32_t x = m_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return m_rng = x ? x : 0x9e3779b9u;
}

float RcLink::gauss() {
    float u1 = std::max(uniform(), 1e-7f), u2 = uniform();
    return std::sqrt(-2.0f * std::log(u1)) * std::cos(2 * kPi * u2);
}

void RcLink::reset(const RcLinkConfig &cfg, uint32_t seed) {
    *this = RcLink();
    m_rng = seed ? seed * 2654435761u : 1;
    m_cfg = cfg;
    m_cfg.mode = std::clamp(cfg.mode, 0, FPV_RC_LINK_MODE_COUNT - 1);
    for (uint8_t &h : m_history) h = 1;
    m_histSum = 100;
    // dynamic power starts at the lowest level, fixed power at the maximum
    setPower(m_cfg.dynamicPower ? 0 : rcPowerLevel(m_cfg.maxPowerMw));
}

void RcLink::configure(const RcLinkConfig &cfg) {
    RcLinkConfig c = cfg;
    c.mode = std::clamp(cfg.mode, 0, FPV_RC_LINK_MODE_COUNT - 1);
    if (c.mode != m_cfg.mode || c.enabled != m_cfg.enabled) {
        reset(c, m_rng);
        return;
    }
    m_cfg = c;
    int maxLevel = rcPowerLevel(m_cfg.maxPowerMw);
    setPower(m_cfg.dynamicPower ? std::min(m_level, maxLevel) : maxLevel);
}

void RcLink::setPower(int level) {
    m_level = std::clamp(level, 0, rcPowerLevel(m_cfg.maxPowerMw));
    m_st.txPowerMw = kRcPowerLevels[m_level];
}

int RcLink::step(float dt, const RcLinkInputs &in) {
    if (!m_cfg.enabled) return packet(in) ? 1 : 0;
    const float packetDt = 1.0f / rcLinkMode(m_cfg.mode).rateHz;
    int received = 0;
    m_clock += std::clamp(dt, 0.0f, 0.25f);
    while (m_clock >= packetDt) {
        m_clock -= packetDt;
        received += sendPacket(in, packetDt) ? 1 : 0;
    }
    return received;
}

bool RcLink::packet(const RcLinkInputs &in, float elapsedS) {
    if (!m_cfg.enabled) {
        m_st = RcLinkStatus{};
        m_st.txPowerMw = kRcPowerLevels[rcPowerLevel(m_cfg.maxPowerMw)];
        return true;
    }
    float interval = 1.0f / rcLinkMode(m_cfg.mode).rateHz;
    return sendPacket(in, elapsedS > 0 ? std::min(elapsedS, 0.25f) : interval);
}

bool RcLink::sendPacket(const RcLinkInputs &in, float packetDt) {
    const RcLinkMode &mode = rcLinkMode(m_cfg.mode);
    // fading: the scattered paths' sum as a Gauss-Markov process with the
    // Doppler's coherence time (Clarke: 0.423 / f_d), the direct path on top
    // with K times their power; slow log-normal shadowing, 2 dB over 0.5 s
    float tc = 0.423f / std::max(in.dopplerHz, 0.5f);
    float rho = std::exp(-packetDt / tc);
    float w = std::sqrt(std::max(1 - rho * rho, 0.0f) * 0.5f);
    m_re = rho * m_re + w * gauss();
    m_im = rho * m_im + w * gauss();
    float a = std::exp(-packetDt / 0.5f);
    m_shadowDb = a * m_shadowDb + std::sqrt(1 - a * a) * 2.0f * gauss();
    float k = std::max(in.kFactor, 0.0f);
    float los = std::sqrt(k / (k + 1)), s = std::sqrt(1 / (k + 1));
    float re = los + s * m_re, im = s * m_im;
    float fadeDb = 10.0f * std::log10(std::max(re * re + im * im, 1e-5f)) + m_shadowDb;

    float txDbm = 10.0f * std::log10((float)m_st.txPowerMw);
    float rxDbm = in.alive ? txDbm + in.pathGainDb + fadeDb : -200.0f;
    bool ok = uniform() < packetSuccess(rxDbm - mode.sensitivityDbm);

    if (!m_st.connected) {
        // searching: only a run of good link finds the sync again
        if (ok) m_searchLeft -= packetDt;
        if (m_searchLeft <= 0) {
            m_st.connected = true;
            m_sinceRx = 0;
            m_telemAge = 0;
        }
        ok = false;
    }
    m_histSum -= m_history[m_histPos];
    m_history[m_histPos] = ok ? 1 : 0;
    m_histSum += m_history[m_histPos];
    m_histPos = (m_histPos + 1) % 100;
    m_st.lq = (float)m_histSum;
    if (ok) {
        m_sinceRx = 0;
        m_st.lostInRow = 0;
        float noiseFloor = mode.sensitivityDbm - mode.snrAtSensitivity;
        float snr = std::min(rxDbm - noiseFloor + 0.5f * gauss(), kSnrCeiling);
        float e = 1 - std::exp(-packetDt / 0.1f);
        m_rssiAvg += (std::max(rxDbm, -130.0f) - m_rssiAvg) * e;
        m_snrAvg += (snr - m_snrAvg) * e;
        m_st.rssiDbm = m_rssiAvg;
        m_st.snrDb = mode.flrc ? 0.0f : m_snrAvg;
    } else {
        m_st.lostInRow++;
        if (m_st.connected) {
            m_sinceRx += packetDt;
            if (m_sinceRx > kDisconnectS) {
                m_st.connected = false;
                m_searchLeft = kResyncS;
            }
        }
    }
    if (!m_st.connected) {
        m_st.rssiDbm = m_rssiAvg = -130.0f;
        m_st.snrDb = m_snrAvg = -30.0f;
    }

    m_telemClock += packetDt;
    m_telemAge += packetDt;
    if (m_telemClock >= 1.0f / kRcTelemetryHz) {
        m_telemClock -= 1.0f / kRcTelemetryHz;
        telemetry(in, rxDbm - txDbm);
    }
    return ok;
}

// A telemetry slot: the receiver answers with its link statistics, which
// the transmitter's dynamic power reads (expresslrs.org, Dynamic Transmit
// Power): one level down on a strong signal at LQ >= 95, one up on a weak
// one or LQ < 85, straight to the maximum on LQ < 50, a sudden LQ drop or
// telemetry lost while armed; the first missed telemetry changes nothing,
// each one after the second raises a level.
void RcLink::telemetry(const RcLinkInputs &in, float pathDb) {
    const RcLinkMode &mode = rcLinkMode(m_cfg.mode);
    const int maxLevel = rcPowerLevel(m_cfg.maxPowerMw);
    bool got = false;
    if (m_st.connected && in.alive) {
        // the same path backwards (reciprocal), at the receiver's power
        float dlMw = std::min((float)m_st.txPowerMw, kRcRxTelemetryMaxMw);
        float dlDbm = 10.0f * std::log10(dlMw) + pathDb;
        got = uniform() < packetSuccess(dlDbm - mode.sensitivityDbm);
    }
    if (!m_cfg.dynamicPower) {
        setPower(maxLevel);
        return;
    }
    if (!got) {
        m_telemMissed++;
        if (in.armed && (!m_st.connected || m_telemAge > 1.0f)) {
            setPower(maxLevel);  // telemetry lost while armed
        } else if (in.armed && m_telemMissed > 2) {
            setPower(m_level + 1);
        }
        return;
    }
    m_telemMissed = 0;
    m_telemAge = 0;
    float lq = m_st.lq;
    bool sudden = lq < m_lqAvg - 15.0f;
    m_lqAvg += (lq - m_lqAvg) * 0.3f;
    if (lq < 50 || sudden) {
        setPower(maxLevel);
        return;
    }
    bool up, down;
    if (mode.flrc) {
        up = m_st.rssiDbm <= kFlrcRssiUp;
        down = m_st.rssiDbm >= kFlrcRssiDown;
    } else {
        up = m_st.snrDb <= mode.snrUp;
        down = m_st.snrDb >= mode.snrDown;
    }
    if (up)
        setPower(m_level + 1);
    else if (down && lq >= 95)
        setPower(m_level - 1);
    else if (lq < 85)
        setPower(m_level + 1);
}

}  // namespace fpv
