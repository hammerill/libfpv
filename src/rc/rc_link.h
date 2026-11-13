// libfpv - the RC control link: an ExpressLRS link budget between the
// pilot's radio and the drone's receiver, packet by packet, with ELRS's
// link quality, RSSI, SNR and dynamic transmit power (docs/DECISIONS.md D36).
// Pure: no Betaflight, no world; Flight feeds it the path and takes packets.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

#include "fpv_math.h"
#include "libfpv.h"

namespace fpv {

enum class RcBand { Ghz24 = 0, Mhz900 = 1 };

// One ELRS packet rate (expresslrs.org, Signal Health: RF mode indexes and
// sensitivity limits; Dynamic Transmit Power: SNR thresholds).
struct RcLinkMode {
    const char *name;        // as in the overlay
    RcBand band;
    float rateHz;            // packets per second
    float sensitivityDbm;    // ELRS's sensitivity limit
    float snrAtSensitivity;  // the SNR a packet needs (LoRa: spreading factor; FLRC: n/a)
    bool flrc;               // FLRC: dynamic power works on RSSI, no SNR
    float snrUp, snrDown;    // dynamic power: raise at SNR <= up, lower at >= down
    uint8_t rfMode;          // RFMD the OSD's link quality element shows (ELRS 4.x)
};

// fpv_rc_link_mode order
const RcLinkMode &rcLinkMode(int mode);

// ELRS power levels, mW
constexpr int kRcPowerLevels[] = {10, 25, 50, 100, 250, 500, 1000};
constexpr int kRcPowerLevelCount = 7;
// The level index of a power (rounded down to a level, at least 10 mW).
int rcPowerLevel(int mw);

constexpr float kRcWavelength24 = 299792458.0f / 2.44e9f;  // m
constexpr float kRcWavelength900 = 299792458.0f / 915e6f;  // m
inline float rcWavelength(RcBand b) { return b == RcBand::Ghz24 ? kRcWavelength24 : kRcWavelength900; }
// The radio held at the pilot's chest, above his feet, m
constexpr float kRcTxHeight = 1.3f;
// Antennas: the radio's dipole, the receiver's dipole on the frame
constexpr float kRcTxGainDbi = 2.0f;
constexpr float kRcRxGainDbi = 2.0f;
// What a real installation loses against the ideal budget: polarisation
// mismatch, detuning by the carbon frame, the pilot's hands and body round
// the radio. Puts a 100 mW 2.4 GHz 500 Hz link's sensitivity limit at
// about 4.5 km in the open with the drone 30 m up (fades take packets
// earlier), within the few km pilots report.
constexpr float kRcSystemLossDb = 8.0f;
// The receiver's own telemetry (downlink) power cap, mW: ELRS receivers
// send at the transmitter's power, up to what their amplifier does.
constexpr float kRcRxTelemetryMaxMw = 100.0f;
// The telemetry the receiver sends back (the TX's dynamic power reads it), Hz
constexpr float kRcTelemetryHz = 4.0f;

// What stands between the radio and the drone, from the video link's
// obstruction probe (the same rays, D34): the direct ray blocked (centre),
// how many of the four Fresnel-zone paths are, the obstacle's depth along
// the direct ray (m). Walls and foliage cost less at 2.4 GHz than at
// 5.8 GHz, and less again at 900 MHz.
float rcObstructionDb(RcBand band, bool centre, int offsets, float depth);
// The receiver's antenna and the frame around it: a dipole at the back of
// the frame, pointing back and down; the frame, battery and stack shadow it
// forward. Drone-local direction to the pilot (X right, Y up, Z forward).
float rcRxAntennaGainDb(RcBand band, Vec3 dirToPilotLocal);

struct RcLinkConfig {
    bool enabled = false;  // off: a perfect link, the way libfpv always flew
    int mode = FPV_RC_LINK_2G4_500HZ;
    int maxPowerMw = 250;
    bool dynamicPower = true;
};

// What the drone and the world give the link each step.
struct RcLinkInputs {
    float pathGainDb = -60;  // everything between the TX's output and the RX's input
    float kFactor = 10;      // Rician K (linear): direct vs scattered power
    float dopplerHz = 0;     // speed / wavelength
    bool armed = false;
    bool alive = true;       // false: no receiver (destroyed)
};

// Where the link is.
struct RcLinkStatus {
    bool connected = true;
    float lq = 100;          // % of the last 100 packets
    float rssiDbm = -50;     // the receiver's averaged RSSI
    float snrDb = 10;
    int txPowerMw = 10;
    int lostInRow = 0;       // packets
};

class RcLink {
public:
    void reset(const RcLinkConfig &cfg, uint32_t seed = 1);
    // A new configuration; the link keeps its state unless the mode changed.
    void configure(const RcLinkConfig &cfg);
    const RcLinkConfig &config() const { return m_cfg; }
    // Advances dt seconds on the link's own packet clock. Returns how many
    // packets the receiver took in that time (0: Betaflight gets nothing
    // new, the sticks stay where the last packet put them). Off: always 1.
    int step(float dt, const RcLinkInputs &in);
    // One packet, sent now by a packet clock outside (SimITL's receiver),
    // `elapsedS` after the previous one (<= 0: the mode's interval): true
    // if the receiver takes it. Off: always true.
    bool packet(const RcLinkInputs &in, float elapsedS = 0);
    const RcLinkStatus &status() const { return m_st; }

    // for tests: a packet's chance at this margin over the sensitivity (dB)
    static float packetSuccess(float marginDb);

private:
    uint32_t rnd();
    float uniform() { return (rnd() >> 8) * (1.0f / 16777216.0f); }
    float gauss();
    bool sendPacket(const RcLinkInputs &in, float packetDt);
    void telemetry(const RcLinkInputs &in, float pathDb);
    void setPower(int level);

    RcLinkConfig m_cfg;
    RcLinkStatus m_st;
    uint32_t m_rng = 1;
    double m_clock = 0;          // seconds into the current packet interval
    uint8_t m_history[100] = {}; // the last 100 packets, 1 = received
    int m_histPos = 0, m_histSum = 100;
    // fading: the scattered paths' complex sum (Gauss-Markov), shadowing
    float m_re = 0, m_im = 0, m_shadowDb = 0;
    // the receiver's averages
    float m_rssiAvg = -50, m_snrAvg = 10;
    // ELRS receiver sync: lost after the disconnect timeout, then it
    // searches until a sync packet gets through
    float m_sinceRx = 0, m_searchLeft = 0;
    // the transmitter's dynamic power
    int m_level = 0;
    float m_telemClock = 0;
    int m_telemMissed = 0;
    float m_lqAvg = 100;
    float m_telemAge = 0;        // since the last telemetry packet, s
};

}  // namespace fpv
