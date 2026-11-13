// libfpv - the video link: link budget (simulation thread) and the feed the
// goggles show (host thread), see docs/DECISIONS.md D34.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>

#include "fpv_math.h"
#include "libfpv.h"

namespace fpv {

struct DronePreset;

// 5.8 GHz band centre
constexpr float kVideoWavelength = 299792458.0f / 5.8e9f;  // m
// Thermal noise in a video FM receiver's IF bandwidth (~18 MHz) with its
// noise figure: the carrier-to-noise ratio is measured against this.
// ImmersionRC measured first snow at about -94 dBm (the FM threshold, CNR
// ~10 dB) and lost sync at about -106 dBm on a good receiver.
constexpr float kVideoNoiseFloorDbm = -104.0f;
// What a real installation loses against the ideal free-space budget:
// polarisation mismatch, the frame and battery in the way, connectors,
// detuned antennas. Puts a 25 mW VTX's first sparkles near 1 km in the open.
constexpr float kVideoSystemLossDb = 10.0f;
constexpr float kVideoTxGainDbi = 2.1f;   // dipole / lollipop, at its best
constexpr float kVideoRxPatchDbi = 8.0f;  // goggles: a patch facing the drone...
constexpr float kVideoRxOmniDbi = 2.1f;   // ...and an omni, diversity
constexpr float kVideoRxHeight = 1.7f;    // the goggles above the pilot's feet, m

// The obstruction probe's verdict (D34): the direct ray blocked (centre),
// how many of the four paths through the first Fresnel zone's edge are
// (offsets), and the obstacle's depth along the direct ray. Gives the loss
// over the free-space budget and the Rician K factor.
inline void videoObstruction(bool centre, int offsets, float depth, float &lossDb, float &kFactor) {
    float f = (centre ? 0.5f : 0.0f) + 0.125f * (float)offsets;
    // a wall costs ~12 dB at 5.8 GHz, a building's depth much more
    lossDb = f * std::min(12.0f + 0.6f * std::max(depth, 0.0f), 45.0f);
    kFactor = 0.3f + 9.7f * (1 - f) * (1 - f);  // blocked: only scattered paths remain
}

// What the preset says about the camera and the transmitter.
struct VideoSpec {
    bool digital = false;
    int system = FPV_VIDEO_NTSC;
    float aspect = 4.0f / 3.0f;
    int rasterW = 720, rasterH = 480;
    float fieldRate = 59.94f;        // fields (analog) or frames (digital) per second
    float lumaCycles = 250, chromaCycles = 53, subcarrierCycles = 188.5f, sharpening = 0.5f;
    float powerDbm = 14.0f;          // 25 mW
    float powerNoise = 0.15f;        // 0..1: motor/ESC noise on the VTX's power
    float minVoltage = 0;            // V, below it the VTX/camera browns out (0 = never)
    bool freezes = false;            // digital: freezes (DJI, Walksnail) or breaks up (HDZero)
    float sensitivityDbm = -95.0f;   // digital: where the picture starts to break up
};
VideoSpec videoSpec(const DronePreset &p);

// --- the link budget (pure, tested) ---------------------------------------
// Shared with the RC link (rc/rc_link.h), which passes its own wavelength.
float freeSpaceLossDb(float distM, float wavelengthM = kVideoWavelength);
// Ground reflection: beyond the two-ray breakpoint (4 h1 h2 / lambda) the
// loss grows at 40 dB/decade instead of 20 (rough ground, 0.7 of it).
float groundLossDb(float distM, float txHeightM, float rxHeightM, float wavelengthM = kVideoWavelength);
// The VTX antenna, on the back of the frame pointing up and back: a dipole's
// doughnut with its null along the antenna (drone-local direction to the
// pilot, X right, Y up, Z forward).
float txAntennaGainDb(Vec3 dirToPilotLocal);
// The goggles' diversity: a patch facing the drone's bearing and an omni
// standing up; elevation of the drone seen from the pilot, radians. Right
// overhead both are weak.
float rxAntennaGainDb(float elevationRad);
// FM clicks (sparklies) per active line below the threshold, after Rice:
// rate = B / (2 sqrt 3) * erfc(sqrt(CNR)), B the IF bandwidth.
float fmClicksPerLine(float cnrDb);

// Slowly varying link state from the simulation thread.
struct LinkState {
    float rxDbm = -40;     // mean received power (no fast fading)
    float kFactor = 10;    // Rician K: direct path vs. scattered power, linear
    float dopplerHz = 0;   // the drone's speed / wavelength: how fast fades go by
};

// What the host-thread feed needs besides the link.
struct VideoInputs {
    LinkState link;
    bool lost = false;          // no transmitter (destroyed)
    bool noise = false, resolution = false;
    int osdStyle = FPV_OSD_STYLE_ANALOG;
    float cellVoltage = 4.0f;
    float throttle = 0;         // mean motor command 0..1
    float motorHz[4] = {0, 0, 0, 0};  // electrical frequency, Hz
    float impact = 0;           // a hit just now, 0..1 (cables and antenna jolted)
};

// Fades, sync and the per-line table, advanced on the host's clock.
class VideoFeed {
public:
    void reset(uint32_t seed = 1);
    // Advances by dt seconds of host time and fills `out`.
    void update(float dt, const VideoSpec &spec, const VideoInputs &in, fpv_video_feed &out);

    // for tests: the fade of each diversity branch at time t of the current field, dB
    float branchFadeDb(int branch, double t, float kFactor) const;

private:
    struct Branch {
        float alpha[8];   // arrival angle of each scattered path
        float phase[8];   // its phase at the start of the current field
        float doppler[8]; // f_d cos(alpha) this field, Hz
    };
    uint32_t rnd();
    float uniform() { return (rnd() >> 8) * (1.0f / 16777216.0f); }
    float gauss();
    void analogField(const VideoSpec &s, const VideoInputs &in, float fieldDt, fpv_video_feed &o);
    void digitalFrame(const VideoSpec &s, const VideoInputs &in, float frameDt, fpv_video_feed &o);
    void advanceFades(float dt);

    uint32_t m_rng = 1;
    double m_time = 0;           // host seconds
    uint64_t m_field = 0;        // fields (frames) received
    bool m_started = false, m_haveField = false;
    fpv_video_feed m_last{};     // the current field, repeated until the next
    Branch m_branch[2];
    int m_selected = 0;          // the diversity's chosen antenna
    float m_shadowDb = 0;        // slow log-normal shadowing
    float m_hShift = 0;          // horizontal PLL error carried from line to line
    bool m_colourKilled = false;
    bool m_vLocked = true;
    float m_roll = 0, m_rollSpeed = 0;
    float m_impact = 0;
    float m_brownout = 0;        // seconds left of a VTX reset
    float m_pwmPhase = 0, m_humPhase[4] = {0, 0, 0, 0};
    int m_freezeFrames = 0;
    float m_hue = 0;             // NTSC phase drift
};

}  // namespace fpv
