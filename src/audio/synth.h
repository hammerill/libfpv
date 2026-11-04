// libfpv - procedural drone sound, pulled from the host's audio thread.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

namespace fpv {

// How the drone reaches the ear when it is heard from the pilot (see
// docs/DECISIONS.md "Hearing the drone from the pilot"). Computed on the
// host thread from the geometry, a few times a second for the walls and
// echoes, every frame for the rest.
struct Hearing {
    float fromPilot = 0;      // 0 onboard .. 1 from the pilot (glides)
    float delay = 0;          // s, the sound's travel time drone -> ear (Doppler from its change)
    float gain = 1;           // spherical spreading, and the walls' broadband loss
    float airCutoff = 20000;  // Hz, air absorption over the distance
    float wallCutoff = 20000; // Hz, what gets through or around the walls
    float pan = 0;            // -1 left .. 1 right of the ear
    float groundDelay = 0;    // s, extra path of the reflection off the ground
    float groundGain = 0;
    float echoDelay = 0;      // s, extra path of a slap echo off walls near the ear
    float echoGain = 0;
    float reverb = 0;         // send into the reverberation
    float reverbTime = 0.5f;  // RT60, s
    float onboardReverb = 0;  // send when heard onboard (a room around the drone)
};

struct AudioParams {
    float rpm[4] = {0, 0, 0, 0};
    float load[4] = {0, 0, 0, 0};      // 0..1, from current draw
    float damage[4] = {0, 0, 0, 0};    // prop damage 0..1
    float pan[4] = {0, 0, 0, 0};       // -1 left .. 1 right, onboard
    float maxRpm = 30000;
    float blades = 3;
    bool beeper = false;     // Betaflight's buzzer (arming, low battery, failsafe...)
    bool motorBeep = false;  // the pilot's beeper switch: beeps through the motors (DShot beacon)
    bool paused = false;
    Hearing hearing;
};

class AudioSynth {
public:
    AudioSynth();
    void configure(uint32_t sampleRate, uint32_t channels);
    void setParams(const AudioParams &p);
    // The motors (with the beeper switch's beeps) and the buzzer.
    void setVolume(float motors, float beeps);
    // Interleaved float samples; always writes `frames` frames.
    uint32_t read(float *out, uint32_t frames);

private:
    float delayTap(double delaySamples) const;

    std::mutex m_mutex;  // short critical sections only (parameter copy)
    AudioParams m_target, m_cur;
    uint32_t m_rate = 48000, m_channels = 2;
    float m_volume = 0.1f, m_beepVolume = 0.1f;
    double m_phase[4] = {0, 0, 0, 0};
    double m_whine[4] = {0, 0, 0, 0};
    double m_wobble[4] = {0, 0, 0, 0};
    float m_noiseLp[4] = {0, 0, 0, 0};
    double m_beepPhase = 0, m_motorBeepPhase = 0;
    float m_beepEnv = 0, m_motorBeepEnv = 0;  // short ramps, so beeps don't click
    uint32_t m_seed = 0x12345678u;

    // from the pilot: the drone's mono signal travels through a delay line
    std::vector<float> m_line;  // power of two
    uint32_t m_write = 0;
    // The read points, in samples, each smoothed per sample by three one-pole
    // stages: the host updates the targets once a frame (a staircase), and
    // following each step within one audio block made the pitch wobble.
    struct Smoothed {
        double stage[2] = {0, 0};
        double value = 0;
    };
    double m_delay = -1;  // the direct sound (Smoothed's value); < 0 = not started
    double m_delayStage[2] = {0, 0};
    Smoothed m_ground, m_echo;  // the extra delays of the ground's reflection and the slap echo
    float m_groundLp = 0;  // the ground keeps the lows of its reflection
    float m_air = 0, m_wall1 = 0, m_wall2 = 0;  // one-pole low-pass states
    float m_mix = 0;      // glides to hearing.fromPilot
    // reverberation: Schroeder/Freeverb, four damped combs and two all-passes
    static constexpr int kCombs = 4, kAllpasses = 2;
    std::vector<float> m_comb[kCombs], m_allpass[kAllpasses];
    uint32_t m_combLen[kCombs] = {}, m_allpassLen[kAllpasses] = {};
    uint32_t m_combPos[kCombs] = {}, m_allpassPos[kAllpasses] = {};
    float m_combLp[kCombs] = {};
};

}  // namespace fpv
