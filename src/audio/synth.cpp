// libfpv - procedural drone sound.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "audio/synth.h"

#include <algorithm>
#include <cmath>

namespace fpv {

namespace {
constexpr double kTwoPi = 6.283185307179586;
constexpr int kHarmonics = 6;
constexpr float kPolePairs = 7.0f;  // typical 12N14P FPV motor
constexpr float kBuzzerHz = 2700;   // Betaflight's piezo buzzer
constexpr float kBeaconHz = 1760;   // the ESCs' beacon tone through the motor windings
// 2 s of travel (690 m) at up to 192 kHz; farther away the drone is inaudible
constexpr uint32_t kLineSize = 1u << 19;
constexpr float kMaxDelay = 2.0f;
// Freeverb's comb and all-pass lengths at 44.1 kHz
constexpr uint32_t kCombLen[] = {1116, 1188, 1277, 1356};
constexpr uint32_t kAllpassLen[] = {556, 441};

float onePole(float cutoff, uint32_t rate) {
    return 1.0f - std::exp(-(float)kTwoPi * std::min(cutoff, rate * 0.45f) / rate);
}
}  // namespace

// Every buffer is allocated once, for the highest rate: configure() may run
// while the host's audio thread reads.
AudioSynth::AudioSynth() {
    m_line.assign(kLineSize, 0.0f);
    for (int i = 0; i < kCombs; i++) m_comb[i].assign(kCombLen[i] * 192000 / 44100 + 1, 0.0f);
    for (int i = 0; i < kAllpasses; i++) m_allpass[i].assign(kAllpassLen[i] * 192000 / 44100 + 1, 0.0f);
    configure(m_rate, m_channels);
}

void AudioSynth::configure(uint32_t sampleRate, uint32_t channels) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rate = std::clamp<uint32_t>(sampleRate, 8000, 192000);
    m_channels = std::clamp<uint32_t>(channels, 1, 8);
    for (int i = 0; i < kCombs; i++) {
        m_combLen[i] = std::max<uint32_t>(1, kCombLen[i] * m_rate / 44100);
        m_combPos[i] = 0;
    }
    for (int i = 0; i < kAllpasses; i++) {
        m_allpassLen[i] = std::max<uint32_t>(1, kAllpassLen[i] * m_rate / 44100);
        m_allpassPos[i] = 0;
    }
    m_delay = -1;
}

void AudioSynth::setParams(const AudioParams &p) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_target = p;
}

void AudioSynth::setVolume(float motors, float beeps) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_volume = std::clamp(motors, 0.0f, 2.0f);
    m_beepVolume = std::clamp(beeps, 0.0f, 2.0f);
}

// Linear interpolation between the samples `delaySamples` ago.
float AudioSynth::delayTap(double delaySamples) const {
    double pos = (double)m_write - delaySamples;
    double fl = std::floor(pos);
    float frac = (float)(pos - fl);
    uint32_t i = (uint32_t)(int64_t)fl & (kLineSize - 1);
    return m_line[i] + (m_line[(i + 1) & (kLineSize - 1)] - m_line[i]) * frac;
}

uint32_t AudioSynth::read(float *out, uint32_t frames) {
    AudioParams target;
    uint32_t rate, channels;
    float volume, beepVolume;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        target = m_target;
        rate = m_rate;
        channels = m_channels;
        volume = m_volume;
        beepVolume = m_beepVolume;
    }
    const double dt = 1.0 / rate;
    // parameters glide over ~15 ms so host frame updates do not click, and
    // the hearing over ~60 ms (it changes a few times a second)
    const float glide = 1.0f - std::exp(-(float)frames / (rate * 0.015f));
    const float slowGlide = 1.0f - std::exp(-(float)frames / (rate * 0.06f));
    AudioParams from = m_cur;
    AudioParams &cur = m_cur;
    for (int m = 0; m < 4; m++) {
        cur.rpm[m] += (target.rpm[m] - cur.rpm[m]) * glide;
        cur.load[m] += (target.load[m] - cur.load[m]) * glide;
        cur.damage[m] = target.damage[m];
        cur.pan[m] = target.pan[m];
    }
    cur.maxRpm = target.maxRpm;
    cur.blades = target.blades;
    cur.beeper = target.beeper;
    cur.motorBeep = target.motorBeep;
    const Hearing &th = target.hearing;
    Hearing &h = cur.hearing;
    if (m_delay < 0) h = th;  // the first block starts where the drone is
    auto toward = [&](float &v, float t) { v += (t - v) * slowGlide; };
    toward(h.gain, th.gain);
    toward(h.airCutoff, th.airCutoff);
    toward(h.wallCutoff, th.wallCutoff);
    toward(h.pan, th.pan);
    toward(h.groundGain, th.groundGain);
    toward(h.echoGain, th.echoGain);
    toward(h.reverb, th.reverb);
    toward(h.reverbTime, th.reverbTime);
    toward(h.onboardReverb, th.onboardReverb);
    h.fromPilot = th.fromPilot;
    const Hearing &hf = from.hearing;
    float pausedGain = target.paused ? 0.0f : 1.0f;

    // the travel time, in samples. The host sets it once a frame; the read
    // point follows it smoothly (three one-pole stages of 40 ms, per sample),
    // so its slope is the Doppler shift without the frame rate's staircase in
    // it (25 fps leaves a ripple of 0.05 %), at most half a sample per sample
    // (a drone at half the speed of sound). A jump of 17 m or more between
    // two updates is a teleport (or a new ear), taken at once.
    double maxDelay = std::min<double>(kMaxDelay * rate, kLineSize - 4.0);
    double delayTarget = std::clamp((double)th.delay * rate, 1.0, maxDelay);
    double groundTarget = std::max(0.0, (double)th.groundDelay * rate);
    double echoTarget = std::max(0.0, (double)th.echoDelay * rate);
    if (m_delay < 0 || std::fabs(delayTarget - m_delay) > 0.05 * rate) {
        m_delay = m_delayStage[0] = m_delayStage[1] = delayTarget;
        m_ground = Smoothed{{groundTarget, groundTarget}, groundTarget};
        m_echo = Smoothed{{echoTarget, echoTarget}, echoTarget};
    }
    const double follow = 1.0 - std::exp(-1.0 / (rate * 0.04));
    auto smooth = [&](double &value, double stage[2], double target) {
        stage[0] += (target - stage[0]) * follow;
        stage[1] += (stage[0] - stage[1]) * follow;
        value += std::clamp((stage[1] - value) * follow, -0.5, 0.5);
    };
    const float aGround = onePole(1500.0f, rate);

    const float aAir = onePole(h.airCutoff, rate), aWall = onePole(h.wallCutoff, rate);
    const float beepRamp = 1.0f - std::exp(-(float)dt / 0.002f);
    const float mixRamp = 1.0f - std::exp(-(float)dt / 0.15f);
    float combFb[kCombs];
    for (int i = 0; i < kCombs; i++)
        combFb[i] = std::pow(10.0f, -3.0f * m_combLen[i] / (rate * std::max(0.1f, h.reverbTime)));
    constexpr float kCombDamp = 0.3f, kAllpassFb = 0.5f;

    for (uint32_t f = 0; f < frames; f++) {
        float t = frames > 1 ? (float)f / (frames - 1) : 1.0f;
        float left = 0, right = 0;
        for (int m = 0; m < 4; m++) {
            float rpm = from.rpm[m] + (cur.rpm[m] - from.rpm[m]) * t;
            float load = from.load[m] + (cur.load[m] - from.load[m]) * t;
            float spin = std::clamp(std::fabs(rpm) / std::max(1000.0f, cur.maxRpm), 0.0f, 1.2f);
            if (spin < 0.002f) continue;
            double rev = std::fabs(rpm) / 60.0;
            double bpf = rev * cur.blades;  // blade-pass frequency
            m_phase[m] += bpf * dt;
            m_phase[m] -= std::floor(m_phase[m]);
            m_whine[m] += rev * kPolePairs * dt;
            m_whine[m] -= std::floor(m_whine[m]);
            m_wobble[m] += rev * dt;
            m_wobble[m] -= std::floor(m_wobble[m]);

            // blade-pass buzz: band-limited saw, harmonics above Nyquist dropped
            float buzz = 0;
            for (int k = 1; k <= kHarmonics; k++) {
                if (bpf * k >= rate * 0.45) break;
                buzz += (float)std::sin(kTwoPi * m_phase[m] * k) / (float)k;
            }
            float whine = (rev * kPolePairs < rate * 0.45) ? (float)std::sin(kTwoPi * m_whine[m]) : 0.0f;
            // air noise, low-passed with a cutoff that follows the rpm
            m_seed = m_seed * 1664525u + 1013904223u;
            float white = (float)(int32_t)m_seed / 2147483648.0f;
            float a = (float)std::min(0.9, kTwoPi * bpf * 2.0 * dt);
            m_noiseLp[m] += (white - m_noiseLp[m]) * a;

            float dmg = cur.damage[m];
            float wobble = 1.0f + dmg * 0.6f * (float)std::sin(kTwoPi * m_wobble[m]);
            float amp = spin * spin * (0.35f + 0.65f * load);
            float s = (buzz * 0.55f + whine * 0.08f * (0.3f + load) + m_noiseLp[m] * (0.6f + dmg * 1.5f)) * amp * wobble;
            float pan = std::clamp(cur.pan[m], -1.0f, 1.0f);
            left += s * (0.5f - 0.35f * pan);
            right += s * (0.5f + 0.35f * pan);
        }
        // the beeper switch: the ESCs beep through the windings, so it is
        // part of the motor sound (and its volume)
        m_motorBeepEnv += ((cur.motorBeep ? 1.0f : 0.0f) - m_motorBeepEnv) * beepRamp;
        float beacon = 0;
        if (m_motorBeepEnv > 1e-4f) {
            m_motorBeepPhase += kBeaconHz * dt;
            m_motorBeepPhase -= std::floor(m_motorBeepPhase);
            float ph = (float)(kTwoPi * m_motorBeepPhase);
            beacon = (std::sin(ph) + 0.3f * std::sin(3 * ph)) * 0.14f * m_motorBeepEnv;
        }
        // every other beep: the piezo buzzer
        bool buzzer = cur.beeper && !cur.motorBeep;
        m_beepEnv += ((buzzer ? 1.0f : 0.0f) - m_beepEnv) * beepRamp;
        float buzz = 0;
        if (m_beepEnv > 1e-4f) {
            m_beepPhase += kBuzzerHz * dt;
            m_beepPhase -= std::floor(m_beepPhase);
            buzz = (m_beepPhase < 0.5 ? 0.12f : -0.12f) * m_beepEnv;
        }
        float onL = (left + beacon) * volume + buzz * beepVolume;
        float onR = (right + beacon) * volume + buzz * beepVolume;
        float mono = 0.5f * (onL + onR);

        // --- from the pilot ---------------------------------------------------
        m_line[m_write & (kLineSize - 1)] = mono;
        m_write++;
        smooth(m_delay, m_delayStage, delayTarget);
        m_delay = std::clamp(m_delay, 1.0, maxDelay);
        smooth(m_ground.value, m_ground.stage, groundTarget);
        smooth(m_echo.value, m_echo.stage, echoTarget);
        float x = delayTap(m_delay);
        // the ground's reflection, low-passed: grass, dirt and asphalt are
        // rough at the highs' wavelengths, so only the lows add up coherently
        // (an unfiltered copy comb-filtered the whole sound into a flanger)
        float gg = hf.groundGain + (h.groundGain - hf.groundGain) * t;
        m_groundLp += (delayTap(std::min(maxDelay, m_delay + m_ground.value)) - m_groundLp) * aGround;
        if (gg > 1e-4f) x += gg * m_groundLp;
        float eg = hf.echoGain + (h.echoGain - hf.echoGain) * t;
        if (eg > 1e-4f) x += eg * delayTap(std::min(maxDelay, m_delay + m_echo.value));
        m_air += (x - m_air) * aAir;
        m_wall1 += (m_air - m_wall1) * aWall;
        m_wall2 += (m_wall1 - m_wall2) * aWall;
        float gain = hf.gain + (h.gain - hf.gain) * t;
        float direct = m_wall2 * gain;
        float pan = std::clamp(hf.pan + (h.pan - hf.pan) * t, -1.0f, 1.0f);
        float ang = (pan + 1) * 0.25f * (float)kTwoPi * 0.5f;  // 0 .. pi/2, equal power
        float pilotL = direct * std::cos(ang) * 1.41421356f;
        float pilotR = direct * std::sin(ang) * 1.41421356f;

        // the room, fed by what the ear would hear (or by the drone onboard)
        m_mix += (h.fromPilot - m_mix) * mixRamp;
        float send = m_mix * (hf.reverb + (h.reverb - hf.reverb) * t) +
                     (1 - m_mix) * (hf.onboardReverb + (h.onboardReverb - hf.onboardReverb) * t);
        float wetL = 0, wetR = 0;
        if (send > 1e-4f || std::fabs(m_combLp[0]) > 1e-6f) {
            float in = (m_mix * m_wall2 + (1 - m_mix) * mono) * send * 0.25f;
            float acc = 0;
            for (int i = 0; i < kCombs; i++) {
                float &cell = m_comb[i][m_combPos[i]];
                float y = cell;
                m_combLp[i] = y * (1 - kCombDamp) + m_combLp[i] * kCombDamp;
                cell = in + m_combLp[i] * combFb[i];
                m_combPos[i] = (m_combPos[i] + 1) % m_combLen[i];
                acc += (i & 1) ? -y : y;  // alternating signs: wider, less coloured
            }
            float y = acc;
            for (int i = 0; i < kAllpasses; i++) {
                float &cell = m_allpass[i][m_allpassPos[i]];
                float b = cell;
                cell = y + b * kAllpassFb;
                float o = b - y;
                m_allpassPos[i] = (m_allpassPos[i] + 1) % m_allpassLen[i];
                if (i == 0) wetL = o;  // left after the first, right after both
                y = o;
            }
            wetR = y;
        }

        left = (onL * (1 - m_mix) + pilotL * m_mix + wetL) * pausedGain;
        right = (onR * (1 - m_mix) + pilotR * m_mix + wetR) * pausedGain;
        left = std::tanh(left);
        right = std::tanh(right);
        float *o = out + (size_t)f * channels;
        if (channels == 1) {
            o[0] = 0.5f * (left + right);
        } else {
            o[0] = left;
            o[1] = right;
            for (uint32_t c = 2; c < channels; c++) o[c] = 0;
        }
    }
    return frames;
}

}  // namespace fpv
