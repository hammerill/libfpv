// libfpv - the drone sound heard from the pilot: a moving drone's Doppler
// shift must be steady although the host updates the hearing only once a
// frame (a staircase of travel times), the bug heard as a wobbly drone.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <cstdio>
#include <vector>

#include "audio/synth.h"

using namespace fpv;

static int g_failures = 0;
#define CHECK(cond, ...)                                       \
    do {                                                       \
        if (!(cond)) {                                         \
            std::printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                          \
            std::printf("\n");                                 \
            g_failures++;                                      \
        }                                                      \
    } while (0)

// A drone receding at `speed` m/s from 30 m, the hearing set at `fps`; only
// the beacon tone (1760 Hz) plays. Returns the pitch of every 8 cycles.
static std::vector<double> pitches(float speed, float fps, double &expected) {
    const uint32_t rate = 44100, block = 512;
    const float c = 343.0f;
    AudioSynth synth;
    synth.configure(rate, 1);
    synth.setVolume(1.0f, 1.0f);
    AudioParams p;
    p.beeper = p.motorBeep = true;  // the beeper switch: the ESCs' 1760 Hz tone
    p.hearing.fromPilot = 1;
    p.hearing.gain = 1;
    std::vector<float> out;
    std::vector<float> buf(block);
    double t = 0, nextUpdate = 0;
    while (t < 3.0) {
        if (t >= nextUpdate) {  // the host's frame
            float r = 30.0f + speed * (float)nextUpdate;
            p.hearing.delay = r / (c + speed);
            synth.setParams(p);
            nextUpdate += 1.0 / fps;
        }
        synth.read(buf.data(), block);
        out.insert(out.end(), buf.begin(), buf.end());
        t += (double)block / rate;
    }
    expected = 1760.0 * c / (c + speed);
    std::vector<double> crossings;  // upward zero crossings, interpolated
    for (size_t i = (size_t)(0.5 * rate); i + 1 < out.size(); i++)
        if (out[i] < 0 && out[i + 1] >= 0) crossings.push_back(i + out[i] / (out[i] - out[i + 1]));
    std::vector<double> f;
    for (size_t i = 8; i < crossings.size(); i += 8) f.push_back(8.0 * rate / (crossings[i] - crossings[i - 8]));
    return f;
}

int main() {
    for (float fps : {25.0f, 30.0f, 60.0f}) {
        double expected;
        std::vector<double> f = pitches(20.0f, fps, expected);
        double mean = 0, worst = 0;
        for (double v : f) mean += v;
        mean /= f.empty() ? 1 : f.size();
        for (double v : f) worst = std::fmax(worst, std::fabs(v - mean));
        std::printf("doppler at %2.0f fps: %.1f Hz (theory %.1f), worst deviation %.2f Hz (%.3f %%)\n", fps, mean,
                    expected, worst, 100 * worst / mean);
        CHECK(!f.empty() && std::fabs(mean - expected) < 2.0, "the shift is the theory's (%.1f vs %.1f Hz)", mean,
              expected);
        // a wobble of 0.3 % is 5 cents; the staircase gave several percent
        CHECK(worst < mean * 0.003, "steady pitch at %.0f fps (worst %.2f Hz off)", fps, worst);
    }
    std::printf(g_failures ? "%d failures\n" : "all synth checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}
