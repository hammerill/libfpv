// libfpv - the drone sound heard from the pilot: a moving drone's Doppler
// shift must be steady although the host updates the hearing only once a
// frame (a staircase of travel times), the bug heard as a wobbly drone.
// And the two ears: a drone to one side is louder and earlier on that side,
// behind the head it is duller, and moving it around never clicks.
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

// A drone heard from the pilot, 3.4 m away; `beacon` plays the 1760 Hz
// tone, otherwise the motors (broadband, for the ears' delay). Returns
// stereo frames from 0.5 s on (after the cross-fade from onboard).
static std::vector<float> ears(float pan, float front, bool beacon, float seconds = 1.5f, float panAfter = 2,
                               float frontAfter = 2, float reverb = 0) {
    const uint32_t rate = 44100, block = 512;
    AudioSynth synth;
    synth.configure(rate, 2);
    synth.setVolume(1.0f, 1.0f);
    AudioParams p;
    if (beacon) {
        p.beeper = p.motorBeep = true;
    } else {
        for (int m = 0; m < 4; m++) p.rpm[m] = 18000 + 700 * m, p.load[m] = 0.5f;
    }
    p.hearing.fromPilot = 1;
    p.hearing.gain = 0.5f;
    p.hearing.delay = 0.01f;
    p.hearing.pan = pan;
    p.hearing.front = front;
    p.hearing.reverb = reverb;
    synth.setParams(p);
    std::vector<float> out, buf(block * 2);
    for (double t = 0; t < seconds; t += (double)block / rate) {
        if (panAfter <= 1 && t >= 1.0) {  // the drone jumps around the head at 1 s
            p.hearing.pan = panAfter;
            p.hearing.front = frontAfter;
            synth.setParams(p);
        }
        synth.read(buf.data(), block);
        if (t >= 0.5) out.insert(out.end(), buf.begin(), buf.end());
    }
    return out;
}

static double rms(const std::vector<float> &st, int ch) {
    double sum = 0;
    for (size_t i = ch; i < st.size(); i += 2) sum += (double)st[i] * st[i];
    return std::sqrt(sum / std::fmax(1.0, st.size() / 2.0));
}

static double db(double a, double b) { return 20 * std::log10(a / b); }

// How many samples the left channel lags the right one (cross-correlation peak).
static int lag(const std::vector<float> &st) {
    size_t n = st.size() / 2;
    int best = 0;
    double bestC = -1e30;
    for (int k = -60; k <= 60; k++) {
        double c = 0;
        for (size_t i = 60; i + 60 < n; i++) c += (double)st[2 * (i + k)] * st[2 * i + 1];
        if (c > bestC) bestC = c, best = k;
    }
    return best;
}

// The largest step between two samples of either channel.
static double maxStep(const std::vector<float> &st, size_t from, size_t to) {
    double m = 0;
    for (size_t i = std::max<size_t>(from, 1); i < std::min(to, st.size() / 2); i++)
        for (int c = 0; c < 2; c++) m = std::fmax(m, std::fabs(st[2 * i + c] - st[2 * (i - 1) + c]));
    return m;
}

static void testEars() {
    std::vector<float> centre = ears(0, 1, true), right = ears(1, 0, true), left = ears(-1, 0, true);
    double centreDiff = db(rms(centre, 1), rms(centre, 0));
    double rightDiff = db(rms(right, 1), rms(right, 0)), leftDiff = db(rms(left, 1), rms(left, 0));
    std::printf("ears: centre R-L %.2f dB, right %.1f dB, left %.1f dB\n", centreDiff, rightDiff, leftDiff);
    CHECK(std::fabs(centreDiff) < 0.1, "ahead is in the middle (%.2f dB)", centreDiff);
    CHECK(rightDiff > 8, "to the right is louder on the right (%.1f dB)", rightDiff);
    CHECK(std::fabs(rightDiff + leftDiff) < 0.1, "left mirrors right (%.1f vs %.1f dB)", leftDiff, rightDiff);
    // equal power: the drone is as loud to one side as ahead
    double power0 = rms(centre, 0) * rms(centre, 0) + rms(centre, 1) * rms(centre, 1);
    double power1 = rms(right, 0) * rms(right, 0) + rms(right, 1) * rms(right, 1);
    CHECK(std::fabs(10 * std::log10(power1 / power0)) < 2.0, "about as loud to the side (%.1f dB)",
          10 * std::log10(power1 / power0));

    // the room is in the middle too (it leaned 3-4 dB to the right)
    std::vector<float> room = ears(0, 1, false, 1.5f, 2, 2, 1.0f);
    double roomDiff = db(rms(room, 1), rms(room, 0));
    std::printf("ears: with the room R-L %.2f dB\n", roomDiff);
    CHECK(std::fabs(roomDiff) < 1.0, "the room is centred (%.2f dB)", roomDiff);

    // the interaural time difference: 0.66 ms (29 samples) to one side
    int lagCentre = lag(ears(0, 1, false)), lagRight = lag(ears(1, 0, false)), lagLeft = lag(ears(-1, 0, false));
    std::printf("ears: left lags right by %d (ahead), %d (right), %d (left) samples\n", lagCentre, lagRight, lagLeft);
    CHECK(lagCentre == 0, "no delay ahead (%d)", lagCentre);
    CHECK(lagRight >= 25 && lagRight <= 35, "the left ear later for a drone to the right (%d)", lagRight);
    CHECK(lagLeft <= -25 && lagLeft >= -35, "the right ear later for a drone to the left (%d)", lagLeft);

    // behind the head: quieter and duller than ahead
    std::vector<float> behind = ears(0, -1, false), ahead = ears(0, 1, false);
    double behindDb = db(rms(behind, 0), rms(ahead, 0));
    std::printf("ears: behind %.1f dB\n", behindDb);
    CHECK(behindDb < -1.5 && behindDb > -8, "behind is a little quieter (%.1f dB)", behindDb);

    // the drone jumping from the right to behind-left: no click, the
    // largest step stays that of the tone (2 pi 1760 / 44100 of its peak)
    std::vector<float> jump = ears(1, 0, true, 2.0f, -1, -0.5f);
    size_t at = (size_t)(0.5 * 44100);  // 1 s, from 0.5 s on
    double steady = maxStep(jump, 0, at - 512), moving = maxStep(jump, at - 512, at + 44100 / 2);
    std::printf("ears: largest step %.4f steady, %.4f while moving\n", steady, moving);
    CHECK(moving < steady * 1.15, "moving around the head doesn't click (%.4f vs %.4f)", moving, steady);
    // and it got there
    std::vector<float> tail(jump.end() - 44100 / 4 * 2, jump.end());
    CHECK(db(rms(tail, 1), rms(tail, 0)) < -8, "ends up on the left (%.1f dB)", db(rms(tail, 1), rms(tail, 0)));
}

int main() {
    testEars();
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
