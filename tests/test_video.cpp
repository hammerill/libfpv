// libfpv - the video link (D34) without the library: the link budget's
// pieces, the fading's statistics, the per-line table the shader reads and
// the field clock.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <cstdio>
#include <cstring>

#include "config/preset.h"
#include "video/video_link.h"

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

static fpv_video_feed g_feed;  // big: keep it off the stack

// decoders of the table, as in docs/INTEGRATION.md#the-video-feed
static float grainOf(const fpv_video_feed &f, int l) { return f.lines[l * 8 + 0] / 255.0f * 0.5f; }
static float densityOf(const fpv_video_feed &f, int l) { float v = f.lines[l * 8 + 1] / 255.0f; return v * v * v; }
static float chromaOf(const fpv_video_feed &f, int l) { return f.lines[l * 8 + 2] / 255.0f; }
static float pictureOf(const fpv_video_feed &f, int l) { return f.lines[l * 8 + 3] / 255.0f; }
static float shiftOf(const fpv_video_feed &f, int l) {
    return (f.lines[l * 8 + 4] * 256 + f.lines[l * 8 + 5]) / 65535.0f * 0.5f - 0.25f;
}

static VideoSpec analogSpec(int system = FPV_VIDEO_NTSC) {
    DronePreset p;
    p.vtx.digital = false;
    p.vtx.powerMw = 25;
    p.camera.tvl = 1200;
    p.camera.system = system;
    p.camera.aspect = 4.0f / 3.0f;
    return videoSpec(p);
}

static VideoInputs link(float rxDbm, float k = 10, float doppler = 100) {
    VideoInputs in;
    in.link.rxDbm = rxDbm;
    in.link.kFactor = k;
    in.link.dopplerHz = doppler;
    in.noise = in.resolution = true;
    return in;
}

static void testBudget() {
    std::printf("link budget\n");
    float fspl = freeSpaceLossDb(1000);
    CHECK(std::fabs(fspl - 107.7f) < 0.2f, "free space at 1 km, 5.8 GHz: %.1f dB, expected 107.7", fspl);
    CHECK(std::fabs(freeSpaceLossDb(2000) - fspl - 6.02f) < 0.05f, "free space must grow 6 dB per doubling");
    CHECK(groundLossDb(100, 20, 1.7f) == 0, "high and close: no ground loss");
    float low = groundLossDb(1000, 1, 1.7f);
    CHECK(low > 5 && low < 25, "1 m over the ground at 1 km: %.1f dB, expected a few dB to 20", low);
    float side = txAntennaGainDb({0, 0, -1}), along = txAntennaGainDb({0, 0.866f, -0.5f});
    CHECK(side > 0 && along < -10, "VTX dipole: %.1f dBi broadside, %.1f dBi along its axis", side, along);
    CHECK(rxAntennaGainDb(0) > 7 && rxAntennaGainDb(1.5f) < 0, "goggles: the patch at the horizon, weak overhead");
    // Rice's clicks: none above ~13 dB, the first ones at ~10 dB, snow at 0
    float c15 = fmClicksPerLine(15), c10 = fmClicksPerLine(10), c0 = fmClicksPerLine(0);
    CHECK(c15 * 480 < 0.05f, "15 dB CNR must be clean: %.4f clicks per frame", c15 * 480);
    CHECK(c10 * 480 > 0.2f && c10 * 480 < 20, "10 dB CNR: first sparkles, %.2f per frame", c10 * 480);
    CHECK(c0 > 10, "0 dB CNR: %.1f clicks per line, expected a line full of them", c0);
    // a 25 mW VTX at 300 m in the open stays above the threshold; at 3 km it is gone
    float rx300 = 14 + kVideoTxGainDbi + kVideoRxPatchDbi - kVideoSystemLossDb - freeSpaceLossDb(300);
    float rx3k = 14 + kVideoTxGainDbi + kVideoRxPatchDbi - kVideoSystemLossDb - freeSpaceLossDb(3000);
    CHECK(rx300 - kVideoNoiseFloorDbm > 15, "25 mW at 300 m: %.1f dB CNR", rx300 - kVideoNoiseFloorDbm);
    CHECK(rx3k - kVideoNoiseFloorDbm < 8, "25 mW at 3 km: %.1f dB CNR", rx3k - kVideoNoiseFloorDbm);
}

static void testSpec() {
    std::printf("camera and feed spec\n");
    VideoSpec ntsc = analogSpec(), pal = analogSpec(FPV_VIDEO_PAL);
    CHECK(ntsc.rasterW == 720 && ntsc.rasterH == 480 && pal.rasterH == 576, "analog rasters 720x480 / 720x576");
    CHECK(ntsc.lumaCycles > 200 && ntsc.lumaCycles < 300,
          "1200TVL through a 5 MHz link: %.0f cycles across, expected 200-300 (~400 TVL)", ntsc.lumaCycles);
    DronePreset cheap;
    cheap.camera.tvl = 600;
    CHECK(videoSpec(cheap).lumaCycles < ntsc.lumaCycles, "a 600TVL camera must be softer");
    CHECK(std::fabs(ntsc.subcarrierCycles - 188.5f) < 1, "NTSC subcarrier: %.1f cycles per line", ntsc.subcarrierCycles);
    DronePreset hd;
    hd.vtx.digital = true;
    hd.vtx.height = 540;
    hd.camera.aspect = 4.0f / 3.0f;
    VideoSpec d = videoSpec(hd);
    CHECK(d.rasterW == 720 && d.rasterH == 540, "digital 540p in 4:3: %dx%d", d.rasterW, d.rasterH);
}

static void testFieldClock() {
    std::printf("field clock\n");
    VideoFeed feed;
    VideoSpec s = analogSpec();
    VideoInputs in = link(-80);
    uint32_t first = 0;
    for (int i = 0; i < 144; i++) {  // a 144 Hz game for one second
        feed.update(1.0f / 144, s, in, g_feed);
        if (i == 0) first = g_feed.field;
    }
    uint32_t fields = g_feed.field - first;
    CHECK(fields >= 58 && fields <= 61, "NTSC: %u fields in a second, expected 59-60", fields);
    // within a field the feed repeats itself
    uint8_t before[FPV_VIDEO_MAX_LINES * 8];
    std::memcpy(before, g_feed.lines, sizeof before);
    uint32_t field = g_feed.field;
    feed.update(0.0001f, s, in, g_feed);
    CHECK(g_feed.field == field && std::memcmp(before, g_feed.lines, sizeof before) == 0, "same field, same lines");
    CHECK(g_feed.line_count == 263, "NTSC field: %u lines with the blanking", g_feed.line_count);
}

static void testStrongAndWeak() {
    std::printf("strong and weak links\n");
    VideoSpec s = analogSpec();
    {
        VideoFeed feed;
        VideoInputs in = link(-60);  // 44 dB CNR
        float worstGrain = 0, worstShift = 0, sparkles = 0;
        for (int i = 0; i < 120; i++) {
            feed.update(1.0f / 60, s, in, g_feed);
            for (int l = 0; l < 240; l++) {
                worstGrain = std::fmax(worstGrain, grainOf(g_feed, l));
                worstShift = std::fmax(worstShift, std::fabs(shiftOf(g_feed, l)));
                sparkles += densityOf(g_feed, l);
                CHECK(pictureOf(g_feed, l) > 0.99f && chromaOf(g_feed, l) > 0.99f, "strong: full picture and colour");
            }
            CHECK(g_feed.roll == 0, "strong: no roll");
        }
        CHECK(worstGrain < 0.02f, "strong: grain %.3f", worstGrain);
        CHECK(worstShift < 0.003f, "strong: lines shifted by %.4f", worstShift);
        CHECK(sparkles * 360 < 1, "strong: %.2f sparkles in two seconds", sparkles * 360);
    }
    {
        VideoFeed feed;
        VideoInputs in = link(-112, 0.3f);  // -8 dB CNR, behind a building
        bool rolled = false;
        float picture = 0, chroma = 0, shift = 0;
        int n = 0;
        for (int i = 0; i < 180; i++) {
            feed.update(1.0f / 60, s, in, g_feed);
            rolled |= g_feed.roll != 0;
            for (int l = 0; l < 240; l++, n++) {
                picture += pictureOf(g_feed, l);
                chroma += chromaOf(g_feed, l);
                shift += std::fabs(shiftOf(g_feed, l));
            }
        }
        CHECK(picture / n < 0.35f, "weak: picture %.2f, expected mostly snow", picture / n);
        CHECK(chroma / n < 0.05f, "weak: colour %.2f, expected the colour killer", chroma / n);
        CHECK(shift / n > 0.004f, "weak: lines torn by %.4f on average", shift / n);
        CHECK(rolled, "weak: vertical sync never lost in three seconds");
    }
    {
        VideoFeed feed;
        VideoInputs in = link(-60);
        in.noise = false;
        feed.update(0.02f, s, in, g_feed);
        CHECK(grainOf(g_feed, 10) == 0 && pictureOf(g_feed, 10) == 1 && std::fabs(shiftOf(g_feed, 10)) < 1e-4f,
              "noise off: neutral lines");
        CHECK((g_feed.flags & FPV_VIDEO_FEED_NOISE) == 0 && (g_feed.flags & FPV_VIDEO_FEED_RESOLUTION),
              "flags follow the player's settings");
        CHECK(g_feed.flags & FPV_VIDEO_FEED_OSD_IN_SIGNAL, "analog: the OSD is in the signal");
    }
}

// Rayleigh (K = 0) fades deep and often, a strong direct path (K = 10)
// hardly; faster flight brings them by faster.
static void testFading() {
    std::printf("fading\n");
    VideoSpec s = analogSpec();
    auto stats = [&](float k, float doppler, float &below10, float &meanLin, int &crossings) {
        VideoFeed feed;
        VideoInputs in = link(-60, k, doppler);
        int n = 0, under = 0;
        double sum = 0;
        bool wasUnder = false;
        crossings = 0;
        for (int i = 0; i < 600; i++) {  // ten seconds
            feed.update(1.0f / 60, s, in, g_feed);
            for (int j = 0; j < 8; j++, n++) {
                float db = feed.branchFadeDb(0, j * 0.002, k);
                sum += std::pow(10.0, db / 10.0);
                bool u = db < -10;
                under += u;
                if (u && !wasUnder) crossings++;
                wasUnder = u;
            }
        }
        below10 = (float)under / n;
        meanLin = (float)(sum / n);
    };
    float r10, rMean, l10, lMean;
    int rCross, lCross, slowCross;
    stats(0, 200, r10, rMean, rCross);
    stats(10, 200, l10, lMean, lCross);
    float s10, sMean;
    stats(0, 20, s10, sMean, slowCross);
    CHECK(rMean > 0.7f && rMean < 1.3f, "mean fade power %.2f, expected 1", rMean);
    CHECK(r10 > 0.04f && r10 < 0.18f, "Rayleigh: %.3f of the time 10 dB down, expected ~0.095", r10);
    CHECK(l10 < r10 * 0.25f, "K = 10: %.3f of the time 10 dB down, expected far less than Rayleigh's %.3f", l10, r10);
    CHECK(rCross > slowCross * 2, "fades at 200 Hz Doppler (%d) must come faster than at 20 Hz (%d)", rCross, slowCross);
}

static void testDigital() {
    std::printf("digital\n");
    DronePreset p;
    p.vtx.digital = true;
    p.vtx.freezes = true;
    p.vtx.sensitivityDbm = -95;
    VideoSpec s = videoSpec(p);
    VideoFeed good, bad;
    float goodErr = 0;
    int frozen = 0;
    for (int i = 0; i < 120; i++) {
        good.update(1.0f / 60, s, link(-70), g_feed);
        goodErr = std::fmax(goodErr, g_feed.block_errors);
        bad.update(1.0f / 60, s, link(-103), g_feed);
        frozen += g_feed.frozen != 0;
    }
    CHECK(goodErr < 0.01f, "25 dB margin: %.3f of the blocks lost", goodErr);
    CHECK(frozen > 30, "8 dB under: frozen %d of 120 frames", frozen);
}

int main() {
    testBudget();
    testSpec();
    testFieldClock();
    testStrongAndWeak();
    testFading();
    testDigital();
    if (g_failures) {
        std::printf("%d FAILED\n", g_failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
