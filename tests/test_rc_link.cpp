// libfpv - the RC link (D36) without the library: the budget's pieces,
// packets and link quality, losing and finding the receiver's sync, and
// ELRS's dynamic power.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <cstdio>

#include "rc/rc_link.h"
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

// The path the way Flight builds it: in the open, the drone side-on.
static float openPath(RcBand band, float dist, float height = 30) {
    float lambda = rcWavelength(band);
    return kRcTxGainDbi + rcRxAntennaGainDb(band, {1, 0, 0}) - freeSpaceLossDb(dist, lambda) -
           groundLossDb(dist, height, kRcTxHeight, lambda) - kRcSystemLossDb;
}

static RcLinkConfig config(int mode, int maxMw, bool dynamic) {
    RcLinkConfig c;
    c.enabled = true;
    c.mode = mode;
    c.maxPowerMw = maxMw;
    c.dynamicPower = dynamic;
    return c;
}

// Runs the link at 1 kHz for `seconds`; returns the packets received.
static int run(RcLink &link, float seconds, RcLinkInputs in) {
    int got = 0;
    int steps = (int)std::lround(seconds * 1000);
    for (int i = 0; i < steps; i++) got += link.step(0.001f, in);
    return got;
}

static void testModesAndPower() {
    std::printf("modes and power levels\n");
    CHECK(rcLinkMode(FPV_RC_LINK_2G4_500HZ).rateHz == 500, "500 Hz mode");
    CHECK(rcLinkMode(FPV_RC_LINK_2G4_500HZ).sensitivityDbm == -105, "500 Hz: -105 dBm");
    CHECK(rcLinkMode(FPV_RC_LINK_2G4_500HZ).rfMode == 29, "500 Hz shows RFMD 29");
    CHECK(rcLinkMode(FPV_RC_LINK_900_50HZ).sensitivityDbm == -120, "900 MHz 50 Hz: -120 dBm");
    CHECK(rcLinkMode(FPV_RC_LINK_900_25HZ).band == RcBand::Mhz900, "25 Hz is 900 MHz");
    CHECK(rcLinkMode(FPV_RC_LINK_2G4_F1000).flrc, "F1000 is FLRC");
    for (int m = 1; m < FPV_RC_LINK_MODE_COUNT; m++) {
        const RcLinkMode &a = rcLinkMode(m);
        CHECK(a.snrUp < a.snrDown, "%s: dynamic power must lower above where it raises", a.name);
    }
    CHECK(kRcPowerLevels[rcPowerLevel(300)] == 250, "300 mW rounds down to 250");
    CHECK(kRcPowerLevels[rcPowerLevel(1)] == 10, "below 10 mW: 10");
    CHECK(kRcPowerLevels[rcPowerLevel(5000)] == 1000, "above 1 W: 1 W");
}

static void testBudget() {
    std::printf("link budget\n");
    float f24 = freeSpaceLossDb(1000, kRcWavelength24);
    float f900 = freeSpaceLossDb(1000, kRcWavelength900);
    CHECK(std::fabs(f24 - 100.2f) < 0.3f, "free space at 1 km, 2.44 GHz: %.1f dB, expected 100.2", f24);
    CHECK(std::fabs(f24 - f900 - 8.5f) < 0.2f, "900 MHz loses 8.5 dB less: %.1f", f24 - f900);
    CHECK(RcLink::packetSuccess(0) > 0.75f && RcLink::packetSuccess(0) < 0.9f, "at the sensitivity most packets still arrive");
    CHECK(RcLink::packetSuccess(3) > 0.99f, "3 dB above: practically all");
    CHECK(RcLink::packetSuccess(-4) < 0.01f, "4 dB below: practically none");

    // Range in the open (drone 30 m up): where the mean margin reaches 0
    auto range = [](int mode, float mw) {
        const RcLinkMode &m = rcLinkMode(mode);
        float d = 10;
        while (d < 200000 && 10 * std::log10(mw) + openPath(m.band, d) > m.sensitivityDbm) d *= 1.02f;
        return d;
    };
    float freestyle = range(FPV_RC_LINK_2G4_500HZ, 100);
    float lr24 = range(FPV_RC_LINK_2G4_50HZ, 100);
    float lr900 = range(FPV_RC_LINK_900_50HZ, 100);
    std::printf("  100 mW range: 2.4 GHz 500 Hz %.1f km, 2.4 GHz 50 Hz %.1f km, 900 MHz 50 Hz %.1f km\n",
                freestyle / 1000, lr24 / 1000, lr900 / 1000);
    CHECK(freestyle > 1500 && freestyle < 6000, "freestyle 100 mW: some km, %.0f m", freestyle);
    CHECK(lr24 > freestyle * 1.5f, "50 Hz reaches farther than 500 Hz");
    CHECK(lr900 > lr24 && lr900 > 10000, "900 MHz 50 Hz: beyond 10 km, %.0f m", lr900);
}

static void testObstruction() {
    std::printf("obstruction and the frame\n");
    CHECK(rcObstructionDb(RcBand::Ghz24, false, 0, 0) == 0, "clear: no loss");
    float wall24 = rcObstructionDb(RcBand::Ghz24, true, 4, 0.3f);
    float wall900 = rcObstructionDb(RcBand::Mhz900, true, 4, 0.3f);
    float deep24 = rcObstructionDb(RcBand::Ghz24, true, 4, 40);
    CHECK(wall24 > 6 && wall24 < 12, "a wall at 2.4 GHz: %.1f dB", wall24);
    CHECK(wall900 < wall24 * 0.7f, "900 MHz goes through better: %.1f vs %.1f", wall900, wall24);
    CHECK(deep24 > wall24 + 10 && deep24 <= 35, "a building's depth: %.1f dB", deep24);
    CHECK(rcObstructionDb(RcBand::Ghz24, false, 2, 0) < wall24, "partial blockage costs less");
    float behind = rcRxAntennaGainDb(RcBand::Ghz24, {0, 0, -1});
    float side = rcRxAntennaGainDb(RcBand::Ghz24, {1, 0, 0});
    float ahead = rcRxAntennaGainDb(RcBand::Ghz24, {0, 0, 1});
    CHECK(side > ahead + 5, "the frame shadows a pilot ahead: side %.1f, ahead %.1f", side, ahead);
    CHECK(behind < side, "along the antenna: the dipole's null, %.1f", behind);
    CHECK(rcRxAntennaGainDb(RcBand::Mhz900, {0, 0, 1}) > ahead, "900 MHz bends round the frame");
}

static void testPerfectAndOff() {
    std::printf("off and close range\n");
    RcLink off;
    off.reset(RcLinkConfig{});
    CHECK(off.step(0.001f, RcLinkInputs{}) == 1, "off: every step delivers");
    CHECK(off.status().lq == 100 && off.status().connected, "off: a perfect link");

    RcLink link;
    link.reset(config(FPV_RC_LINK_2G4_500HZ, 250, true), 7);
    RcLinkInputs in;
    in.pathGainDb = openPath(RcBand::Ghz24, 50, 5);
    in.armed = true;
    int got = run(link, 3.0f, in);
    CHECK(std::abs(got - 1500) <= 2, "500 Hz: 500 packets a second, got %d in 3 s", got);
    CHECK(link.status().lq == 100, "close: LQ 100, got %.0f", link.status().lq);
    CHECK(link.status().txPowerMw == 10, "dynamic power close by: 10 mW, got %d", link.status().txPowerMw);
    CHECK(link.status().rssiDbm > -85 && link.status().rssiDbm < -40, "close RSSI %.1f dBm", link.status().rssiDbm);
    CHECK(link.status().snrDb > 9, "close SNR %.1f dB", link.status().snrDb);

    RcLink fixed;
    fixed.reset(config(FPV_RC_LINK_2G4_500HZ, 500, false), 7);
    run(fixed, 1.0f, in);
    CHECK(fixed.status().txPowerMw == 500, "dynamic off: max power, got %d", fixed.status().txPowerMw);
    fixed.configure(config(FPV_RC_LINK_2G4_500HZ, 300, false));
    CHECK(fixed.status().txPowerMw == 250, "max lowered: 250, got %d", fixed.status().txPowerMw);
}

static void testLossAndResync() {
    std::printf("fading out, failsafe, back in range\n");
    RcLink link;
    link.reset(config(FPV_RC_LINK_2G4_500HZ, 100, false), 3);
    RcLinkInputs in;
    in.armed = true;
    in.kFactor = 3;
    in.dopplerHz = 80;
    // 1 dB above the sensitivity on average, with fading: stale packets
    in.pathGainDb = -105 + 1 - 20;
    run(link, 3.0f, in);
    float lq = link.status().lq;
    CHECK(lq > 20 && lq < 98, "at the edge LQ wanders: %.0f", lq);
    CHECK(link.status().connected, "still connected at the edge");

    // far beyond: nothing arrives, the receiver gives up after its timeout
    in.pathGainDb = -160;
    int got = run(link, 0.5f, in);
    CHECK(got == 0, "out of range: no packets, got %d", got);
    CHECK(link.status().connected, "not yet disconnected after 0.5 s");
    run(link, 1.0f, in);
    CHECK(!link.status().connected, "disconnected after 1.5 s");
    CHECK(link.status().lq == 0, "disconnected: LQ 0");
    CHECK(link.status().rssiDbm <= -129, "disconnected: no RSSI");

    // back in range: the sync takes a moment, then packets flow again
    in.pathGainDb = -40;
    got = run(link, 0.1f, in);
    CHECK(got == 0, "resync: not in the first 0.1 s");
    got = run(link, 1.0f, in);
    CHECK(link.status().connected && got > 300, "reconnected, %d packets", got);
}

static void testDynamicPower() {
    std::printf("dynamic power\n");
    RcLink link;
    link.reset(config(FPV_RC_LINK_2G4_500HZ, 1000, true), 11);
    RcLinkInputs in;
    in.armed = true;
    in.kFactor = 10;
    in.dopplerHz = 50;
    // flying away: the path gets 1 dB weaker every second
    int maxSeen = 10, lowLq = 0, minLq = 100, at100 = 0;
    for (int s = 0; s < 55; s++) {
        in.pathGainDb = -70.0f - (float)s;
        run(link, 1.0f, in);
        maxSeen = std::max(maxSeen, link.status().txPowerMw);
        if (link.status().lq < 90) lowLq++;
        minLq = std::min(minLq, (int)link.status().lq);
        if (s == 30) at100 = link.status().txPowerMw;
    }
    // at -124 dB path, 1 W gives -94 dBm: 11 dB above the sensitivity
    CHECK(maxSeen >= 250, "power ramps up as the drone flies away: %d mW", maxSeen);
    // at -100 dB, 10 mW still gives -90 dBm, 15 dB above the sensitivity
    CHECK(at100 <= 50, "but no more than needed: %d mW at -100 dB", at100);
    CHECK(lowLq <= 2, "LQ stays high while dynamic power follows, %d low seconds (min %d)", lowLq, minLq);

    // coming back: power steps down again
    in.pathGainDb = -60;
    run(link, 6.0f, in);
    CHECK(link.status().txPowerMw == 10, "back close: 10 mW, got %d", link.status().txPowerMw);

    // armed and the link gone: straight to the maximum
    in.pathGainDb = -170;
    run(link, 1.5f, in);
    CHECK(link.status().txPowerMw == 1000, "telemetry lost while armed: max power, got %d", link.status().txPowerMw);

    // disarmed and lost: keeps its level (a battery swap)
    RcLink bench;
    bench.reset(config(FPV_RC_LINK_2G4_500HZ, 1000, true), 5);
    in.armed = false;
    in.pathGainDb = -60;
    run(bench, 2.0f, in);
    in.pathGainDb = -170;
    run(bench, 2.0f, in);
    CHECK(bench.status().txPowerMw == 10, "disarmed: no jump without telemetry, got %d", bench.status().txPowerMw);
}

int main() {
    testModesAndPower();
    testBudget();
    testObstruction();
    testPerfectAndOff();
    testLossAndResync();
    testDynamicPower();
    if (g_failures) {
        std::printf("%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
