// libfpv - the Server calls as messages (split builds).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "simapi/wire.h"

#include "libfpv.h"

namespace fpv {
namespace wire {

std::string protocolId() {
    std::string id = "libfpv-wire 3 " LIBFPV_VERSION_STRING;
#if defined(LIBFPV_KAMIKAZE)
    id += " k";
#endif
    // the structs that go as they are must have the same layout on both ends
    id += " " + std::to_string(sizeof(void *)) + "/" + std::to_string(sizeof(FlightSnapshot)) + "/" +
          std::to_string(sizeof(ImpactEvent)) + "/" + std::to_string(sizeof(PhysicsParams)) + "/" +
          std::to_string(sizeof(Triangle)) + "/" + std::to_string(sizeof(Material)) + "/" +
          std::to_string(sizeof(PortalLink)) + "/" + std::to_string(kChannelCount);
    return id;
}

void put(Writer &w, const std::string &s) {
    put(w, (uint32_t)s.size());
    w.bytes(s.data(), s.size());
}

void get(Reader &r, std::string &s) {
    uint32_t n = 0;
    get(r, n);
    s.clear();
    if (!r.ok || n > r.left) {
        r.ok = false;
        return;
    }
    s.assign(r.p, n);
    r.p += n;
    r.left -= n;
}

void put(Writer &w, const CameraSettings &c) {
    pack(w, c.uptiltDeg, c.fovYDeg, c.aspect, c.k1, c.k2, c.zoom, c.offset, c.model, c.tvl, c.system, c.sharpening,
         c.displayAspect, c.osdRows);
}
void get(Reader &r, CameraSettings &c) {
    unpack(r, c.uptiltDeg, c.fovYDeg, c.aspect, c.k1, c.k2, c.zoom, c.offset, c.model, c.tvl, c.system, c.sharpening,
           c.displayAspect, c.osdRows);
}

void put(Writer &w, const VtxSettings &v) {
    pack(w, v.name, v.digital, v.noiseFloor, v.noiseFactor, v.rangeM, v.powerMw, v.powerNoise, v.minVoltage, v.height,
         v.fps, v.freezes, v.sensitivityDbm);
}
void get(Reader &r, VtxSettings &v) {
    unpack(r, v.name, v.digital, v.noiseFloor, v.noiseFactor, v.rangeM, v.powerMw, v.powerNoise, v.minVoltage, v.height,
           v.fps, v.freezes, v.sensitivityDbm);
}

void put(Writer &w, const DronePreset &p) {
    pack(w, p.name, p.fileName, p.description, p.physics, p.gyroBaseNoiseAmp, p.gyrobaseNoiseFreq, p.frameHarmonic1Amp,
         p.frameHarmonic1Freq, p.frameHarmonic2Amp, p.frameHarmonic2Freq, p.motorImbalance, p.maxGroundEffectDist,
         p.soundMaxRpm, p.propBladeCount, p.camera, p.vtx, p.vtxShadowDb, p.collision, p.betaflightDiff);
#if defined(LIBFPV_KAMIKAZE)
    pack(w, p.payloadMass, p.payloadDrop);
#endif
}
void get(Reader &r, DronePreset &p) {
    unpack(r, p.name, p.fileName, p.description, p.physics, p.gyroBaseNoiseAmp, p.gyrobaseNoiseFreq, p.frameHarmonic1Amp,
           p.frameHarmonic1Freq, p.frameHarmonic2Amp, p.frameHarmonic2Freq, p.motorImbalance, p.maxGroundEffectDist,
           p.soundMaxRpm, p.propBladeCount, p.camera, p.vtx, p.vtxShadowDb, p.collision, p.betaflightDiff);
#if defined(LIBFPV_KAMIKAZE)
    unpack(r, p.payloadMass, p.payloadDrop);
#endif
}

void put(Writer &w, const ServerConfig &c) {
    pack(w, c.eepromDir, c.preset, c.defaultDiff, c.userDefaults, c.configurator, c.configuratorPort, c.threaded,
         c.simRateHz, c.autoPauseSeconds);
}
void get(Reader &r, ServerConfig &c) {
    unpack(r, c.eepromDir, c.preset, c.defaultDiff, c.userDefaults, c.configurator, c.configuratorPort, c.threaded,
           c.simRateHz, c.autoPauseSeconds);
}

void put(Writer &w, const WindState &s) { pack(w, s.on, s.mean, s.convection, s.sigma, s.lengthScale, s.sources); }
void get(Reader &r, WindState &s) { unpack(r, s.on, s.mean, s.convection, s.sigma, s.lengthScale, s.sources); }

void put(Writer &w, const HoverFeedback &f) { pack(w, f.preset, f.climbRate, f.upright, f.airborne); }
void get(Reader &r, HoverFeedback &f) { unpack(r, f.preset, f.climbRate, f.upright, f.airborne); }

namespace {
constexpr uint32_t kMaxMessage = 1u << 30;
}

bool sendMessage(LoopbackSocket &s, uint16_t op, const std::string &payload) {
    if (payload.size() > kMaxMessage) return false;
    char head[6];
    uint32_t n = (uint32_t)payload.size();
    std::memcpy(head, &n, 4);
    std::memcpy(head + 4, &op, 2);
    // one send for small messages: the step's channels go many times a second
    if (payload.size() <= 256) {
        char buf[6 + 256];
        std::memcpy(buf, head, 6);
        std::memcpy(buf + 6, payload.data(), payload.size());
        return s.sendAll(buf, 6 + payload.size());
    }
    return s.sendAll(head, 6) && s.sendAll(payload.data(), payload.size());
}

bool recvMessage(LoopbackSocket &s, uint16_t &op, std::string &payload) {
    char head[6];
    if (!s.recvAll(head, 6)) return false;
    uint32_t n;
    std::memcpy(&n, head, 4);
    std::memcpy(&op, head + 4, 2);
    if (n > kMaxMessage) return false;
    payload.resize(n);
    return n == 0 || s.recvAll(&payload[0], n);
}

}  // namespace wire
}  // namespace fpv
