// libfpv - physical drone presets.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "config/preset.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "util/fs.h"
#include "video/video_link.h"

namespace fpv {

using json = nlohmann::json;
namespace fs = std::filesystem;

static json readJson(const fs::path &p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open " + pathToUtf8(p));
    try {
        return json::parse(f, nullptr, true, /*ignore_comments=*/true);
    } catch (const std::exception &e) {
        throw std::runtime_error(pathToUtf8(p) + ": " + e.what());
    }
}

static float num(const json &j, const char *key, float def) {
    auto it = j.find(key);
    return (it != j.end() && it->is_number()) ? it->get<float>() : def;
}

static Vec3F vec3f(const json &j, const char *key, Vec3F def) {
    auto it = j.find(key);
    if (it == j.end() || !it->is_object()) return def;
    return {num(*it, "x", def.x), num(*it, "y", def.y), num(*it, "z", def.z)};
}

static std::string str(const json &j, const char *key, const std::string &def = "") {
    auto it = j.find(key);
    return (it != j.end() && it->is_string()) ? it->get<std::string>() : def;
}

// An aspect ratio: 1.333 or "4:3"
static float aspectOf(const json &j, float def) {
    auto it = j.find("aspect");
    if (it == j.end()) return def;
    if (it->is_number()) return std::clamp(it->get<float>(), 0.5f, 4.0f);
    if (it->is_string()) {
        float w = 0, h = 0;
        if (std::sscanf(it->get<std::string>().c_str(), "%f:%f", &w, &h) == 2 && w > 0 && h > 0)
            return std::clamp(w / h, 0.5f, 4.0f);
    }
    return def;
}

// Component files are referenced by their "name" field (SimITL convention).
static json findByName(const fs::path &dir, const std::string &name) {
    std::error_code ec;
    for (auto &e : fs::directory_iterator(dir, ec)) {
        if (e.path().extension() != ".json") continue;
        try {
            json j = readJson(e.path());
            if (str(j, "name") == name || e.path().stem().string() == name) return j;
        } catch (...) {
        }
    }
    throw std::runtime_error("component \"" + name + "\" not found in " + pathToUtf8(dir));
}

// "SProp 5.1x3.7x3" -> 5.1 inch diameter, 3.7 inch pitch
static bool propSizeFromName(const std::string &name, float &diameter, float &pitch) {
    std::smatch m;
    static const std::regex re(R"((\d+(?:\.\d+)?)x(\d+(?:\.\d+)?))");
    if (!std::regex_search(name, m, re)) return false;
    diameter = std::stof(m[1].str());
    pitch = std::stof(m[2].str());
    return true;
}

DronePreset loadPresetFile(const std::string &quadJsonPath) {
    fs::path qp = pathFromUtf8(quadJsonPath);
    fs::path root = qp.parent_path().parent_path();
    json q = readJson(qp);

    DronePreset p;
    p.name = str(q, "name", qp.stem().string());
    p.fileName = qp.stem().string();
    p.description = str(q, "description");
    StateInit &s = p.init;

    // --- frame ---------------------------------------------------------
    s.frameDragArea = vec3f(q, "frameDragArea", {0.008f, 0.0077f, 0.008f});
    s.frameDragConstant = num(q, "frameDragConstant", 1.2f);
    float frameMass = num(q, "mass", 0.1f);
    Vec3F inv = vec3f(q, "invInertia", {900.0f, 480.0f, 900.0f});
    // SimITL flips the yaw inverse inertia to match its handedness.
    s.quadInvInertia = {inv.x, -std::fabs(inv.y), inv.z};
    s.minPropWashSpeed = num(q, "minPropWashSpeed", 1.0f);
    s.maxPropWashSpeed = num(q, "maxPropWashSpeed", 18.0f);
    s.propWashAngleOfAttack = num(q, "propWashAngleOfAttack", 0.5f);
    s.propWashFactor = num(q, "propWashFactor", 1.0f);
    // open props ~0.6 (Faessler et al. 2018); ducts add their own ram drag
    s.rotorDragFactor = num(q, "rotorDragFactor", 0.6f);
    s.ductThrustGain = num(q, "ductThrustGain", 0.0f);
    s.electronicsPower = num(q, "electronicsPower", 5.0f);
    const char *posKeys[4] = {"motor1Pos", "motor2Pos", "motor3Pos", "motor4Pos"};
    const char *dirKeys[4] = {"motor1Dir", "motor2Dir", "motor3Dir", "motor4Dir"};
    const float defDir[4] = {-1, 1, 1, -1};
    const Vec3F defPos[4] = {{0.08f, 0.005f, -0.08f}, {0.08f, 0.005f, 0.08f}, {-0.08f, 0.005f, -0.08f},
                             {-0.08f, 0.005f, 0.08f}};
    for (int i = 0; i < 4; i++) {
        s.quadMotorPos[i] = vec3f(q, posKeys[i], defPos[i]);
        s.quadMotorDir[i] = num(q, dirKeys[i], defDir[i]);
    }
    s.ambientTemp = num(q, "ambientTemp", 25.0f);
    p.gyroBaseNoiseAmp = num(q, "gyroBaseNoiseAmp", 0.015f);
    p.gyrobaseNoiseFreq = num(q, "gyrobaseNoiseFreq", 333.0f);
    p.frameHarmonic1Amp = num(q, "frameHarmonic1Amp", 0.0f);
    p.frameHarmonic1Freq = num(q, "frameHarmonic1Freq", 0.0f);
    p.frameHarmonic2Amp = num(q, "frameHarmonic2Amp", 0.0f);
    p.frameHarmonic2Freq = num(q, "frameHarmonic2Freq", 0.0f);
    p.maxGroundEffectDist = num(q, "maxGroundEffectDist", 0.25f);
    p.soundMaxRpm = num(q, "soundMaxRpm", 30000.0f);
    float variance[4] = {1, 1, 1, 1};
    if (q.contains("motorVariance") && q["motorVariance"].is_object()) {
        const json &v = q["motorVariance"];
        variance[0] = num(v, "x", 1);
        variance[1] = num(v, "y", 1);
        variance[2] = num(v, "z", 1);
        variance[3] = num(v, "w", 1);
    }

    // --- motor ---------------------------------------------------------
    json m = findByName(root / "motor", str(q, "motor"));
    for (int i = 0; i < 4; i++) {
        s.motorKV[i] = num(m, "motorKV", 3300.0f) * variance[i];
        s.motorR[i] = num(m, "motorR", 0.135f);
        s.motorI0[i] = num(m, "motorI0", 0.8f);
    }
    s.motorRth = num(m, "motorRth", 12.0f);
    s.motorCth = num(m, "motorCth", 9.5f);
    s.motorMaxT = num(m, "motorMaxT", 128.0f);
    Vec3F imb = vec3f(m, "motorImbalance", {13.0f, 7.0f, 5.0f});
    p.motorImbalance = {imb.x, imb.y, imb.z};
    float motorMass = num(m, "mass", 0.019f);

    // --- propeller -----------------------------------------------------
    json pr = findByName(root / "propeller", str(q, "propeller"));
    s.propBladeCount = (uint8_t)num(pr, "bladeCount", 3);
    p.propBladeCount = s.propBladeCount;
    s.propMaxRpm = num(pr, "propMaxRpm", 36000.0f);
    s.propAFactor = num(pr, "propAFactor", 6.25e-9f);
    s.propTorqueFactor = num(pr, "propTorqueFactor", 0.009f);
    s.propInertia = num(pr, "propInertia", 2.45e-6f);
    s.propThrustFactor = vec3f(pr, "propThrustFactor", {-0.0000111f, -0.14f, 9.55f});
    s.propHarmonic1Amp = num(pr, "propHarmonic1Amp", 0.1f);
    s.propHarmonic2Amp = num(pr, "propHarmonic2Amp", 0.3f);
    float propMass = num(pr, "mass", 0.0028f);
    float nameDiameter = 5.0f, namePitch = 0.0f;
    propSizeFromName(str(pr, "name"), nameDiameter, namePitch);
    float diameterInch = num(pr, "diameterInch", nameDiameter);
    // a known pitch selects libfpv's thrust model (rpm^2 and advance ratio)
    s.propPitch = num(pr, "pitchInch", namePitch) * 0.0254f;
    s.propDiameter = diameterInch * 0.0254f;
    p.collision.propRadius = diameterInch * 0.0254f * 0.5f;
    p.collision.propDurability = num(pr, "durability", 11.0f);

    // --- battery -------------------------------------------------------
    json b = findByName(root / "battery", str(q, "battery"));
    s.maxVoltageSag = num(b, "maxVoltageSag", 1.3f);
    s.batInternalResistance = num(b, "internalResistance", 0.0f);
    s.quadBatCellCount = (uint8_t)num(b, "batCellCount", 4);
    s.quadBatCapacityCharged = num(b, "batCapacityCharged", 850.0f);
    s.quadBatCapacity = num(b, "batCapacity", 850.0f);
    float batMass = num(b, "mass", 0.11f);

    s.quadMass = frameMass + 4 * motorMass + 4 * propMass + batMass;

    // --- vtx (optional) ------------------------------------------------
    if (!str(q, "vtx").empty()) {
        try {
            json v = findByName(root / "vtx", str(q, "vtx"));
            std::string type = str(v, "type", "analog");
            p.vtx.name = str(v, "name");
            p.vtx.digital = type != "analog";
            p.vtx.noiseFloor = num(v, "noiseFloor", p.vtx.noiseFloor);
            p.vtx.noiseFactor = num(v, "noiseFactor", p.vtx.noiseFactor);
            p.vtx.rangeM = num(v, "range", p.vtx.rangeM);
            // no power given (SimITL's own files): the one whose first
            // sparkles come at `range` in the open
            float derivedDbm = -94.0f + kVideoSystemLossDb - kVideoTxGainDbi - kVideoRxPatchDbi +
                               freeSpaceLossDb(std::max(p.vtx.rangeM, 10.0f));
            p.vtx.powerMw = num(v, "powerMw", std::pow(10.0f, derivedDbm / 10.0f));
            p.vtx.powerNoise = num(v, "powerNoise", p.vtx.powerNoise);
            p.vtx.minVoltage = num(v, "minCellVoltage", p.vtx.minVoltage);
            p.vtx.height = (int)num(v, "height", type == "hdzero" ? 720.0f : 1080.0f);
            p.vtx.fps = num(v, "fps", 60.0f);
            p.vtx.freezes = v.value("freezes", type != "hdzero");
            p.vtx.sensitivityDbm = num(v, "sensitivityDbm", type == "hdzero" ? -92.0f : -98.0f);
        } catch (...) {
        }
    }

    // --- libfpv extensions (optional) ------------------------------------
    float armLen = std::sqrt(s.quadMotorPos[0].x * s.quadMotorPos[0].x + s.quadMotorPos[0].z * s.quadMotorPos[0].z);
    p.collision.bodyRadius = std::clamp(armLen * 0.4f, 0.02f, 0.06f);
    p.collision.motorRadius = std::clamp(armLen * 0.2f, 0.01f, 0.03f);
    p.vtxShadowDb = std::clamp(num(q, "vtxAntennaShadowDb", defaultVtxShadowDb(armLen)), 0.0f, 30.0f);
#if defined(LIBFPV_KAMIKAZE)
    if (q.contains("payload") && q["payload"].is_object()) {
        p.payloadMass = num(q["payload"], "mass", -1.0f);
        p.payloadDrop = num(q["payload"], "drop", -1.0f);
    }
#endif
    if (q.contains("collision") && q["collision"].is_object()) {
        const json &c = q["collision"];
        p.collision.bodyRadius = num(c, "bodyRadius", p.collision.bodyRadius);
        p.collision.motorRadius = num(c, "motorRadius", p.collision.motorRadius);
        p.collision.propRadius = num(c, "propRadius", p.collision.propRadius);
        p.collision.propDurability = num(c, "propDurability", p.collision.propDurability);
    }
    p.camera.aspect = p.vtx.digital ? 16.0f / 9.0f : 4.0f / 3.0f;
    p.camera.offset = {0.0f, 0.02f, std::clamp(armLen * 0.4f, 0.02f, 0.05f)};
    if (q.contains("camera") && q["camera"].is_object()) {
        const json &c = q["camera"];
        // the camera model: presets/camera/<name>.json
        if (!str(c, "model").empty()) {
            try {
                json m = findByName(root / "camera", str(c, "model"));
                p.camera.model = str(m, "name");
                p.camera.aspect = aspectOf(m, p.camera.aspect);
                p.camera.tvl = num(m, "tvl", p.camera.tvl);
                p.camera.system = str(m, "system", "NTSC") == "PAL" ? FPV_VIDEO_PAL : FPV_VIDEO_NTSC;
                p.camera.sharpening = num(m, "sharpening", p.camera.sharpening);
                // a digital camera sets the mode it streams in
                if (p.vtx.digital) {
                    p.vtx.height = (int)num(m, "height", (float)p.vtx.height);
                    p.vtx.fps = num(m, "fps", p.vtx.fps);
                }
            } catch (...) {
            }
        }
        p.camera.aspect = aspectOf(c, p.camera.aspect);
        p.camera.system = str(c, "system").empty() ? p.camera.system
                          : str(c, "system") == "PAL" ? FPV_VIDEO_PAL : FPV_VIDEO_NTSC;
        p.camera.uptiltDeg = num(c, "uptiltDeg", p.camera.uptiltDeg);
        p.camera.fovYDeg = num(c, "fovYDeg", p.camera.fovYDeg);
        p.camera.k1 = num(c, "k1", p.camera.k1);
        p.camera.k2 = num(c, "k2", p.camera.k2);
        p.camera.zoom = num(c, "zoom", p.camera.zoom);
        Vec3F off = vec3f(c, "offset", {p.camera.offset.x, p.camera.offset.y, p.camera.offset.z});
        p.camera.offset = {off.x, off.y, off.z};
    }

    // Betaflight tune shipped next to the preset: <file>.diff.txt
    fs::path diffPath = qp.parent_path() / (qp.stem().string() + ".diff.txt");
    std::ifstream df(diffPath, std::ios::binary);
    if (df) {
        std::stringstream ss;
        ss << df.rdbuf();
        p.betaflightDiff = ss.str();
    }
    return p;
}

std::vector<DronePreset> loadPresetDirectory(const std::string &dir, std::vector<std::string> &errors) {
    std::vector<DronePreset> out;
    std::error_code ec;
    fs::path quadDir = pathFromUtf8(dir) / "quad";
    std::vector<fs::path> files;
    for (auto &e : fs::directory_iterator(quadDir, ec)) {
        if (e.path().extension() == ".json") files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());
    for (auto &f : files) {
        try {
            out.push_back(loadPresetFile(pathToUtf8(f)));
        } catch (const std::exception &e) {
            errors.push_back(e.what());
        }
    }
    return out;
}

DronePreset builtinDefaultPreset() {
    // assets/presets/quad/freestyler-5.json with its components
    DronePreset p;
    p.name = "Freestyler (5\" 6S)";
    p.fileName = "builtin-default";
    StateInit &s = p.init;
    s.frameDragArea = {0.0095f, 0.0085f, 0.0095f};
    s.frameDragConstant = 1.2f;
    s.rotorDragFactor = 0.6f;
    s.quadInvInertia = {620.0f, -340.0f, 620.0f};
    const Vec3F pos[4] = {{0.0795f, 0.005f, -0.0795f}, {0.0795f, 0.005f, 0.0795f}, {-0.0795f, 0.005f, -0.0795f},
                          {-0.0795f, 0.005f, 0.0795f}};
    const float dir[4] = {-1, 1, 1, -1};
    const float variance[4] = {0.985f, 0.995f, 0.99f, 1.0f};
    for (int i = 0; i < 4; i++) {
        s.quadMotorPos[i] = pos[i];
        s.quadMotorDir[i] = dir[i];
        s.motorKV[i] = 1950.0f * variance[i];
        s.motorR[i] = 0.125f;
        s.motorI0[i] = 1.4f;
    }
    s.motorRth = 7.0f;
    s.motorCth = 18.0f;
    s.motorMaxT = 128.0f;
    s.propBladeCount = 3;
    s.propMaxRpm = 33000.0f;
    s.propAFactor = 1.423e-8f;
    s.propTorqueFactor = 0.0095f;
    s.propInertia = 7.5e-6f;
    s.propThrustFactor = {0.0f, 0.0f, 15.5f};
    s.propPitch = 3.0f * 0.0254f;
    s.propDiameter = 5.1f * 0.0254f;
    s.propHarmonic1Amp = 0.02f;
    s.propHarmonic2Amp = 0.04f;
    s.maxVoltageSag = 3.0f;
    s.quadBatCellCount = 6;
    s.quadBatCapacityCharged = 1100.0f;
    s.quadBatCapacity = 1100.0f;
    s.quadMass = 0.21f + 4 * 0.035f + 4 * 0.0043f + 0.185f;
    s.ambientTemp = 25.0f;
    s.minPropWashSpeed = 1.0f;
    s.maxPropWashSpeed = 20.0f;
    s.propWashAngleOfAttack = 0.1f;
    s.propWashFactor = 1.1f;
    p.gyroBaseNoiseAmp = 0.005f;
    p.gyrobaseNoiseFreq = 233.0f;
    p.frameHarmonic1Amp = 0.0175f;
    p.frameHarmonic1Freq = 210.0f;
    p.frameHarmonic2Amp = 0.004f;
    p.frameHarmonic2Freq = 420.0f;
    p.motorImbalance = {8.0f, 5.0f, 4.0f};
    p.maxGroundEffectDist = 0.33f;
    p.soundMaxRpm = 33000.0f;
    // a classic analog setup: TBS Unify Pro32 HV at 400 mW, Caddx Ratel 2 in 4:3
    p.vtx.name = "TBS Unify Pro32 HV";
    p.vtx.digital = false;
    p.vtx.powerMw = 400.0f;
    p.vtx.powerNoise = 0.12f;
    p.camera.model = "Caddx Ratel 2";
    p.camera.aspect = 4.0f / 3.0f;
    p.camera.tvl = 1200.0f;
    p.camera.sharpening = 0.55f;
    p.collision.propRadius = 5.1f * 0.0254f * 0.5f;
    p.vtxShadowDb = defaultVtxShadowDb(0.0795f * 1.41421356f);
    return p;
}

}  // namespace fpv
