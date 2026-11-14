// libfpv - physical drone presets (frame, motor, propeller, battery, VTX,
// camera). File format: SimITL's preset JSON, see docs/PRESETS.md.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "fpv_math.h"
#include "network/packets.h"

namespace fpv {

struct CameraSettings {
    float uptiltDeg = 25.0f;
    float fovYDeg = 100.0f;     // vertical
    float aspect = 4.0f / 3.0f;
    float k1 = 0.0f, k2 = 0.0f; // Brown-Conrady radial distortion, r = 1 at the corners
                                // (used when the player turns the fisheye slider off)
    float zoom = 1.0f;
    Vec3 offset{0.0f, 0.02f, 0.03f};  // from the centre of mass, metres
    // the camera model (presets/camera, "model" in the quad's camera object)
    std::string model;
    float tvl = 1000.0f;         // analog horizontal resolution, TV lines
    int system = 0;              // FPV_VIDEO_NTSC or FPV_VIDEO_PAL (analog)
    float sharpening = 0.5f;     // the camera's edge enhancement 0..1
};

struct VtxSettings {
    std::string name;
    bool digital = false;
    float noiseFloor = 0.014f;   // SimITL's fields, unused by libfpv's link model
    float noiseFactor = 1.4f;
    float rangeM = 1100.0f;      // when powerMw is missing, the power is derived from it
    float powerMw = 25.0f;
    float powerNoise = 0.15f;    // 0..1: motor and ESC noise reaching the video (analog)
    float minVoltage = 0.0f;     // per cell: below it the VTX browns out (1S whoops), 0 = never
    // digital: the transmitted mode, and how the link fails
    int height = 720;
    float fps = 60.0f;
    bool freezes = false;        // DJI / Walksnail hold the frame; HDZero breaks up
    float sensitivityDbm = -95.0f;
};

struct CollisionSettings {
    float bodyRadius = 0.035f;   // sphere at the centre of mass
    float motorRadius = 0.018f;  // sphere per motor bell
    float propRadius = 0.065f;   // propeller disc radius
    float propDurability = 11.0f;// relative-speed budget before a prop breaks
};

struct DronePreset {
    std::string name;       // display name
    std::string fileName;   // without extension
    std::string description;
    StateInit init{};       // SimITL physical parameters (eepromName left empty)
    // runtime noise inputs from the preset
    float gyroBaseNoiseAmp = 0, gyrobaseNoiseFreq = 0;
    float frameHarmonic1Amp = 0, frameHarmonic1Freq = 0;
    float frameHarmonic2Amp = 0, frameHarmonic2Freq = 0;
    Vec3 motorImbalance{0, 0, 0};
    float maxGroundEffectDist = 0.25f;
    float soundMaxRpm = 30000.0f;
    float propBladeCount = 3;
    CameraSettings camera;
    VtxSettings vtx;
    // dB the frame and battery cost the VTX's signal toward the front
    // (txAntennaGainDb); "vtxAntennaShadowDb" in the quad JSON, else from
    // the frame's size
    float vtxShadowDb = 8.0f;
    CollisionSettings collision;
    // Betaflight CLI tune applied when the player first uses this preset.
    std::string betaflightDiff;
#if defined(LIBFPV_KAMIKAZE)
    // the warhead this quad carries ("payload" in the quad JSON):
    // kg, and metres below the frame centre; < 0 = automatic
    float payloadMass = -1, payloadDrop = -1;
#endif
};

// Loads every quad preset in <dir>/quad, resolving component references in
// the sibling folders. Errors are reported per file in `errors`.
std::vector<DronePreset> loadPresetDirectory(const std::string &dir, std::vector<std::string> &errors);
DronePreset loadPresetFile(const std::string &quadJsonPath);  // throws std::runtime_error

// The Freestyler 5" (the default preset), used when no preset files are found.
DronePreset builtinDefaultPreset();

}  // namespace fpv
