// libfpv - C API: OSD, audio, textures, overlay.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>

#include "audio/synth.h"
#include "lib.h"
#include "osd/osd.h"
#include "overlay/overlay.h"

using namespace fpv;

namespace fpv {

// The audio synth is read from the host's audio thread without the API
// mutex, so it lives behind an atomic pointer and is never destroyed.
static std::atomic<AudioSynth *> g_audio{nullptr};
static std::atomic<bool> g_audioPulled{false};

bool audioPulled() { return g_audioPulled.load(std::memory_order_relaxed); }

namespace {
constexpr float kSpeedOfSound = 343.0f;  // m/s, dry air at 20 C
// The onboard sound is the level heard this far from the drone, so the two
// match when the drone flies by the pilot.
constexpr float kReferenceDistance = 2.0f;
// Air absorption grows about with the frequency squared (ISO 9613-1, 20 C,
// 70 % humidity: 4.7, 9.7, 33 and 117 dB/km at 1, 2, 4 and 8 kHz), so the
// highs go first: a low-pass where the loss over the distance is 3 dB.
constexpr float kAirAbsorption = 2.0e-9f;  // dB per metre per Hz^2
}  // namespace

AudioSynth &audioSynth() {
    AudioSynth *a = g_audio.load();
    if (!a) {
        AudioSynth *fresh = new AudioSynth();
        if (g_audio.compare_exchange_strong(a, fresh)) {
            a = fresh;
        } else {
            delete fresh;
        }
    }
    return *a;
}

OsdRenderer &osdRenderer() {
    Lib &L = lib();
    if (!L.osd) L.osd = new OsdRenderer();
    return *L.osd;
}

bool hearsFromPilot() {
    Lib &L = lib();
    switch (L.settings.hearFrom) {
    case HearFrom::Drone: return false;
    case HearFrom::Pilot: return true;
    default: return L.view == FPV_VIEW_LOS || L.view == FPV_VIEW_PILOT;
    }
}

// How the drone sounds at the ear (internal frame, metres). See
// docs/DECISIONS.md "Hearing the drone from the pilot".
static Hearing hearing(Vec3 dronePos, Vec3 droneVel) {
    Lib &L = lib();
    const Lib::HearingProbe &pr = L.hearingProbe;
    Hearing h;
    h.fromPilot = hearsFromPilot() ? 1.0f : 0.0f;
    Vec3 ear = earPosition();
    Vec3 d = dronePos - ear;
    float r = length(d);
    float rr = std::max(r, 0.3f);
    // what the ear hears now left the drone when it was where it was then:
    // tau = r(t - tau) / c gives tau = r / (c + dr/dt), and its change the
    // moving source's Doppler shift, f c / (c - v)
    float closing = r > 1e-3f ? dot(droneVel, d / r) : 0.0f;
    h.delay = r / std::max(0.5f * kSpeedOfSound, kSpeedOfSound + closing);
    // spherical spreading: -6 dB per doubling of the distance
    float spread = std::min(1.0f, kReferenceDistance / rr);
    // Walls: a wall's mass law (TL = 20 log10(f m) - 47 dB) lets the lows
    // through and blocks the highs, and the sound that bends around an
    // edge instead (Maekawa: 10 log10(3 + 20 N) dB) also loses its highs
    // first. One wall or building edge ~12 dB and a cutoff near 1.6 kHz,
    // each more one less, down to a low rumble.
    float walls = pr.valid ? pr.walls : 0.0f;
    float wallDb = 12.0f * std::log2(1.0f + walls);
    h.gain = spread * std::pow(10.0f, -wallDb / 20.0f);
    h.wallCutoff = 20000.0f / (1.0f + 11.5f * std::pow(walls, 1.5f));
    h.airCutoff = std::min(20000.0f, std::sqrt(3.0f / (kAirAbsorption * rr)));
    // where the drone is for the ear: the game's listener, or facing it
    if (L.listenerSet && r > 1e-3f) h.pan = std::clamp(rotate(conj(L.listenerRot), d / r).x, -1.0f, 1.0f);
    // the ground's reflection (an image source under the ground below the
    // ear): a copy that arrives a little later, comb-filtering the direct
    // sound as the drone climbs or comes closer
    float hEar = kPilotEarHeight, hDrone = d.y + hEar;
    if (hDrone > 0 && r > 0.5f) {
        float horiz2 = std::max(0.0f, r * r - d.y * d.y);
        float reflected = std::sqrt(horiz2 + (hDrone + hEar) * (hDrone + hEar));
        h.groundDelay = (reflected - r) / kSpeedOfSound;
        h.groundGain = 0.5f * r / reflected;  // grass and dirt keep about half
    }
    if (pr.valid) {
        // a slap echo off the walls around the ear (urban canyon, a yard),
        // there and back
        if (pr.reflectors > 0 && pr.echoDistance > 0) {
            h.echoDelay = 2 * pr.echoDistance / kSpeedOfSound;
            h.echoGain = 0.35f * pr.reflectors * std::min(1.0f, 10.0f / pr.echoDistance) * (1 - 0.6f * pr.enclosure);
        }
        // reverberation: a room's diffuse sound hardly falls with the
        // distance, the direct sound does, so far drones sound wetter
        float wet = 0.6f * pr.enclosure + 0.15f * pr.reflectors;
        float wallLoss = h.gain / std::max(spread, 1e-6f);
        h.reverb = wet * wallLoss * std::min(1.0f, std::sqrt(kReferenceDistance / rr));
        h.reverbTime = std::clamp(pr.enclosure > 0.5f ? 0.3f + 0.04f * pr.roomSize : 0.6f, 0.3f, 2.5f);
        h.onboardReverb = 0.25f * wet;
    }
    return h;
}

// Called by fpv_drone_tick with the API mutex held.
void updatePresentation(const fpv_frame &f, Vec3 dronePos, Vec3 droneVel, bool beeperSwitch) {
    Lib &L = lib();
    const DronePreset &p = currentPreset();
    AudioParams ap;
    float arm = std::sqrt(p.init.quadMotorPos[0].x * p.init.quadMotorPos[0].x +
                          p.init.quadMotorPos[0].z * p.init.quadMotorPos[0].z);
    for (int i = 0; i < 4; i++) {
        ap.rpm[i] = f.motors[i].rpm;
        ap.load[i] = std::clamp(std::fabs(f.motors[i].current_amp) / 35.0f, 0.0f, 1.0f);
        ap.damage[i] = f.motors[i].prop_damage;
        ap.pan[i] = arm > 0 ? std::clamp(p.init.quadMotorPos[i].x / arm, -1.0f, 1.0f) : 0.0f;
    }
    ap.maxRpm = p.soundMaxRpm;
    ap.blades = p.propBladeCount;
    ap.beeper = (f.flags & FPV_DRONE_BEEPER) != 0;
    ap.motorBeep = ap.beeper && beeperSwitch;
    ap.paused = (f.flags & FPV_DRONE_PAUSED) != 0;
    ap.hearing = hearing(dronePos, droneVel);
    audioSynth().setParams(ap);
    audioSynth().setVolume(L.settings.audioVolume, L.settings.beepVolume);
}

}  // namespace fpv

extern "C" {

FPV_API fpv_result fpv_texture_get(uint64_t id, fpv_texture *out) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!out) return setError(FPV_ERROR_INVALID_ARGUMENT, "out is NULL");
    if (id == FPV_TEXTURE_OSD_FONT) {
        OsdRenderer &r = osdRenderer();
        out->width = OsdRenderer::kAtlasW;
        out->height = OsdRenderer::kAtlasH;
        out->rgba = r.atlas().data();
        out->revision = 1;
        return FPV_OK;
    }
    if (id == FPV_TEXTURE_OVERLAY_FONT) {
        Overlay &o = overlay();
        return o.fontTexture(*out) ? FPV_OK : setError(FPV_ERROR_INTERNAL, "overlay font unavailable");
    }
    return setError(FPV_ERROR_NOT_FOUND, "unknown texture id %llu", (unsigned long long)id);
}

FPV_API const fpv_draw_data *fpv_osd_draw_data(const fpv_frame *frame, float width, float height,
                                              fpv_osd_style style) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!frame || !(width > 0) || !(height > 0)) {
        setError(FPV_ERROR_INVALID_ARGUMENT, "frame and a positive size are required");
        return nullptr;
    }
    return osdRenderer().drawData(frame->osd, width, height, style);
}

FPV_API fpv_result fpv_osd_render_rgba(const fpv_frame *frame, uint8_t *rgba, uint32_t width, uint32_t height,
                                       uint32_t stride, fpv_osd_style style) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!frame || !rgba || !width || !height) return setError(FPV_ERROR_INVALID_ARGUMENT, "frame, buffer and size required");
    if (stride == 0) stride = width * 4;
    if (stride < width * 4) return setError(FPV_ERROR_INVALID_ARGUMENT, "stride smaller than width * 4");
    osdRenderer().rasterize(frame->osd, rgba, width, height, stride, style);
    return FPV_OK;
}

FPV_API fpv_result fpv_audio_configure(uint32_t sampleRate, uint32_t channels) {
    if (sampleRate < 8000 || channels < 1) return setError(FPV_ERROR_INVALID_ARGUMENT, "bad audio format");
    audioSynth().configure(sampleRate, channels);
    return FPV_OK;
}

FPV_API uint32_t fpv_audio_read(float *out, uint32_t frames) {
    if (!out || !frames) return 0;
    g_audioPulled.store(true, std::memory_order_relaxed);
    return audioSynth().read(out, frames);
}

FPV_API void fpv_audio_set_volume(float v) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    L.settings.audioVolume = std::clamp(std::isfinite(v) ? v : 0.1f, 0.0f, 2.0f);
    audioSynth().setVolume(L.settings.audioVolume, L.settings.beepVolume);
}

FPV_API void fpv_audio_set_beep_volume(float v) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    L.settings.beepVolume = std::clamp(std::isfinite(v) ? v : 0.1f, 0.0f, 2.0f);
    audioSynth().setVolume(L.settings.audioVolume, L.settings.beepVolume);
}

FPV_API void fpv_audio_set_listener(const float position[3], const float rotation[4]) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    L.listenerSet = false;
    if (!position) return;
    Vec3 p = L.conv.posToInternal({position[0], position[1], position[2]});
    if (!finite(p)) return;
    Quat q{0, 0, 0, 1};
    if (rotation) q = L.conv.rotToInternal({rotation[0], rotation[1], rotation[2], rotation[3]});
    if (!std::isfinite(q.w) || qdot(q, q) < 1e-12f) q = Quat{0, 0, 0, 1};
    L.listenerPos = p;
    L.listenerRot = normalize(q);
    L.listenerSet = true;
}

// --- input and overlay ---------------------------------------------------------

FPV_API void fpv_input_key(int key, int down) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (overlay().capturingKey()) {
        overlay().key(key, down != 0);
        return;
    }
    if (key == L.settings.overlayToggleKey && down) {
        overlay().toggle();
        return;
    }
    if (overlay().isOpen()) {
        overlay().key(key, down != 0);
        L.input.key(key, false);  // no flying while typing
        return;
    }
    L.input.key(key, down != 0);
}

FPV_API void fpv_overlay_set_open(int open) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().setOpen(open != 0);
}
FPV_API int fpv_overlay_is_open(void) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    return overlay().isOpen() ? 1 : 0;
}
FPV_API int fpv_overlay_wants_input(void) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    return overlay().isOpen() ? 1 : 0;
}
FPV_API void fpv_overlay_set_toggle_key(int key) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    L.settings.overlayToggleKey = key;
    if (L.initialized) saveSettings();
}
FPV_API void fpv_overlay_set_display(float w, float h, float scale) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().setDisplay(w, h, scale);
}
FPV_API void fpv_overlay_mouse_move(float x, float y) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().mouseMove(x, y);
}
FPV_API void fpv_overlay_mouse_button(int b, int down) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().mouseButton(b, down != 0);
}
FPV_API void fpv_overlay_mouse_wheel(float dx, float dy) {
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().mouseWheel(dx, dy);
}
FPV_API void fpv_overlay_text(const char *utf8) {
    if (!utf8) return;
    std::lock_guard<std::recursive_mutex> lock(lib().mutex);
    overlay().text(utf8);
}
FPV_API void fpv_overlay_set_action(uint32_t id, const char *label) {
    if (!id) return;
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    auto &v = L.hostActions;
    auto it = std::find_if(v.begin(), v.end(), [id](const Lib::HostAction &a) { return a.id == id; });
    if (!label || !label[0]) {
        if (it != v.end()) v.erase(it);
        L.clickedActions.erase(std::remove(L.clickedActions.begin(), L.clickedActions.end(), id),
                               L.clickedActions.end());
    } else if (it != v.end()) {
        it->label = label;
    } else {
        v.push_back({id, label});
    }
}
FPV_API uint32_t fpv_overlay_poll_action(void) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (L.clickedActions.empty()) return 0;
    uint32_t id = L.clickedActions.front();
    L.clickedActions.erase(L.clickedActions.begin());
    return id;
}
FPV_API const fpv_draw_data *fpv_overlay_render(float dt) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return nullptr;
    L.runner.pump();  // browsers: the radio wizard reads live sticks with no drone
    return overlay().render(dt);
}

}  // extern "C"
