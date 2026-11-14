// libfpv - C API of kamikaze builds (LIBFPV_KAMIKAZE): warhead state,
// detonation events, host detonation and configuration.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstring>

#include "lib.h"

using namespace fpv;

namespace {

void put3(float out[3], Vec3 v) {
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
}

}  // namespace

namespace fpv {

void applyKamikazeSettings() {
    Lib &L = lib();
    const Settings &s = L.settings;
    WarheadConfig w;
    w.armingDelay = s.kamikazeArmingDelay;
    w.minImpactSpeed = s.kamikazeMinImpactSpeed;
    w.ignitionDelay = s.kamikazeIgnitionDelay;
    w.osd = s.kamikazeOsd;
    L.flight.warhead().setConfig(w);
    float kg = s.kamikazePayloadKg, drop = s.kamikazePayloadDrop;
    bool installed = kamikazeInstalledOnCurrent();
    L.runner.call([&] {
        L.flight.setBombInstalled(installed);
        L.flight.setPayload(kg, drop);
    });
}

bool kamikazeInstalledOnCurrent() {
    Lib &L = lib();
    if (L.presets.empty()) return false;
    const auto &on = L.settings.kamikazeInstalled;
    return std::find(on.begin(), on.end(), currentPreset().fileName) != on.end();
}

uint32_t kamikazeEndingView() {
    const Lib &L = lib();
    int v = L.settings.kamikazeEndingView;
    return v >= 0 && (L.kamikazeViewMask & (1u << v)) ? (uint32_t)v : L.kamikazeDefaultView;
}

uint32_t kamikazeEndingThen() {
    const Lib &L = lib();
    int t = L.settings.kamikazeEndingThen;
    return t >= 0 && (L.kamikazeThenMask & (1u << t)) ? (uint32_t)t : L.kamikazeDefaultThen;
}

}  // namespace fpv

extern "C" {

FPV_API fpv_result fpv_kamikaze_get_state(fpv_drone *d, fpv_kamikaze_state *state) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!d || d != &L.drone || !L.droneExists) return setError(FPV_ERROR_INVALID_ARGUMENT, "invalid or deleted drone handle");
    if (!state || state->struct_size < offsetof(fpv_kamikaze_state, flags))
        return setError(FPV_ERROR_ABI_MISMATCH, "set state->struct_size = sizeof(fpv_kamikaze_state)");
    FlightSnapshot prev, cur;
    double pw, cw;
    L.runner.snapshots(prev, pw, cur, cw);
    const WarheadSnapshot &w = cur.warhead;
    fpv_kamikaze_state out{};
    out.struct_size = sizeof(out);
    out.warhead = (fpv_warhead_state)w.state;
    out.arming_remaining = w.armingRemaining;
    if (w.installed) out.flags |= FPV_KAMIKAZE_INSTALLED;
    if (w.bombSwitch) out.flags |= FPV_KAMIKAZE_BOMB_SWITCH;
    if (w.explodeSwitch) out.flags |= FPV_KAMIKAZE_EXPLODE_SWITCH;
    if (w.blocksArming) out.flags |= FPV_KAMIKAZE_BLOCKS_ARMING;
    if (w.needsCycle && w.bombSwitch) out.flags |= FPV_KAMIKAZE_NEEDS_SWITCH_CYCLE;
    out.ignition_remaining = w.ignitionRemaining;
    out.detonation_count = w.detonations;
    out.payload_mass = cur.payloadMass;
    out.total_mass = cur.totalMass;
    uint32_t size = std::min<uint32_t>(state->struct_size, sizeof(out));
    memcpy(state, &out, size);
    state->struct_size = size;
    return FPV_OK;
}

FPV_API int fpv_kamikaze_poll_detonation(fpv_detonation *out) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!out || out->struct_size < offsetof(fpv_detonation, material)) return 0;
    DetonationEvent e;
    if (!L.flight.warhead().pollDetonation(e)) return 0;
    const CoordConverter &cv = L.conv;
    fpv_detonation d{};
    d.struct_size = sizeof(d);
    d.sim_time = e.simTime;
    d.cause = (fpv_detonation_cause)e.cause;
    put3(d.position, cv.posToHost(e.position));
    put3(d.velocity, cv.dirToHost(e.velocity) * cv.unitsPerMeter());
    if (length2(e.normal) > 0) put3(d.normal, cv.dirToHost(e.normal));
    d.material = e.material;
    d.object_id = e.objectId;
    d.motor_index = e.motorIndex;
    uint32_t size = std::min<uint32_t>(out->struct_size, sizeof(d));
    memcpy(out, &d, size);
    out->struct_size = size;
    return 1;
}

FPV_API fpv_result fpv_kamikaze_set_installed(int installed) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if ((installed != 0) != kamikazeInstalledOnCurrent()) {
        auto &on = L.settings.kamikazeInstalled;
        const std::string &quad = currentPreset().fileName;
        if (installed) on.push_back(quad);
        else on.erase(std::remove(on.begin(), on.end(), quad), on.end());
        saveSettings();
    }
    applyKamikazeSettings();
    return FPV_OK;
}

FPV_API int fpv_kamikaze_is_installed(void) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    return L.initialized && kamikazeInstalledOnCurrent() ? 1 : 0;
}

FPV_API fpv_result fpv_kamikaze_detonate(fpv_drone *d, int evenIfSafe) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!d || d != &L.drone || !L.droneExists) return setError(FPV_ERROR_INVALID_ARGUMENT, "invalid or deleted drone handle");
    bool done = L.runner.call([&] { return L.flight.detonate(evenIfSafe != 0); });
    return done ? FPV_OK : setError(FPV_ERROR_NOT_FOUND, "the warhead is not live (or already exploded, or not installed)");
}

FPV_API void fpv_kamikaze_config_defaults(fpv_kamikaze_config *c) {
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->struct_size = sizeof(*c);
    Settings def;
    c->arming_delay = def.kamikazeArmingDelay;
    Lib &L = lib();
    c->min_impact_speed = L.initialized ? L.conv.lenToHost(def.kamikazeMinImpactSpeed) : def.kamikazeMinImpactSpeed;
    c->payload_mass = def.kamikazePayloadKg;
    c->payload_drop = def.kamikazePayloadDrop;
    c->ending_duration = def.kamikazeEndingDuration;
    c->ending_view = L.kamikazeDefaultView;
    c->ending_then = L.kamikazeDefaultThen;
    c->ignition_delay = def.kamikazeIgnitionDelay;
}

FPV_API fpv_result fpv_kamikaze_set_config(const fpv_kamikaze_config *in) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!in) return setError(FPV_ERROR_INVALID_ARGUMENT, "config is NULL");
    fpv_kamikaze_config c;
    fpv_kamikaze_config_defaults(&c);
    size_t size = std::min<size_t>(sizeof(c), in->struct_size ? in->struct_size : sizeof(c));
    memcpy(&c, in, size);
    Settings &s = L.settings;
    // older hosts do not know it: keep the player's
    if (size < offsetof(fpv_kamikaze_config, ending_duration) + sizeof(c.ending_duration))
        c.ending_duration = s.kamikazeEndingDuration;
    bool hasView = size >= offsetof(fpv_kamikaze_config, ending_view) + sizeof(c.ending_view);
    bool hasThen = size >= offsetof(fpv_kamikaze_config, ending_then) + sizeof(c.ending_then);
    if (size < offsetof(fpv_kamikaze_config, ignition_delay) + sizeof(c.ignition_delay))
        c.ignition_delay = s.kamikazeIgnitionDelay;
    if (!std::isfinite(c.arming_delay) || c.arming_delay < 0 || c.arming_delay > 600)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "arming_delay must be 0..600 s");
    float fuse = L.conv.lenToInternal(c.min_impact_speed);
    if (!std::isfinite(fuse) || fuse < 0.1f - 1e-4f || fuse > 30.0f)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "min_impact_speed must be 0.1..30 m/s (in host units)");
    if (!std::isfinite(c.payload_mass) || c.payload_mass > 50)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "payload_mass must be up to 50 kg (negative = automatic)");
    if (!std::isfinite(c.payload_drop)) return setError(FPV_ERROR_INVALID_ARGUMENT, "payload_drop is not finite");
    if (!std::isfinite(c.ending_duration) || c.ending_duration < 0 || c.ending_duration > 30)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "ending_duration must be 0..30 s");
    if (!std::isfinite(c.ignition_delay) || c.ignition_delay < 0 || c.ignition_delay > 1)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "ignition_delay must be 0..1 s");
    if (hasView && c.ending_view > FPV_KAMIKAZE_VIEW_CUTSCENE)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "ending_view is not a fpv_kamikaze_ending_view");
    if (hasThen && c.ending_then > FPV_KAMIKAZE_THEN_NOTHING)
        return setError(FPV_ERROR_INVALID_ARGUMENT, "ending_then is not a fpv_kamikaze_ending_then");
    s.kamikazeArmingDelay = c.arming_delay;
    s.kamikazeMinImpactSpeed = std::max(0.1f, fuse);
    s.kamikazePayloadKg = c.payload_mass < 0 ? -1.0f : c.payload_mass;
    s.kamikazePayloadDrop = c.payload_drop < 0 ? -1.0f : std::min(L.conv.lenToInternal(c.payload_drop), 1.0f);
    s.kamikazeEndingDuration = c.ending_duration;
    s.kamikazeIgnitionDelay = c.ignition_delay;
    // a choice the host does not support reads back as its default
    if (hasView) s.kamikazeEndingView = (int)c.ending_view;
    if (hasThen) s.kamikazeEndingThen = (int)c.ending_then;
    s.kamikazeOsd = !(c.flags & FPV_KAMIKAZE_NO_OSD);
    applyKamikazeSettings();
    saveSettings();
    return FPV_OK;
}

FPV_API fpv_result fpv_kamikaze_get_config(fpv_kamikaze_config *out) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!out || out->struct_size < offsetof(fpv_kamikaze_config, flags))
        return setError(FPV_ERROR_ABI_MISMATCH, "set config->struct_size = sizeof(fpv_kamikaze_config)");
    const Settings &s = L.settings;
    fpv_kamikaze_config c;
    fpv_kamikaze_config_defaults(&c);
    c.arming_delay = s.kamikazeArmingDelay;
    c.min_impact_speed = L.conv.lenToHost(s.kamikazeMinImpactSpeed);
    c.flags = s.kamikazeOsd ? 0 : FPV_KAMIKAZE_NO_OSD;
    c.payload_mass = s.kamikazePayloadKg;
    c.payload_drop = s.kamikazePayloadDrop < 0 ? -1.0f : L.conv.lenToHost(s.kamikazePayloadDrop);
    c.ending_duration = s.kamikazeEndingDuration;
    c.ending_view = kamikazeEndingView();
    c.ending_then = kamikazeEndingThen();
    c.ignition_delay = s.kamikazeIgnitionDelay;
    uint32_t size = std::min<uint32_t>(out->struct_size, sizeof(c));
    memcpy(out, &c, size);
    out->struct_size = size;
    return FPV_OK;
}

FPV_API fpv_result fpv_kamikaze_set_ending_support(uint32_t viewMask, uint32_t thenMask, uint32_t defaultView,
                                                   uint32_t defaultThen) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    // signal lost and "nothing" need nothing from the game: always possible
    viewMask = (viewMask & 0x3u) | (1u << FPV_KAMIKAZE_VIEW_SIGNAL_LOST);
    thenMask = (thenMask & 0x7u) | (1u << FPV_KAMIKAZE_THEN_NOTHING);
    if (defaultView > FPV_KAMIKAZE_VIEW_CUTSCENE || !(viewMask & (1u << defaultView)))
        return setError(FPV_ERROR_INVALID_ARGUMENT, "default_view must be a supported fpv_kamikaze_ending_view");
    if (defaultThen > FPV_KAMIKAZE_THEN_NOTHING || !(thenMask & (1u << defaultThen)))
        return setError(FPV_ERROR_INVALID_ARGUMENT, "default_then must be a supported fpv_kamikaze_ending_then");
    L.kamikazeViewMask = viewMask;
    L.kamikazeThenMask = thenMask;
    L.kamikazeDefaultView = defaultView;
    L.kamikazeDefaultThen = defaultThen;
    return FPV_OK;
}

}  // extern "C"
