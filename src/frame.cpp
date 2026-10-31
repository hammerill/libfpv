// libfpv - fpv_drone_tick: interpolated pose, cameras and everything else
// the host needs for one rendered frame.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/bf_bridge.h"
#include "lib.h"
#include "util/log.h"

namespace fpv {

void updatePresentation(const fpv_frame &f);  // api_present.cpp

static_assert(FPV_CHANNEL_COUNT == kChannelCount, "fpv_frame.channels matches the input channels");

#if defined(LIBFPV_KAMIKAZE)
// After the explosion the camera and the transmitter are gone: the goggles
// show only static, with no OSD, until repaired. The warhead state before
// that is Betaflight's custom message 1 element (Flight::step).
static void clearOsd(uint8_t *osd) { std::memset(osd, ' ', FPV_OSD_ROWS * FPV_OSD_COLS); }
#endif

static void put3(float out[3], Vec3 v) {
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
}
static void put4(float out[4], Quat q) {
    out[0] = q.x;
    out[1] = q.y;
    out[2] = q.z;
    out[3] = q.w;
}

// Builds a host-convention camera from internal position and look vectors.
static void makeCamera(fpv_camera &c, Vec3 pos, Vec3 fwd, Vec3 up, float fovY, float aspect, float nearM) {
    const CoordConverter &cv = lib().conv;
    fwd = normalize(fwd, {0, 0, 1});
    Quat qi = lookRotation(fwd, up);
    Vec3 u = rotate(qi, {0, 1, 0});
    Vec3 r = rotate(qi, {1, 0, 0});
    put3(c.position, cv.posToHost(pos));
    put4(c.rotation, cv.rotToHost(qi));
    put3(c.forward, cv.dirToHost(fwd));
    put3(c.up, cv.dirToHost(u));
    put3(c.right, cv.dirToHost(r));
    c.fov_y_degrees = fovY;
    c.aspect_hint = aspect;
    c.near_plane = cv.lenToHost(nearM);
    c.distortion_k1 = c.distortion_k2 = 0;
    c.distortion_zoom = 1;
}

// Host raycast callback -> temporary collision planes around the drone.
static void refreshRaycastProxies(const FlightSnapshot &s) {
    Lib &L = lib();
    if (!L.raycast || !s.active) return;
    const CoordConverter &cv = L.conv;
    float reach = std::max(2.0f, length(s.vel) * 0.3f);
    static const Vec3 dirs[] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},   {0, -1, 0},  {0, 0, 1},  {0, 0, -1},
                                {1, 1, 1},  {1, 1, -1}, {1, -1, 1},  {1, -1, -1}, {-1, 1, 1}, {-1, 1, -1},
                                {-1, -1, 1}, {-1, -1, -1}};
    std::vector<Contact> planes;
    auto cast = [&](Vec3 d) {
        d = normalize(d);
        float o[3], dh[3];
        put3(o, cv.posToHost(s.pos));
        put3(dh, cv.dirToHost(d));
        fpv_raycast_hit hit{};
        if (L.raycast(L.raycastUser, o, dh, cv.lenToHost(reach), &hit)) {
            float t = cv.lenToInternal(hit.distance);
            if (!(t >= 0) || t > reach) return;
            Contact c;
            c.point = s.pos + d * t;
            c.normal = normalize(cv.dirToInternal({hit.normal[0], hit.normal[1], hit.normal[2]}), -d);
            if (dot(c.normal, d) > 0) c.normal = -c.normal;
            c.material = hit.material;
            c.objectId = hit.object_id;
            planes.push_back(c);
        }
    };
    for (Vec3 d : dirs) cast(d);
    if (length2(s.vel) > 0.25f) cast(s.vel);
    L.world.setProxyPlanes(std::move(planes));
}

// The game's wind made local around the drone (probes into the world) for
// the next simulation steps.
static void refreshWind(const FlightSnapshot &s, float frameDt) {
    Lib &L = lib();
    if (!L.settings.windEnabled || !s.active) {
        L.windProbe.reset();
        L.flight.setWind({});
        return;
    }
    const CoordConverter &cv = L.conv;
    // the drone's own collision world (static surfaces, objects) and the
    // game's raycast, whichever is nearer
    auto ray = [&](Vec3 o, Vec3 d, float maxDist, float &t, Vec3 &n) {
        bool any = false;
        {
            std::lock_guard<std::mutex> lock(L.world.mutex());
            uint8_t mat;
            uint32_t obj;
            float wt;
            Vec3 wn;
            if (L.world.raycast(o, d, maxDist, s.simTime, wt, wn, mat, obj)) {
                any = true;
                t = wt;
                n = wn;
                maxDist = wt;
            }
        }
        if (L.raycast) {
            float oh[3], dh[3];
            put3(oh, cv.posToHost(o));
            put3(dh, cv.dirToHost(d));
            fpv_raycast_hit hit{};
            if (L.raycast(L.raycastUser, oh, dh, cv.lenToHost(maxDist), &hit)) {
                float ht = cv.lenToInternal(hit.distance);
                if (ht >= 0 && ht <= maxDist) {
                    any = true;
                    t = ht;
                    n = normalize(cv.dirToInternal({hit.normal[0], hit.normal[1], hit.normal[2]}), -d);
                }
            }
        }
        return any;
    };
    WindAmbient ambient = L.windSet ? L.windAmbient : WindAmbient{};
    L.flight.setWind(L.windProbe.update(frameDt, s.pos, ambient, L.settings.windStrength, L.windSources, ray));
}

}  // namespace fpv

using namespace fpv;

extern "C" FPV_API fpv_result fpv_drone_tick(fpv_drone *d, float frameDt, fpv_frame *frameOut) {
    Lib &L = lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (!L.initialized) return setError(FPV_ERROR_NOT_INITIALIZED, "call fpv_global_init first");
    if (!d || d != &L.drone || !L.droneExists) return setError(FPV_ERROR_INVALID_ARGUMENT, "invalid or deleted drone handle");
    if (!frameOut || frameOut->struct_size < offsetof(fpv_frame, channels))
        return setError(FPV_ERROR_ABI_MISMATCH, "set frame->struct_size = sizeof(fpv_frame)");
    if (!std::isfinite(frameDt) || frameDt < 0) frameDt = 0;
    frameDt = std::min(frameDt, 0.25f);
    L.runner.pump();  // browsers; before the heartbeat, so a long gap auto-pauses
    L.runner.heartbeat();

    const CoordConverter &cv = L.conv;
    FlightSnapshot prev, cur;
    double prevWall, curWall;
    L.runner.snapshots(prev, prevWall, cur, curWall);

    // Render one simulation step in the past, interpolating between the two
    // latest steps (frame-rate independent, no extrapolation jitter).
    float alpha = 1.0f;
    if (L.runner.threaded() && curWall > prevWall && prev.active) {
        double renderTime = wallSeconds() - L.runner.stepSeconds();
        alpha = (float)std::clamp((renderTime - prevWall) / (curWall - prevWall), 0.0, 1.0);
    }
    Vec3 pos = prev.active ? lerp(prev.pos, cur.pos, alpha) : cur.pos;
    Quat rot = prev.active ? slerp(prev.rot, cur.rot, alpha) : cur.rot;
    Vec3 vel = prev.active ? lerp(prev.vel, cur.vel, alpha) : cur.vel;

    refreshRaycastProxies(cur);
    refreshWind(cur, frameDt);

    fpv_frame f;
    memset(&f, 0, sizeof(f));
    f.struct_size = sizeof(f);
    f.sim_time = cur.simTime;
    uint32_t flags = 0;
    if (cur.armed) flags |= FPV_DRONE_ARMED;
    if (cur.failsafe) flags |= FPV_DRONE_FAILSAFE;
    if (cur.crashed) flags |= FPV_DRONE_CRASHED;
    if (cur.beeper) flags |= FPV_DRONE_BEEPER;
    if (L.runner.paused() || L.runner.autoPaused()) flags |= FPV_DRONE_PAUSED;
    if (cur.onGround) flags |= FPV_DRONE_ON_GROUND;
    if (cur.modeAngle) flags |= FPV_DRONE_MODE_ANGLE;
    if (cur.modeHorizon) flags |= FPV_DRONE_MODE_HORIZON;
    if (cur.turtle) flags |= FPV_DRONE_TURTLE;
    if (cur.osdDisabled) flags |= FPV_DRONE_OSD_DISABLED;
#if defined(LIBFPV_KAMIKAZE)
    if (cur.active && cur.warhead.state == WarheadState::Detonated) flags |= FPV_DRONE_DETONATED;
#endif
    f.flags = flags;
    f.arming_disable_flags = cur.armingDisableFlags;
    std::string why;
#if defined(LIBFPV_KAMIKAZE)
    if (cur.active && cur.warhead.blocksArming) why = "BOMB";
#endif
    for (int i = 0; i < 32; i++) {
        if (cur.armingDisableFlags & (1u << i)) {
            if (!why.empty()) why += ' ';
            why += bfb_arming_disable_flag_name(i);
        }
    }
    snprintf(f.arming_disable_text, sizeof(f.arming_disable_text), "%s", why.c_str());

    put3(f.position, cv.posToHost(pos));
    put4(f.rotation, cv.rotToHost(rot));
    put3(f.velocity, cv.dirToHost(vel) * cv.unitsPerMeter());
    put3(f.angular_velocity, cv.angVelToHost(cur.angVel));
    f.speed_kmh = length(vel) * 3.6f;
    f.altitude = cur.altitude >= 0 ? cv.lenToHost(cur.altitude) : -1.0f;

    // --- cameras ---------------------------------------------------------------
    const DronePreset &p = currentPreset();
    float tilt = (L.settings.cameraUptilt >= 0 ? L.settings.cameraUptilt : p.camera.uptiltDeg) * kPi / 180.0f;
    float fov = L.settings.cameraFov > 0 ? L.settings.cameraFov : p.camera.fovYDeg;
    Vec3 camPos = pos + rotate(rot, p.camera.offset);
    Vec3 camFwd = rotate(rot, {0, std::sin(tilt), std::cos(tilt)});
    Vec3 camUp = rotate(rot, {0, std::cos(tilt), -std::sin(tilt)});
    float k1 = p.camera.k1, k2 = p.camera.k2, zoom = p.camera.zoom;
    bool lens = !(L.config.flags & FPV_INIT_NO_LENS_DISTORTION);
    if (!lens) k1 = k2 = 0, zoom = 1;
    if (lens && L.settings.cameraFisheye > 0) {
        // Barrel lens filling the whole frame: the corners (r = 1) sample
        // the corners of the rendered image, so there are no black borders.
        float s = std::min(L.settings.cameraFisheye, 1.0f);
        k1 = 0.55f * s;
        k2 = 0.25f * s;
        zoom = 1 + k1 + k2;
        // The post-process shrinks the centre, so render wider: the top and
        // bottom edges of the distorted image still show the chosen FOV.
        float a = p.camera.aspect;
        float rv2 = 1 / (1 + a * a);  // squared radius of the top edge centre
        float edge = (1 + k1 * rv2 + k2 * rv2 * rv2) / zoom;
        float t = std::tan(fov * kPi / 360.0f) / edge;
        fov = std::min(std::atan(t) * 360.0f / kPi, 170.0f);
    }
    makeCamera(f.camera, camPos, camFwd, camUp, fov, p.camera.aspect, 0.01f);
    f.camera.distortion_k1 = k1;
    f.camera.distortion_k2 = k2;
    f.camera.distortion_zoom = zoom;

    // Chase: behind along the heading, smoothed, pulled in front of walls.
    Vec3 heading = rotate(rot, {0, 0, 1});
    heading.y = 0;
    if (length2(heading) < 1e-4f) heading = rotate(rot, {0, 1, 0}) * -1.0f, heading.y = 0;
    heading = normalize(heading, {0, 0, 1});
    Vec3 desired = pos - heading * 1.3f + Vec3{0, 0.45f, 0};
    if (!L.chaseValid) {
        L.chasePos = desired;
        L.chaseValid = true;
    }
    float k = L.settings.chaseSmoothing ? 1.0f - std::exp(-frameDt * 6.0f) : 1.0f;
    L.chasePos = lerp(L.chasePos, desired, k);
    Vec3 chase = L.chasePos;
    {
        std::lock_guard<std::mutex> wl(L.world.mutex());
        Vec3 dir = chase - pos;
        float dist = length(dir);
        float t;
        Vec3 n;
        uint8_t m;
        uint32_t o;
        if (dist > 1e-3f && L.world.raycast(pos, dir / dist, dist, cur.simTime, t, n, m, o)) chase = pos + dir / dist * std::max(0.05f, t - 0.1f);
    }
    makeCamera(f.chase_camera, chase, pos - chase, {0, 1, 0}, 75.0f, 16.0f / 9.0f, 0.05f);
    Vec3 eye = L.flight.pilotPosition() + Vec3{0, 1.6f, 0};
    makeCamera(f.los_camera, eye, pos - eye, {0, 1, 0}, 50.0f, 16.0f / 9.0f, 0.1f);

    // --- motors, battery, video --------------------------------------------------
    // Spin directions follow Betaflight's default "props in" Quad X layout.
    static const float kSpin[4] = {+1.0f, -1.0f, -1.0f, +1.0f};
    for (int i = 0; i < 4; i++) {
        const MotorSnapshot &m = cur.motors[i];
        fpv_motor_state &o = f.motors[i];
        o.rpm = m.rpm;
        o.thrust_newton = m.thrust;
        o.current_amp = m.current;
        o.temperature_c = m.temperature;
        o.command = m.command;
        o.prop_damage = m.propDamage;
        const Vec3F &lp = p.init.quadMotorPos[i];
        put3(o.position, cv.dirToHost({lp.x, lp.y, lp.z}) * cv.unitsPerMeter());
        o.spin_direction = kSpin[i];
        o.flags = m.flags;
    }
    f.battery.voltage = cur.batVoltageSag;
    f.battery.cell_count = p.init.quadBatCellCount;
    f.battery.cell_voltage = cur.batVoltageSag / std::max<uint32_t>(1, f.battery.cell_count);
    f.battery.current_amp = cur.batCurrent;
    f.battery.mah_drawn = cur.mahDrawn;
    f.battery.capacity_mah = p.init.quadBatCapacity;
    f.video.noise = cur.videoNoise;
    f.video.breakup = cur.videoBreakup;
    f.video.signal_strength = cur.videoSignal;
    f.video.is_digital = p.vtx.digital ? 1 : 0;

    // --- impacts -------------------------------------------------------------------
    L.impactScratch.clear();
    L.runner.takeImpacts(L.impactScratch);
    for (const ImpactEvent &e : L.impactScratch) {
        if (f.impact_count >= FPV_MAX_IMPACTS) break;
        fpv_impact &o = f.impacts[f.impact_count++];
        put3(o.position, cv.posToHost(e.point));
        put3(o.normal, cv.dirToHost(e.normal));
        o.speed = cv.lenToHost(e.speed);
        o.impulse = e.impulse;
        o.material = e.material;
        o.object_id = e.objectId;
        o.motor_index = e.motorIndex;
    }

    memcpy(f.osd, cur.osd.data(), sizeof(f.osd));
    f.osd_revision = cur.osdRevision;
#if defined(LIBFPV_KAMIKAZE)
    // no flight controller OSD over the lost feed, whatever the OSD disable switch
    if (cur.active && L.settings.kamikazeOsd && cur.warhead.state == WarheadState::Detonated) clearOsd(f.osd);
    // Betaflight's revision cannot see the cleared grid: count changes here
    if (memcmp(L.kamikazeOsd, f.osd, sizeof(f.osd)) != 0) {
        memcpy(L.kamikazeOsd, f.osd, sizeof(f.osd));
        L.kamikazeOsdRevision++;
    }
    f.osd_revision = L.kamikazeOsdRevision;
#endif
    for (int i = 0; i < kChannelCount; i++) f.channels[i] = cur.channels[i];
    put3(f.wind, cv.dirToHost(cur.wind) * cv.unitsPerMeter());

    updatePresentation(f);
    L.lastFrame = f;
    L.lastFrameValid = true;
    memcpy(frameOut, &f, std::min<size_t>(frameOut->struct_size, sizeof(f)));
    frameOut->struct_size = std::min<uint32_t>(frameOut->struct_size, sizeof(f));
    return FPV_OK;
}
