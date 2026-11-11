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

void updatePresentation(const fpv_frame &f, Vec3 dronePos, Vec3 droneVel, bool beeperSwitch);  // api_present.cpp

static_assert(FPV_CHANNEL_COUNT == kChannelCount, "fpv_frame.channels matches the input channels");

constexpr float kWallThickness = 0.5f;   // m: hits closer than this after one are the same wall
constexpr float kEchoRange = 40.0f;      // m: surfaces farther away make no echo worth hearing
constexpr float kRadioJitter = 0.05f;    // of the -1..1 range: a pot's or switch's noise
constexpr double kRadioActiveSeconds = 3.0;

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
    // One ray of the host's; a hit inside a portal's opening goes on from
    // the linked one, and what it finds there is brought back to this side
    // (where it seems to be, seen through the opening).
    auto cast = [&](Vec3 from, float range, Vec3 d) {
        d = normalize(d);
        Vec3 o = from;
        float left = range, travelled = 0;
        PortalTransform back{Quat{}, {}, {}};  // from the far side to the drone's
        bool through = false;
        for (int hop = 0; hop < 3 && left > 0; hop++) {
            float oh[3], dh[3];
            put3(oh, cv.posToHost(o));
            put3(dh, cv.dirToHost(d));
            fpv_raycast_hit hit{};
            if (!L.raycast(L.raycastUser, oh, dh, cv.lenToHost(left), &hit)) return;
            float t = cv.lenToInternal(hit.distance);
            if (!(t >= 0) || t > left) return;
            Vec3 at = o + d * t;
            int hole = L.portals.empty() ? -1 : portalHoleAt(L.portals, at);
            if (hole >= 0) {
                const PortalLink &link = L.portals[hole / 2];
                const Portal &in = link.side(hole & 1), &out = link.side(1 - (hole & 1));
                // only entering from the front of the opening
                if (dot(d, rotate(in.rot, Vec3{0, 0, 1})) < 0) {
                    PortalTransform x = portalTransform(in, out), inv = portalTransform(out, in);
                    // through the surface point on the ray, then on from just past the far opening
                    Vec3 l = in.toLocal(o);
                    Vec3 dl = rotate(conj(in.rot), d);
                    float tc = dl.z < -1e-6f ? -l.z / dl.z : t;  // where the ray meets the surface
                    tc = std::clamp(tc, 0.0f, left);
                    Vec3 cross = o + d * tc;
                    back = through ? PortalTransform{normalize(back.rot * inv.rot), inv.fromPos, back.point(inv.toPos)}
                                   : inv;
                    through = true;
                    o = x.point(cross) + x.dir(d) * 0.002f;
                    d = x.dir(d);
                    travelled += tc;
                    left -= tc;
                    continue;
                }
            }
            Contact c;
            Vec3 n = normalize(cv.dirToInternal({hit.normal[0], hit.normal[1], hit.normal[2]}), -d);
            if (dot(n, d) > 0) n = -n;
            c.point = through ? back.point(at) : at;
            c.normal = through ? back.dir(n) : n;
            c.material = hit.material;
            c.objectId = hit.object_id;
            c.throughPortal = through;
            planes.push_back(c);
            (void)travelled;
            return;
        }
    };
    for (Vec3 d : dirs) cast(s.pos, reach, d);
    if (length2(s.vel) > 0.25f) cast(s.pos, reach, s.vel);
    if (s.cargoHanging) {
        // the cargo on its line collides on its own, maybe metres away
        float cargoReach = std::max(1.5f, length(s.cargoVel) * 0.3f);
        for (Vec3 d : dirs) cast(s.cargoPos, cargoReach, d);
        if (length2(s.cargoVel) > 0.25f) cast(s.cargoPos, cargoReach, s.cargoVel);
    }
    L.world.setProxyPlanes(std::move(planes));
}

// A ray against the drone's own collision world (static surfaces, objects)
// and the game's raycast, whichever is nearer. Host thread.
static bool worldRay(double simTime, Vec3 o, Vec3 d, float maxDist, float &t, Vec3 &n) {
    Lib &L = lib();
    const CoordConverter &cv = L.conv;
    bool any = false;
    {
        std::lock_guard<std::mutex> lock(L.world.mutex());
        uint8_t mat;
        uint32_t obj;
        float wt;
        Vec3 wn;
        if (L.world.raycast(o, d, maxDist, simTime, wt, wn, mat, obj)) {
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
    auto ray = [&](Vec3 o, Vec3 d, float maxDist, float &t, Vec3 &n) { return worldRay(s.simTime, o, d, maxDist, t, n); };
    WindAmbient ambient = L.windSet ? L.windAmbient : WindAmbient{};
    L.flight.setWind(L.windProbe.update(frameDt, s.pos, ambient, L.settings.windStrength, L.windSources, ray));
}

// Where the ear is when the drone is heard from the pilot: the game's
// listener, or the pilot's head.
Vec3 earPosition() {
    Lib &L = lib();
    return L.listenerSet ? L.listenerPos : L.flight.pilotPosition() + Vec3{0, kPilotEarHeight, 0};
}

// For the sound heard from the pilot (or onboard, for the room around the
// drone): the walls between the ear and the drone, and the surfaces around
// the ear that echo. 10 times a second, smoothed.
static void refreshHearing(const FlightSnapshot &s, Vec3 dronePos, bool fromPilot, float frameDt) {
    Lib &L = lib();
    Lib::HearingProbe &h = L.hearingProbe;
    if (!s.active || !audioPulled()) {  // no sound pulled yet: no rays
        h.valid = false;
        return;
    }
    h.timer -= frameDt;
    if (h.valid && h.timer > 0) return;
    h.timer = 0.1f;
    float t;
    Vec3 n;
    // walls: hits along the line, both faces of a thin wall counted once
    Vec3 ear = earPosition();
    Vec3 d = dronePos - ear;
    float dist = length(d);
    int walls = 0;
    if (dist > 0.5f) {
        Vec3 dir = d / dist;
        float from = 0;
        while (walls < 4) {
            float left = dist - 0.3f - from;
            if (left <= 0 || !worldRay(s.simTime, ear + dir * from, dir, left, t, n)) break;
            walls++;
            from += t + kWallThickness;
        }
    }
    // the surroundings of whichever ear is heard: six directions around and up
    Vec3 at = fromPilot ? ear : dronePos;
    int around = 0;
    float hitSum = 0, allSum = 0;
    for (int i = 0; i < 6; i++) {
        float a = i * kPi / 3;
        Vec3 dir{std::sin(a), 0, std::cos(a)};
        if (worldRay(s.simTime, at, dir, kEchoRange, t, n)) {
            around++;
            hitSum += t;
            allSum += t;
        } else {
            allSum += kEchoRange;
        }
    }
    bool roof = worldRay(s.simTime, at, {0, 1, 0}, kEchoRange, t, n);
    allSum += roof ? t : kEchoRange;
    float reflectors = around / 6.0f;
    float enclosure = roof ? 0.4f + 0.6f * reflectors : 0.3f * reflectors;
    float k = h.valid ? 0.35f : 1.0f;
    h.walls += (walls - h.walls) * k;
    h.enclosure += (enclosure - h.enclosure) * k;
    h.reflectors += (reflectors - h.reflectors) * k;
    if (around) h.echoDistance += (hitSum / around - h.echoDistance) * (h.echoDistance > 0 ? k : 1.0f);
    h.roomSize += (allSum / 7 - h.roomSize) * k;
    h.valid = true;
}

// FPV_DRONE_RADIO_ACTIVE: armed, or a channel moved past the jitter lately.
static bool radioActive(const FlightSnapshot &s) {
    Lib &L = lib();
    double now = wallSeconds();
    for (int i = 0; i < kChannelCount; i++) {
        if (std::fabs(s.channels[i] - L.radioRest[i]) > kRadioJitter) {
            std::copy(s.channels.begin(), s.channels.end(), L.radioRest);
            L.radioMovedAt = now;
            break;
        }
    }
    return s.armed || now - L.radioMovedAt < kRadioActiveSeconds;
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
    // a jump (portal, teleport) between the two: show where it is now
    bool jumped = prev.teleports != cur.teleports;
    if (cur.teleports != L.seenTeleports) {
        L.seenTeleports = cur.teleports;
        L.chaseValid = false;
        L.windProbe.reset();
    }
    if (jumped) prev = cur;
    if (L.runner.threaded() && curWall > prevWall && prev.active && !jumped) {
        double renderTime = wallSeconds() - L.runner.stepSeconds();
        alpha = (float)std::clamp((renderTime - prevWall) / (curWall - prevWall), 0.0, 1.0);
    }
    Vec3 pos = prev.active ? lerp(prev.pos, cur.pos, alpha) : cur.pos;
    Quat rot = prev.active ? slerp(prev.rot, cur.rot, alpha) : cur.rot;
    Vec3 vel = prev.active ? lerp(prev.vel, cur.vel, alpha) : cur.vel;

    refreshRaycastProxies(cur);
    refreshWind(cur, frameDt);
    refreshHearing(cur, pos, hearsFromPilot(), frameDt);

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
    if (cur.active && radioActive(cur)) flags |= FPV_DRONE_RADIO_ACTIVE;
    if (hearsFromPilot()) flags |= FPV_DRONE_HEARD_FROM_PILOT;
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
    // The lens sits ahead of the centre: past an opening the centre has not
    // crossed yet (or still behind one it came out of, flying backwards),
    // it already sees the far side.
    for (size_t i = 0; i < L.portals.size(); i++) {
        bool moved = false;
        for (int sd = 0; sd < 2 && !moved; sd++) {
            const Portal &o = L.portals[i].side(sd);
            Vec3 lc = o.toLocal(camPos), lb = o.toLocal(pos);
            if (lb.z < 0 || lc.z >= 0 || lc.z < -kPortalHoleDepth || !o.insideOpening(lc)) continue;
            PortalTransform x = portalTransform(o, L.portals[i].side(1 - sd));
            camPos = x.point(camPos);
            camFwd = x.dir(camFwd);
            camUp = x.dir(camUp);
            moved = true;
        }
        if (moved) break;
    }
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
    Vec3 eye = L.flight.pilotPosition() + Vec3{0, kPilotEarHeight, 0};
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
    f.portal_passes = cur.portalPasses;
    f.last_portal = cur.lastPortal;
    if (cur.cargo) {
        f.cargo_flags = FPV_CARGO_CARRIED;
        if (cur.cargoHanging) f.cargo_flags |= FPV_CARGO_ON_LINE;
        if (cur.cargoLineTaut) f.cargo_flags |= FPV_CARGO_LINE_TAUT;
        bool both = prev.active && prev.cargo && prev.cargoHanging == cur.cargoHanging;
        put3(f.cargo_position, cv.posToHost(both ? lerp(prev.cargoPos, cur.cargoPos, alpha) : cur.cargoPos));
        put3(f.cargo_velocity, cv.dirToHost(both ? lerp(prev.cargoVel, cur.cargoVel, alpha) : cur.cargoVel) *
                                   cv.unitsPerMeter());
    }

    updatePresentation(f, pos, vel, cur.beeperSwitch);
    L.lastFrame = f;
    L.lastFrameValid = true;
    memcpy(frameOut, &f, std::min<size_t>(frameOut->struct_size, sizeof(f)));
    frameOut->struct_size = std::min<uint32_t>(frameOut->struct_size, sizeof(f));
    return FPV_OK;
}
