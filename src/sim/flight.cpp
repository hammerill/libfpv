// libfpv - one drone: SimITL flight model + libfpv rigid body and collisions.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "sim/flight.h"

#include <algorithm>
#include <cstring>

#include "core/bf_bridge.h"
#include "sim/bf.h"
#include "sim/sim.h"
#include "simitl.h"
#include "util/log.h"

namespace fpv {

namespace {
constexpr float kImpactReportSpeed = 1.5f;   // m/s, below this a contact is not an "impact"
constexpr float kRestingSpeed = 0.6f;        // m/s, no bounce below this
// A contact is a patch, not a point: its friction also resists turning about
// the normal (torsional friction), up to friction × load × this × the sphere's
// radius. Without it a level quad resting on a flat surface spins freely: the
// body sphere under the centre of mass takes the whole load, the landing
// points none, and friction under the centre of mass makes no yaw torque.
constexpr float kContactPatchRatio = 0.5f;
constexpr float kPropDiscHalfThickness = 0.012f;
constexpr float kPropNutHeight = 0.006f;     // motor top above the prop disc
constexpr float kMaxSpeed = 120.0f;          // m/s, sanity clamp
constexpr float kMaxAngularSpeed = 100.0f;   // rad/s, SimITL's own clamp
#if defined(LIBFPV_KAMIKAZE)
constexpr float kPayloadRadius = 0.035f;     // the warhead's size (its own inertia)
constexpr float kAutoPayloadRatio = 0.6f;    // automatic payload: 60% of the quad's mass
// Contacts slower than this never reach the fuse: resting contact jitter
// stays well under it. The fuse's own threshold is a setting (>= this).
constexpr float kWarheadTouchSpeed = 0.1f;  // m/s
#endif

Mat3 toSimBasis(Quat q) { return toMat3(q); }

void fillRotation(StateInput &in, Quat q) {
    Mat3 m = toSimBasis(q);
    for (int i = 0; i < 3; i++) in.rotation[i] = {m.r[i].x, m.r[i].y, m.r[i].z};
}
}  // namespace

void Flight::initBetaflight(const std::string &eepromPath, const DronePreset &parkedPreset) {
    if (m_bfReady) return;
    m_preset = parkedPreset;
    StateInit init = parkedPreset.init;
    std::memset(init.eepromName, 0, sizeof(init.eepromName));
    std::strncpy((char *)init.eepromName, eepromPath.c_str(), sizeof(init.eepromName) - 1);
    simitl_init(init);
    m_bfReady = true;

    m_input = StateInput{};
    fillRotation(m_input, Quat{});
    for (float &c : m_input.rcData) c = -1.0f;
    m_input.rcData[0] = m_input.rcData[1] = m_input.rcData[3] = 0.0f;
    m_input.contact = 2;
    buildShape();
}

// The props spin the way the flight controller is configured, like a
// correctly built quad: Betaflight's Quad X default is "props in"
// (motor 1 rear right clockwise); yaw_motors_reversed means "props out".
// SimITL's motor direction is +1 clockwise, -1 counter-clockwise.
void Flight::syncPropDirection() {
    int rev = bfb_yaw_motors_reversed();
    if (rev == m_yawReversed) return;
    m_yawReversed = rev;
    static const float kPropsIn[4] = {1, -1, -1, 1};
    StateInit &live = SimITL::Sim::getInstance().mSimState.stateInit;
    for (int i = 0; i < 4; i++) {
        float d = rev ? -kPropsIn[i] : kPropsIn[i];
        m_preset.init.quadMotorDir[i] = d;
        live.quadMotorDir[i] = d;
    }
}

// Collision shape: a sphere at the centre of mass and one under each motor
// bell. Their bottoms are level, so the drone rests on five points and
// ground friction can stop it from spinning. One more sphere on top of each
// motor (bell and prop nut, just above the prop disc): upside down the
// drone rests on those with its props clear of the ground, and in turtle
// mode it pivots on an arm like a real quad instead of rolling on the body.
void Flight::buildShape() {
    m_shape.clear();
    const CollisionSettings &c = m_preset.collision;
    m_shape.push_back({{0, 0, 0}, c.bodyRadius, -1});
    for (int i = 0; i < 4; i++) {
        const Vec3F &p = m_preset.init.quadMotorPos[i];
        float y = -c.bodyRadius + c.motorRadius;
        m_shape.push_back({{p.x, std::min(p.y, y), p.z}, c.motorRadius, i});
        float top = std::max(c.bodyRadius, p.y + kPropDiscHalfThickness + kPropNutHeight);
        m_shape.push_back({{p.x, top - c.motorRadius, p.z}, c.motorRadius, i});
    }
#if defined(LIBFPV_KAMIKAZE)
    // the warhead under the frame: flush with the landing points by
    // default, below them (touching first) when it hangs lower
    if (m_payloadMass > 0) {
        float drop = -m_payloadPos.y;
        float r = drop < c.bodyRadius ? c.bodyRadius - drop : kPayloadRadius;
        m_shape.push_back({m_payloadPos, std::max(0.005f, r), -1});
    }
#endif
    m_mass = std::max(0.01f, m_preset.init.quadMass);
    const Vec3F &ii = m_preset.init.quadInvInertia;
    m_invInertiaBody = {std::fabs(ii.x), std::fabs(ii.y), std::fabs(ii.z)};
}

float Flight::restHeight() const {
    float h = 0;
    for (const Sphere &s : m_shape) h = std::max(h, s.radius - s.local.y);
    return h;
}

// World height of the collision shape's lowest point at the current pose.
float Flight::shapeBottom() const {
    Mat3 rot = toMat3(m_rot);
    float y = m_pos.y;
    for (const Sphere &s : m_shape) y = std::min(y, m_pos.y + (rot * s.local).y - s.radius);
    return y;
}

void Flight::applyPreset(const DronePreset &presetIn) {
#if defined(LIBFPV_KAMIKAZE)
    m_basePreset = presetIn;
    const DronePreset preset = withPayload(presetIn);
#else
    const DronePreset &preset = presetIn;
#endif
    // a drone switched in place keeps its lowest point where it was: a
    // bigger quad would otherwise start inside the ground, and the
    // penetration push would shove it sideways until it fell through
    float bottom = m_shape.empty() ? 0 : shapeBottom();
    m_preset = preset;
    StateInit init = preset.init;
    std::memset(init.eepromName, 0, sizeof(init.eepromName));
    simitl_reinit_physics(init);
    // Betaflight sets its low-voltage thresholds once, when it first sees a
    // battery: without this a 6S quad's thresholds stay in effect on a 4S
    // quad and it warns LAND NOW at full charge
    if (m_bfReady) bfb_battery_reset();
    bool hadShape = !m_shape.empty();
    buildShape();
    if (m_active && hadShape) m_pos.y += bottom - shapeBottom();
    m_yawReversed = -1;
    syncPropDirection();
    for (int i = 0; i < 4; i++) {
        m_input.motorImbalance[i] = {preset.motorImbalance.x, preset.motorImbalance.y, preset.motorImbalance.z};
    }
    m_input.gyroBaseNoiseAmp = preset.gyroBaseNoiseAmp;
    m_input.gyrobaseNoiseFreq = preset.gyrobaseNoiseFreq;
    m_input.frameHarmonic1Amp = preset.frameHarmonic1Amp;
    m_input.frameHarmonic1Freq = preset.frameHarmonic1Freq;
    m_input.frameHarmonic2Amp = preset.frameHarmonic2Amp;
    m_input.frameHarmonic2Freq = preset.frameHarmonic2Freq;
    m_input.vbat = preset.init.quadBatCellCount * 4.2f;
}

// Stops the propellers and clears the ESC models (filters, delay line).
// SimITL's Reset/Repair commands keep them, so a drone deleted while flying
// would otherwise respawn with spinning motors.
void Flight::stopMotors() {
    SimITL::SimState &ss = SimITL::Sim::getInstance().mSimState;
    for (auto &m : ss.motorsState) {
        m.rpm = 0;
        m.pwm = 0;
        m.thrust = 0;
        m.current = 0;
        m.mTorque = m.pTorque = m.netTorque = 0;
        m.pwmLowPassFilter = SimITL::LowPassFilter{};
        m.propWashLowPassFilter = SimITL::LowPassFilter{};
        m.pwmHistory.clear();
    }
    ss.acceleration = {0, 0, 0};
}

void Flight::spawn(const DronePreset &preset, Vec3 pos, Quat rot, Vec3 pilotPos) {
    applyPreset(preset);
    repair();
    stopMotors();
    resetFlightTime();
    m_active = true;
    m_pilotPos = pilotPos;
    teleport(pos, rot);
}

void Flight::despawn() {
    m_active = false;
    stopMotors();
    teleport({0, 0, 0}, Quat{});
}

void Flight::movePos(Vec3 d) {
    d += m_posCarry;
    Vec3 next = m_pos + d;
    m_posCarry = d - (next - m_pos);  // the differences of nearby floats are exact
    m_pos = next;
}

void Flight::teleport(Vec3 pos, Quat rot) {
    m_pos = pos;
    m_posCarry = {};
    m_rot = normalize(rot);
    m_vel = {};
    m_angVel = {};
    simitl_command(CommandType::Reset);
    m_lastGroundContact = -1;
}

void Flight::resetFlightTime() {
    if (m_bfReady) bfb_flight_time_reset();
}

void Flight::repair() {
    simitl_command(CommandType::Repair);
    for (float &d : m_propDamage) d = 0;
#if defined(LIBFPV_KAMIKAZE)
    m_warhead.reset(m_simTime);  // a repaired drone carries a new warhead
#endif
}

#if defined(LIBFPV_KAMIKAZE)
// The blast destroys the drone: the props are gone, the motors stop, and it
// stays where it exploded (frozen) until it is repaired.
void Flight::onDetonated() {
    for (float &d : m_propDamage) d = 1.0f;
    m_vel = {};
    m_angVel = {};
    stopMotors();
}

// The bare quad plus the payload: more mass, and more inertia from a point
// mass hanging `drop` below the centre (parallel axis) plus its own solid
// sphere. The centre of mass is not moved (SimITL has no offset for it).
DronePreset Flight::withPayload(const DronePreset &bare) {
    DronePreset p = bare;
    float m = m_payloadKg >= 0 ? m_payloadKg : bare.payloadMass >= 0 ? bare.payloadMass : kAutoPayloadRatio * bare.init.quadMass;
    if (!m_warhead.installed()) m = 0;  // no bomb, no extra weight
    float drop = m_payloadDropM >= 0 ? m_payloadDropM
                 : bare.payloadDrop >= 0 ? bare.payloadDrop
                                         : 0.5f * bare.collision.bodyRadius;  // strapped under the frame
    m_payloadMass = std::max(0.0f, m);
    m_payloadPos = {0, -drop, 0};
    if (m_payloadMass <= 0) return p;
    p.init.quadMass = bare.init.quadMass + m_payloadMass;
    float own = 0.4f * m_payloadMass * kPayloadRadius * kPayloadRadius;
    auto add = [](float inv, float extra) {
        float i = 1.0f / std::max(1e-6f, std::fabs(inv));
        return std::copysign(1.0f / (i + extra), inv);
    };
    Vec3F &ii = p.init.quadInvInertia;
    ii.x = add(ii.x, m_payloadMass * drop * drop + own);  // pitch
    ii.y = add(ii.y, own);                               // yaw: on the axis
    ii.z = add(ii.z, m_payloadMass * drop * drop + own);  // roll
    return p;
}

void Flight::setPayload(float massKg, float dropM) {
    m_payloadKg = massKg;
    m_payloadDropM = dropM;
    refreshPayload();
}

void Flight::setBombInstalled(bool on) {
    if (m_warhead.installed() == on) return;
    m_warhead.setInstalled(on, m_simTime);
    refreshPayload();
}

void Flight::refreshPayload() {
    if (!m_bfReady) return;
    // patch the live physics (no reinit: keeps the flight, battery and motors)
    float dir[4];  // synced from Betaflight (syncPropDirection), not the preset
    std::memcpy(dir, m_preset.init.quadMotorDir, sizeof dir);
    m_preset = withPayload(m_basePreset);
    std::memcpy(m_preset.init.quadMotorDir, dir, sizeof dir);
    StateInit &live = SimITL::Sim::getInstance().mSimState.stateInit;
    live.quadMass = m_preset.init.quadMass;
    live.quadInvInertia = m_preset.init.quadInvInertia;
    float bottom = shapeBottom();
    buildShape();
    if (m_active) m_pos.y += bottom - shapeBottom();  // a warhead hanging lower lifts it
}

bool Flight::detonate(bool evenIfSafe) {
    if (!m_active || !m_warhead.detonate(m_simTime, m_pos, m_vel, evenIfSafe)) return false;
    onDetonated();
    return true;
}
#endif

Mat3 Flight::worldInvInertia() const {
    Mat3 r = toMat3(m_rot);
    return r * Mat3::diag(m_invInertiaBody) * transpose(r);
}

void Flight::recordImpact(const Contact &c, float speed, float impulse, int motor) {
    int slot = motor + 1;
    if (m_simTime - m_lastMotorStrike[slot] < 0.05) return;  // debounce per sphere
    m_lastMotorStrike[slot] = m_simTime;
    if (m_impacts.size() >= 64) m_impacts.erase(m_impacts.begin());
    ImpactEvent e;
    e.point = c.point;
    e.normal = c.normal;
    e.speed = speed;
    e.impulse = impulse;
    e.material = c.material;
    e.objectId = c.objectId;
    e.motorIndex = motor;
    m_impacts.push_back(e);
}

void Flight::takeImpacts(std::vector<ImpactEvent> &out) {
    out.insert(out.end(), m_impacts.begin(), m_impacts.end());
    m_impacts.clear();
}

// Sequential impulses over the contacts of all collision spheres at the
// current pose; also pushes the body out of penetration.
void Flight::resolveContacts(float dt) {
    (void)dt;
    struct Item {
        Contact c;
        int motor;
        float radius;
        float accN = 0;
        float approach = 0;
    };
    std::vector<Item> items;
    Mat3 rot = toMat3(m_rot);
    for (const Sphere &s : m_shape) {
        m_contacts.clear();
        Vec3 centre = m_pos + rot * s.local;
        m_world->sphereContacts(centre, s.radius, m_simTime, m_contacts);
        for (const Contact &c : m_contacts) items.push_back({c, s.motor, s.radius});
    }
    if (items.empty()) return;
#if defined(LIBFPV_KAMIKAZE)
    m_impactVel = m_vel;
#endif

    // positional correction: deepest penetration along each normal
    Vec3 push{};
    for (const Item &it : items) {
        float need = it.c.depth - dot(push, it.c.normal);
        if (need > 0) push += it.c.normal * need;
    }
    movePos(push);

    Mat3 invI = worldInvInertia();
    float invM = 1.0f / m_mass;
    for (Item &it : items) {
        Vec3 r = it.c.point - m_pos;
        Vec3 v = m_vel + cross(m_angVel, r) - it.c.surfaceVel;
        it.approach = -dot(v, it.c.normal);
        if (it.c.normal.y > 0.5f) m_lastGroundContact = m_simTime;
    }

    for (int iter = 0; iter < 6; iter++) {
        for (Item &it : items) {
            const Contact &c = it.c;
            Material mat = m_world->material(c.material);
            Vec3 r = c.point - m_pos;
            Vec3 v = m_vel + cross(m_angVel, r) - c.surfaceVel;
            float vn = dot(v, c.normal);
            // arms and props soak up hits on a motor: only the body bounces
            float e = (it.approach > kRestingSpeed && it.motor < 0) ? mat.restitution : 0.0f;
            Vec3 rn = cross(r, c.normal);
            float kN = invM + dot(c.normal, cross(invI * rn, r));
            float target = it.approach > 0 ? e * it.approach : 0.0f;
            float jn = (target - vn) / kN;
            float newAcc = std::max(it.accN + jn, 0.0f);
            jn = newAcc - it.accN;
            it.accN = newAcc;
            m_vel += c.normal * (jn * invM);
            m_angVel += invI * cross(r, c.normal * jn);

            // Coulomb friction against the accumulated normal impulse
            v = m_vel + cross(m_angVel, r) - c.surfaceVel;
            Vec3 vt = v - c.normal * dot(v, c.normal);
            float vtLen = length(vt);
            if (vtLen > 1e-5f) {
                Vec3 t = vt / vtLen;
                Vec3 rt = cross(r, t);
                float kT = invM + dot(t, cross(invI * rt, r));
                float jt = std::min(vtLen / kT, mat.friction * it.accN);
                m_vel -= t * (jt * invM);
                m_angVel -= invI * cross(r, t * jt);
            }

            // torsional friction: the spin about the normal, relative to the surface
            float spin = dot(m_angVel - c.surfaceAngVel, c.normal);
            float kS = dot(c.normal, invI * c.normal);
            if (std::fabs(spin) > 1e-6f && kS > 0) {
                float limit = mat.friction * it.accN * kContactPatchRatio * it.radius;
                float js = std::clamp(-spin / kS, -limit, limit);
                m_angVel += invI * (c.normal * js);
            }
        }
    }

    for (const Item &it : items) {
        Material mat = m_world->material(it.c.material);
        if (it.motor >= 0 && (mat.flags & kMatKillsProps)) m_propDamage[it.motor] = 1.0f;
        if (it.approach > kImpactReportSpeed) recordImpact(it.c, it.approach, it.accN, it.motor);
#if defined(LIBFPV_KAMIKAZE)
        if (it.approach > kWarheadTouchSpeed &&
            m_warhead.impact(m_simTime, m_pos, m_impactVel, it.c.point, it.c.normal, it.approach, it.c.material,
                             it.c.objectId, it.motor)) {
            onDetonated();
            break;
        }
#endif
    }
}

// Propeller discs touching geometry damage the props.
void Flight::propStrikes(float dt) {
    (void)dt;
    Mat3 rot = toMat3(m_rot);
    Vec3 up = rot.col(1);
    const StateOutput &out = SimITL::Sim::getInstance().getStateUpdate();
    float maxRpm = std::max(1000.0f, m_preset.init.propMaxRpm);
#if defined(LIBFPV_KAMIKAZE)
    m_impactVel = m_vel;
#endif
    for (int i = 0; i < 4; i++) {
        if (m_propDamage[i] >= 1.0f) continue;
        const Vec3F &lp = m_preset.init.quadMotorPos[i];
        Vec3 hub = m_pos + rot * Vec3{lp.x, lp.y, lp.z};
        m_contacts.clear();
        m_world->sphereContacts(hub, m_preset.collision.propRadius, m_simTime, m_contacts);
        for (const Contact &c : m_contacts) {
            if (std::fabs(dot(c.point - hub, up)) > kPropDiscHalfThickness) continue;
            Material mat = m_world->material(c.material);
            if (mat.flags & kMatKillsProps) {
                m_propDamage[i] = 1.0f;
                break;
            }
            if (m_simTime - m_lastMotorStrike[i + 1] < 0.02) continue;
            Vec3 r = c.point - m_pos;
            Vec3 v = m_vel + cross(m_angVel, r) - c.surfaceVel;
            float closing = std::max(0.0f, -dot(v, c.normal));
            float spin = std::fabs(out.motorRpm[i]) / maxRpm;
            float severity = closing + spin * 3.0f;
            if (severity < 0.5f) continue;
            m_propDamage[i] = std::min(1.0f, m_propDamage[i] + severity / std::max(1.0f, m_preset.collision.propDurability));
            recordImpact(c, closing, 0.0f, i);
#if defined(LIBFPV_KAMIKAZE)
            // a spinning prop hitting something is a touch too
            if (m_warhead.impact(m_simTime, m_pos, m_impactVel, c.point, c.normal, closing, c.material, c.objectId, i))
                onDetonated();
#endif
            break;
        }
    }
}

void Flight::updateGroundEffect(float out[4]) {
    Mat3 rot = toMat3(m_rot);
    Vec3 down = -rot.col(1);
    float maxD = std::max(0.01f, m_preset.maxGroundEffectDist);
    for (int i = 0; i < 4; i++) {
        out[i] = 0;
        const Vec3F &lp = m_preset.init.quadMotorPos[i];
        Vec3 hub = m_pos + rot * Vec3{lp.x, lp.y, lp.z};
        float t;
        Vec3 n;
        uint8_t mat;
        uint32_t obj;
        if (m_world->raycast(hub, down, maxD, m_simTime, t, n, mat, obj)) out[i] = std::clamp(1.0f - t / maxD, 0.0f, 1.0f);
    }
}

void Flight::updateVideo() {
    // occlusion between pilot (receiver) and drone (transmitter), 50 Hz
    if (m_videoTick++ % 20 == 0) {
        Vec3 rx = m_pilotPos + Vec3{0, 1.5f, 0};
        Vec3 d = m_pos - rx;
        float dist = length(d);
        float blocked = 0;
        if (dist > 0.5f) {
            float t;
            Vec3 n;
            uint8_t mat;
            uint32_t obj;
            if (m_world->raycast(rx, d / dist, dist - 0.3f, m_simTime, t, n, mat, obj, true)) blocked = 1;
        }
        m_videoOcclusion += (blocked - m_videoOcclusion) * 0.25f;
    }
    float dist = length(m_pos - m_pilotPos);
    const VtxSettings &v = m_preset.vtx;
    float rangeFrac = dist / std::max(1.0f, v.rangeM);
    float noise = v.noiseFloor + v.noiseFactor * 0.1f * rangeFrac * rangeFrac * 10.0f * 0.1f + m_videoOcclusion * 0.45f;
    noise = std::clamp(noise + std::max(0.0f, rangeFrac - 0.8f) * 2.5f, 0.0f, 1.0f);
    m_snap.videoNoise = noise;
    m_snap.videoBreakup = std::clamp((noise - 0.55f) * 2.2f, 0.0f, 1.0f);
    m_snap.videoSignal = std::clamp(1.0f - noise, 0.0f, 1.0f);
}

void Flight::setHome(const Vec3 *home) {
    std::lock_guard<std::mutex> lock(m_homeMutex);
    m_hasHome = home != nullptr;
    if (home) m_home = *home;
}

void Flight::setWind(WindState state) {
    std::lock_guard<std::mutex> lock(m_windMutex);
    m_windIn = std::move(state);
    m_windChanged = true;
}

void Flight::addBlast(Vec3 pos, float radius) {
    std::lock_guard<std::mutex> lock(m_windMutex);
    m_blastsIn.push_back({pos, radius, 0});
}

// The air at the centre of mass and at each motor hub, for SimITL's
// aerodynamics. Returns the one at the centre of mass.
Vec3 Flight::updateWind(float dt) {
    {
        std::lock_guard<std::mutex> lock(m_windMutex);
        if (m_windChanged) m_wind = m_windIn;
        m_windChanged = false;
        for (WindBlast &b : m_blastsIn) {
            b.start = m_simTime;
            if (m_active) m_blasts.push_back(b);  // none waits for a later drone
        }
        m_blastsIn.clear();
    }
    m_blasts.erase(std::remove_if(m_blasts.begin(), m_blasts.end(),
                                  [&](const WindBlast &b) { return blastOver(b, m_simTime); }),
                   m_blasts.end());
    m_windClock.advance(m_wind, dt);
    Vec3 com;
    Vec3 hub[4];
    if (m_active && (m_wind.on || !m_blasts.empty())) {
        com = sampleWind(m_wind, m_windClock, m_blasts, m_pos, m_simTime);
        Mat3 rot = toMat3(m_rot);
        for (int i = 0; i < 4; i++) {
            const Vec3F &lp = m_preset.init.quadMotorPos[i];
            hub[i] = sampleWind(m_wind, m_windClock, m_blasts, m_pos + rot * Vec3{lp.x, lp.y, lp.z}, m_simTime);
        }
    }
    m_input.windVelocity = {com.x, com.y, com.z};
    for (int i = 0; i < 4; i++) m_input.rotorWind[i] = {hub[i].x, hub[i].y, hub[i].z};
    return com;
}

void Flight::step(float dt, const float channels[kChannelCount], bool radioLost) {
    std::unique_lock<std::mutex> worldLock(m_world->mutex());

    m_simTime += dt;
    // --- inputs for SimITL ----------------------------------------------
    for (int i = 0; i < kChannelCount; i++) m_input.rcData[i] = std::clamp(channels[i], -1.0f, 1.0f);
    for (int i = kChannelCount; i < 16; i++) m_input.rcData[i] = -1.0f;  // unused channels
#if defined(LIBFPV_KAMIKAZE)
    if (m_active) m_warhead.filterChannels(m_input.rcData, bfb_is_armed());
#endif
    if (!m_active) {
        // parked: never armed, Betaflight still runs for the Configurator
        m_input.rcData[4] = -1.0f;
        m_input.rcData[2] = -1.0f;
        m_pos = {};
        m_posCarry = {};
        m_rot = Quat{};
        m_vel = {};
        m_angVel = {};
    }
    SimITL::BF::setRcFailsafe(radioLost && m_active);
    // turtle mode: Betaflight picks it at arming, SimITL spins the props backwards
    m_input.motorsReversed = bfb_crash_flip_active() ? 1 : 0;

    if ((m_videoTick & 63) == 0) syncPropDirection();  // follows Configurator edits

    float ge[4] = {0, 0, 0, 0};
    if (m_active) updateGroundEffect(ge);
    Vec3 wind = updateWind(dt);
    m_input.delta = dt;
    m_input.position = {m_pos.x, m_pos.y, m_pos.z};
    fillRotation(m_input, m_rot);
    m_input.linearVelocity = {m_vel.x, m_vel.y, m_vel.z};
    m_input.angularVelocity = {m_angVel.x, m_angVel.y, m_angVel.z};
    m_input.contact = 2;  // our velocities are authoritative (patch 0002)
    for (int i = 0; i < 4; i++) {
        m_input.groundEffect[i] = ge[i];
        m_input.propDamage[i] = m_propDamage[i];
    }

    // internal X right, Y up, Z forward = east, up, north
    bfb_gps_update(m_pos.x, m_pos.z, m_pos.y, m_vel.x, m_vel.z, m_vel.y);
    bfb_baro_update(m_pos.y);

    simitl_update(m_input);
    // after the update: arming inside it may have just moved home
    {
        std::lock_guard<std::mutex> lock(m_homeMutex);
        if (m_hasHome) bfb_gps_set_home(m_home.x, m_home.z, m_home.y);
    }
    const StateOutput &out = SimITL::Sim::getInstance().getStateUpdate();

    // --- integrate pose with collisions -----------------------------------
#if defined(LIBFPV_KAMIKAZE)
    if (m_active && m_warhead.detonated()) {
        // the wreck stays where it exploded until repaired
        m_vel = {};
        m_angVel = {};
    } else
#endif
    if (m_active) {
        m_rot = normalize(Quat{out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w});
        m_vel = {out.linearVelocity.x, out.linearVelocity.y, out.linearVelocity.z};
        m_angVel = {out.angularVelocity.x, out.angularVelocity.y, out.angularVelocity.z};
        if (!finite(m_vel) || !finite(m_angVel) || !std::isfinite(m_rot.w)) {
            FPV_LOGW("flight model produced a non-finite state; resetting velocities");
            m_vel = {};
            m_angVel = {};
            m_rot = Quat{};
            simitl_command(CommandType::Reset);
        }
        float speed = length(m_vel);
        if (speed > kMaxSpeed) m_vel *= kMaxSpeed / speed;
        if (length(m_angVel) > kMaxAngularSpeed) m_angVel *= kMaxAngularSpeed / length(m_angVel);

        // Substeps keep every sphere moving less than half its radius, so
        // thin geometry cannot be skipped (conservative advancement).
        float minRadius = m_preset.collision.motorRadius;
        for (const Sphere &s : m_shape) minRadius = std::min(minRadius, s.radius);
        float travel = length(m_vel) * dt + length(m_angVel) * dt * 0.15f;
        int sub = std::clamp((int)std::ceil(travel / (0.5f * minRadius)), 1, 64);
        float h = dt / (float)sub;
        resolveContacts(h);
        for (int k = 0; k < sub; k++) {
            movePos(m_vel * h);
            resolveContacts(h);
        }
        propStrikes(dt);
    }
#if defined(LIBFPV_KAMIKAZE)
    if (m_active && m_warhead.update(dt, m_simTime, m_input.rcData, bfb_is_armed(), m_pos, m_vel)) onDetonated();
    // the warhead state is Betaflight's custom message 1 element: placed,
    // layered and hidden (OSD disable switch) like any other element
    char bombText[24] = "";
    if (m_active) m_warhead.statusText(bombText, sizeof bombText, m_simTime, bfb_is_armed());
    bfb_osd_set_host_message(0, bombText);
#endif

    // --- snapshot -----------------------------------------------------------
    const SimITL::SimState &ss = SimITL::Sim::getInstance().mSimState;
    m_snap.simTime = m_simTime;
    m_snap.active = m_active;
    m_snap.pos = m_pos;
    m_snap.rot = m_rot;
    m_snap.vel = m_vel;
    m_snap.angVel = m_angVel;
    m_snap.wind = wind;
    bool anyBurned = false, anyBroken = false;
    for (int i = 0; i < 4; i++) {
        MotorSnapshot &m = m_snap.motors[i];
        m.rpm = out.motorRpm[i];
        m.thrust = out.thrust[i];
        m.current = out.motorCurrent[i];
        m.temperature = out.motorT[i];
        m.command = std::clamp(out.motorOutput[i], 0.0f, 1.0f);
        m.propDamage = m_propDamage[i];
        m.flags = 0;
        if (out.motorStatus[i] & MotorStatus::MotorBurnedOut) {
            m.flags |= 1u;
            anyBurned = true;
        }
        if (out.motorStatus[i] & MotorStatus::MotorDamaged || m_propDamage[i] > 0.05f) m.flags |= 2u;
        if (m_propDamage[i] >= 0.99f) {
            m.flags |= 4u;
            anyBroken = true;
        }
    }
    m_snap.batVoltage = out.batteryVoltage;
    m_snap.batVoltageSag = ss.batteryState.batVoltageSag;
    m_snap.batCurrent = (float)ss.batteryState.amperage;
    m_snap.mahDrawn = (float)ss.batteryState.mAhDrawn;
    m_snap.armed = bfb_is_armed();
    m_snap.failsafe = bfb_failsafe_active();
    m_snap.beeper = out.beep != 0;
    m_snap.armingDisableFlags = bfb_arming_disable_flags();
    uint32_t modes = bfb_flight_mode_flags();
    m_snap.modeAngle = (modes & 1u) != 0;    // ANGLE_MODE
    m_snap.modeHorizon = (modes & 2u) != 0;  // HORIZON_MODE
    m_snap.turtle = m_input.motorsReversed != 0;
    m_snap.osdDisabled = bfb_osd_disabled() != 0;
    m_snap.onGround = m_active && m_lastGroundContact >= 0 && m_simTime - m_lastGroundContact < 0.05;
    m_snap.crashed = m_active && (anyBurned || anyBroken);
    if (std::memcmp(m_snap.osd.data(), out.osd, m_snap.osd.size()) != 0) {
        std::memcpy(m_snap.osd.data(), out.osd, m_snap.osd.size());
        m_snap.osdRevision++;
    }
    for (int i = 0; i < kChannelCount; i++) m_snap.channels[i] = m_input.rcData[i];
    if (m_active) {
        float t;
        Vec3 n;
        uint8_t mat;
        uint32_t obj;
        if ((m_videoTick & 7) == 0) {
            m_snap.altitude = m_world->raycast(m_pos, {0, -1, 0}, 1000.0f, m_simTime, t, n, mat, obj) ? t : -1.0f;
        }
        updateVideo();
    }
#if defined(LIBFPV_KAMIKAZE)
    m_snap.warhead = m_warhead.snapshot();
    m_snap.payloadMass = m_payloadMass;
    m_snap.totalMass = m_preset.init.quadMass;
    if (m_active && m_warhead.detonated()) {
        // the video transmitter went with it: no signal
        m_snap.videoNoise = m_snap.videoBreakup = 1.0f;
        m_snap.videoSignal = 0.0f;
        m_snap.crashed = true;
    }
#endif
}

}  // namespace fpv
