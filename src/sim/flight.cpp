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
constexpr float kWinchSpeed = 0.5f;          // m/s, the cargo line reeled in or out
constexpr float kCargoDragCoef = 1.0f;       // the hanging cargo's drag coefficient (on pi r^2)
constexpr float kCargoRestitution = 0.15f;
constexpr float kMaxAngularSpeed = 100.0f;   // rad/s, SimITL's own clamp
// After a repair or a teleport the props take no damage for this long (sim
// seconds): it covers a repair and a reset called one after the other while
// the simulation keeps stepping, and the host's next frame or two, whose
// world (a water plane following the drone, its own damage checks) still
// describes the old spot. Spawned straight into lava, the props still go.
constexpr double kRespawnDamageGrace = 0.25;
// A hit this hard (m/s) or harder hurts the video transmitter for good: its
// antenna bends, the connector cracks. 1/kVtxHitScale of its health per m/s
// above it (a 10 m/s crash ~12 %, a 45 m/s one the whole of it).
constexpr float kVtxHitSpeed = 5.0f, kVtxHitScale = 40.0f;
// What a fully damaged VTX costs the link budget, dB (a tenth of the range).
constexpr float kVtxDamageLossDb = 20.0f;
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
    // the cargo collides as one sphere around its centre (on a line: on its own)
    if (m_cargo.radius > 0 && !hanging()) m_shape.push_back({m_cargo.offset, m_cargo.radius, kCargoMotorIndex});
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
    m_basePreset = presetIn;
    const DronePreset preset = withExtraMass(presetIn);
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
    m_portalPasses = 0;  // counted per drone
    m_lastPortal = -1;
    m_rcLink.reset(m_rcLink.config(), 0x5eed + m_teleports);  // a new drone: a fresh link
    m_rcPathGain = -60;
    m_rcLossDb = m_rcLossTarget = 0;
    m_rcLastPacket = -1;
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
    m_teleports++;
    if (hanging()) {
        m_cargoPos = pos + (m_cargoPos - m_pos);
        m_cargoVel = {};
    }
    m_pos = pos;
    m_posCarry = {};
    m_rot = normalize(rot);
    m_vel = {};
    m_angVel = {};
    simitl_command(CommandType::Reset);
    rcRespawnGrace();
    m_damageGraceUntil = m_simTime + kRespawnDamageGrace;
    m_lastGroundContact = -1;
}

// A respawned (moved or repaired) drone gets 1 s in which no RC packet is
// lost, so a link that was down (a wall, a blast) never leaves Betaflight in
// RXLOSS, refusing to arm, right after the respawn.
void Flight::rcRespawnGrace() {
    m_rcImmuneUntil = m_simTime + 1.0;
    // and Betaflight skips its own recovery (FAILSAFE, then NOT_DISARMED)
    if (m_bfReady && m_active) bfb_rx_recovered();
}

void Flight::resetFlightTime() {
    if (m_bfReady) bfb_flight_time_reset();
}

void Flight::repair() {
    simitl_command(CommandType::Repair);
    for (float &d : m_propDamage) d = 0;
    m_vtxDamage = 0;
    rcRespawnGrace();
    m_damageGraceUntil = m_simTime + kRespawnDamageGrace;
#if defined(LIBFPV_KAMIKAZE)
    m_warhead.reset(m_simTime);  // a repaired drone carries a new warhead
#endif
}

void Flight::damageProps(int motor, float amount) {
    if (!canDamage() || !m_active || !(amount > 0)) return;
    for (int i = 0; i < 4; i++)
        if (motor < 0 || motor == i) m_propDamage[i] = std::min(1.0f, m_propDamage[i] + amount);
    // fire, a blast, a bullet: the whole drone takes it, VTX included; one
    // prop's hit reaches it less
    damageVtx(motor < 0 ? amount : 0.25f * amount);
}

void Flight::damageVtx(float amount) {
    if (!canDamage() || !m_active || !(amount > 0)) return;
    m_vtxDamage = std::min(1.0f, m_vtxDamage + amount);
}

// The charge a step drew is put back after it, so the pack stays at the
// level it had when drain was turned off (a respawn or repair still fits a
// full one); the sag (current times the internal resistance) still follows
// the load.
void Flight::keepBattery(double capacity) {
    SimITL::SimState &ss = SimITL::Sim::getInstance().mSimState;
    ss.batteryState.batCapacity = capacity;
    ss.batteryState.mAhDrawn = ss.stateInit.quadBatCapacityCharged - capacity;
}

// The windings never reach the burn-out temperature (they still warm up),
// and a motor burned before the option was turned on works again.
void Flight::keepMotors() {
    SimITL::SimState &ss = SimITL::Sim::getInstance().mSimState;
    for (auto &m : ss.motorsState) {
        m.temp = std::min(m.temp, ss.stateInit.motorMaxT - 1.0f);
        m.status &= ~(int32_t)MotorStatus::MotorBurnedOut;
    }
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

void Flight::setPayload(float massKg, float dropM) {
    m_payloadKg = massKg;
    m_payloadDropM = dropM;
    refreshMass();
}

void Flight::setBombInstalled(bool on) {
    if (m_warhead.installed() == on) return;
    m_warhead.setInstalled(on, m_simTime);
    refreshMass();
}

bool Flight::detonate(bool evenIfSafe) {
    if (!m_active || !m_warhead.detonate(m_simTime, m_pos, m_vel, evenIfSafe)) return false;
    onDetonated();
    return true;
}
#endif

// The bare quad plus what it carries (the cargo, fpv_drone_set_cargo, and any
// payload the build adds). More mass, and more inertia from point
// masses away from the centre (parallel axis) plus their own solid spheres.
// The centre of mass is not moved (SimITL has no offset for it).
DronePreset Flight::withExtraMass(const DronePreset &bare) {
    DronePreset p = bare;
    float extraMass = 0;
    Vec3 extraI{};  // about pitch (X), yaw (Y), roll (Z)
    auto addPoint = [&](float m, Vec3 at, float radius) {
        if (!(m > 0)) return;
        float own = 0.4f * m * radius * radius;
        extraMass += m;
        extraI.x += m * (at.y * at.y + at.z * at.z) + own;
        extraI.y += m * (at.x * at.x + at.z * at.z) + own;
        extraI.z += m * (at.x * at.x + at.y * at.y) + own;
    };
#if defined(LIBFPV_KAMIKAZE)
    float m = m_payloadKg >= 0 ? m_payloadKg : bare.payloadMass >= 0 ? bare.payloadMass : kAutoPayloadRatio * bare.init.quadMass;
    if (!m_warhead.installed()) m = 0;  // no bomb, no extra weight
    float drop = m_payloadDropM >= 0 ? m_payloadDropM
                 : bare.payloadDrop >= 0 ? bare.payloadDrop
                                         : 0.5f * bare.collision.bodyRadius;  // strapped under the frame
    m_payloadMass = std::max(0.0f, m);
    m_payloadPos = {0, -drop, 0};
    addPoint(m_payloadMass, m_payloadPos, kPayloadRadius);
#endif
    // on a line only its pull reaches the quad (stepCargo), not its mass
    if (!hanging()) addPoint(m_cargo.mass, m_cargo.offset, std::max(m_cargo.radius, 0.02f));
    if (extraMass <= 0) return p;
    p.init.quadMass = bare.init.quadMass + extraMass;
    auto add = [](float inv, float extra) {
        float i = 1.0f / std::max(1e-6f, std::fabs(inv));
        return std::copysign(1.0f / (i + extra), inv);
    };
    Vec3F &ii = p.init.quadInvInertia;
    ii.x = add(ii.x, extraI.x);
    ii.y = add(ii.y, extraI.y);
    ii.z = add(ii.z, extraI.z);
    return p;
}

void Flight::refreshMass() {
    if (!m_bfReady) return;
    // patch the live physics (no reinit: keeps the flight, battery and motors)
    float dir[4];  // synced from Betaflight (syncPropDirection), not the preset
    std::memcpy(dir, m_preset.init.quadMotorDir, sizeof dir);
    m_preset = withExtraMass(m_basePreset);
    std::memcpy(m_preset.init.quadMotorDir, dir, sizeof dir);
    StateInit &live = SimITL::Sim::getInstance().mSimState.stateInit;
    live.quadMass = m_preset.init.quadMass;
    live.quadInvInertia = m_preset.init.quadInvInertia;
    float bottom = shapeBottom();
    buildShape();
    if (m_active) m_pos.y += bottom - shapeBottom();  // something hanging lower lifts it
}

void Flight::setCargo(const Cargo &cargo) {
    bool wasHanging = hanging();
    m_cargo = cargo;
    if (!(m_cargo.mass > 0)) m_cargo.mass = 0;
    if (!(m_cargo.radius > 0)) m_cargo.radius = 0;
    if (!(m_cargo.line > 0)) m_cargo.line = 0;
    if (hanging() && !wasHanging) {
        // picked up where it is: the line as long as it is far, reeled in from there
        m_cargoPos = m_pos + toMat3(m_rot) * m_cargo.offset;
        m_cargoVel = m_vel;
        m_lineNow = std::max(m_cargo.line, length(m_cargoPos - hookPoint()));
        m_lineTaut = false;
    }
    // picked up off the ground, not out of it: no lift (unlike a strapped payload)
    Vec3 pos = m_pos;
    refreshMass();
    m_pos = pos;
}

Vec3 Flight::hookPoint() const { return m_pos + toMat3(m_rot) * Vec3{0, -m_preset.collision.bodyRadius, 0}; }

// A cargo on a line is a body of its own: it falls, drags in the air, rests
// on and slides along the world, and the line, when taut, holds it within
// its length of the hook. The line's pull goes through the quad's centre of
// mass (an impulse, split by the two masses), so the quad keeps its own
// inertia and the flight controller its authority: the pilot feels the
// weight and the swing, as with a real slung load. The winch reels the line
// to the requested length at kWinchSpeed.
void Flight::stepCargo(float dt) {
    const float r = std::max(0.02f, m_cargo.radius);
    const float invMc = 1.0f / m_cargo.mass;
    const float area = 3.14159265f * r * r;
    int sub = std::clamp((int)std::ceil(length(m_cargoVel) * dt / (0.5f * r)), 1, 16);
    float h = dt / (float)sub;
    Vec3 wind{m_input.windVelocity.x, m_input.windVelocity.y, m_input.windVelocity.z};
    for (int k = 0; k < sub; k++) {
        m_cargoVel.y -= 9.81f * h;
        Vec3 rel = m_cargoVel - wind;
        float drag = 0.5f * 1.225f * kCargoDragCoef * area * invMc * length(rel) * h;
        m_cargoVel -= rel * std::min(1.0f, drag);
        m_cargoPos += m_cargoVel * h;
        m_contacts.clear();
        if (m_cargo.radius > 0) m_world->sphereContacts(m_cargoPos, m_cargo.radius, m_simTime, m_contacts);
        dropPortalHoleContacts(0);
        Vec3 push{};
        for (const Contact &c : m_contacts) {
            float need = c.depth - dot(push, c.normal);
            if (need > 0) push += c.normal * need;
            Vec3 v = m_cargoVel - c.surfaceVel;
            float vn = dot(v, c.normal);
            if (vn >= 0) continue;
            Material mat = m_world->material(c.material);
            if (-vn > kImpactReportSpeed)
                recordImpact(c, -vn, -vn * m_cargo.mass, kCargoMotorIndex);
            float e = -vn > kRestingSpeed ? kCargoRestitution : 0.0f;
            v -= c.normal * (vn * (1 + e));
            // Coulomb friction from the normal impulse just taken
            Vec3 vt = v - c.normal * dot(v, c.normal);
            float vtLen = length(vt);
            if (vtLen > 1e-5f) v -= vt * std::min(1.0f, mat.friction * -vn / vtLen);
            m_cargoVel = v + c.surfaceVel;
        }
        m_cargoPos += push;
    }

    // the winch, then the line
    m_lineNow = m_lineNow > m_cargo.line ? std::max(m_cargo.line, m_lineNow - kWinchSpeed * dt)
                                         : std::min(m_cargo.line, m_lineNow + kWinchSpeed * dt);
    Vec3 hook = hookPoint();
    Vec3 d = m_cargoPos - hook;
    float len = length(d);
    m_lineTaut = false;
    if (len > m_lineNow && len > 1e-5f) {
        Vec3 n = d / len;
        float invMd = 1.0f / m_mass;
        float w = invMd + invMc;
        float stretch = len - m_lineNow;
        m_cargoPos -= n * (stretch * invMc / w);
        movePos(n * (stretch * invMd / w));
        float vrel = dot(m_cargoVel - m_vel, n);
        if (vrel > 0) {
            float j = vrel / w;
            m_cargoVel -= n * (j * invMc);
            m_vel += n * (j * invMd);
        }
        m_lineTaut = true;
    }
    // the cargo and the quad don't go through each other (the quad lands on it)
    Vec3 dc = m_cargoPos - m_pos;
    float lc = length(dc), minD = m_cargo.radius + m_preset.collision.bodyRadius;
    if (m_cargo.radius > 0 && lc < minD) {
        Vec3 n = lc > 1e-5f ? dc / lc : Vec3{0, -1, 0};
        float invMd = 1.0f / m_mass;
        float w = invMd + invMc;
        float overlap = minD - lc;
        m_cargoPos += n * (overlap * invMc / w);
        movePos(-n * (overlap * invMd / w));
        float vrel = dot(m_cargoVel - m_vel, n);
        if (vrel < 0) {
            float j = -vrel / w;
            m_cargoVel += n * (j * invMc);
            m_vel -= n * (j * invMd);
        }
    }
    if (!finite(m_cargoPos) || !finite(m_cargoVel)) {
        m_cargoPos = hook - Vec3{0, m_lineNow, 0};
        m_cargoVel = m_vel;
    }
}

void Flight::push(Vec3 velocity, bool replace) {
    if (!m_active || !finite(velocity)) return;
    m_vel = replace ? velocity : m_vel + velocity;
}

void Flight::setPortals(std::vector<PortalLink> links) {
    std::lock_guard<std::mutex> lock(m_portalMutex);
    m_portalsIn = std::move(links);
    m_portalsChanged = true;
}

void Flight::dropPortalHoleContacts(size_t first) {
    if (m_portals.empty()) return;
    auto end = std::remove_if(m_contacts.begin() + (std::ptrdiff_t)first, m_contacts.end(), [&](const Contact &c) {
        return !c.throughPortal && portalHoleAt(m_portals, c.point) >= 0;
    });
    m_contacts.erase(end, m_contacts.end());
}

// The centre went from in front of an opening to behind it, through the
// opening: the drone leaves the linked one, pose, velocity and spin turned
// with it. Betaflight gets the new attitude from the next step's state.
void Flight::passPortals(Vec3 before) {
    for (size_t i = 0; i < m_portals.size(); i++) {
        for (int s = 0; s < 2; s++) {
            const Portal &in = m_portals[i].side(s);
            Vec3 a = in.toLocal(before), b = in.toLocal(m_pos);
            if (!(a.z > 0 && b.z <= 0)) continue;
            float t = a.z / (a.z - b.z);
            if (!in.insideOpening(a + (b - a) * t)) continue;
            PortalTransform x = portalTransform(in, m_portals[i].side(1 - s));
            m_pos = x.point(m_pos);
            m_posCarry = {};
            m_rot = x.orient(m_rot);
            m_vel = x.dir(m_vel);
            m_angVel = x.dir(m_angVel);
            m_teleports++;
            m_portalPasses++;
            m_lastPortal = (int)(i * 2 + s);
            if (hanging()) {
                // the cargo comes along on its line, where it was relative to the drone
                m_cargoPos = x.point(m_cargoPos);
                m_cargoVel = x.dir(m_cargoVel);
            }
            return;  // one pass per step
        }
    }
}

Mat3 Flight::worldInvInertia() const {
    Mat3 r = toMat3(m_rot);
    return r * Mat3::diag(m_invInertiaBody) * transpose(r);
}

void Flight::recordImpact(const Contact &c, float speed, float impulse, int motor) {
    int slot = motor == kCargoMotorIndex ? 5 : motor + 1;
    if (m_simTime - m_lastMotorStrike[slot] < 0.05) return;  // debounce per sphere
    m_lastMotorStrike[slot] = m_simTime;
    // a hard hit damages the VTX for good (with damage on); the jolt every
    // hit gives the picture is the feed's (VideoFeed), damage or not
    if (motor != kCargoMotorIndex && speed > kVtxHitSpeed) damageVtx((speed - kVtxHitSpeed) / kVtxHitScale);
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
        dropPortalHoleContacts(0);
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
        if (it.motor >= 0 && (mat.flags & kMatKillsProps) && canDamage()) m_propDamage[it.motor] = 1.0f;
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
        dropPortalHoleContacts(0);
        for (const Contact &c : m_contacts) {
            if (std::fabs(dot(c.point - hub, up)) > kPropDiscHalfThickness) continue;
            Material mat = m_world->material(c.material);
            if (mat.flags & kMatKillsProps) {
                if (canDamage()) m_propDamage[i] = 1.0f;
                break;
            }
            if (m_simTime - m_lastMotorStrike[i + 1] < 0.02) continue;
            Vec3 r = c.point - m_pos;
            Vec3 v = m_vel + cross(m_angVel, r) - c.surfaceVel;
            float closing = std::max(0.0f, -dot(v, c.normal));
            float spin = std::fabs(out.motorRpm[i]) / maxRpm;
            float severity = closing + spin * 3.0f;
            if (severity < 0.5f) continue;
            if (canDamage())
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

// The video link's budget (D34): free-space loss, the ground's reflection,
// both antennas' patterns and what stands between the pilot and the drone.
// The fast fading is the host thread's (VideoFeed), from the K factor and
// the Doppler given here.
void Flight::updateVideo(float dt) {
    Vec3 rx = m_pilotPos + Vec3{0, kVideoRxHeight, 0};
    Vec3 d = m_pos - rx;
    float dist = length(d);
    VideoProbe &v = m_videoProbe;
    // Obstruction, spread over a 20-tick cycle: the direct ray from both
    // ends (how deep the obstacle is), four paths bent through the first
    // Fresnel zone's edge (partial blockage, diffraction round a corner).
    uint32_t slot = m_videoTick++ % 20;
    if (dist > 0.5f) {
        Vec3 u = d / dist;
        float t;
        Vec3 n;
        uint8_t mat;
        uint32_t obj;
        auto blocked = [&](Vec3 a, Vec3 b, float *tHit) {
            Vec3 ab = b - a;
            float len = length(ab);
            if (len < 0.3f) return false;
            bool hit = m_world->raycast(a, ab / len, len - 0.15f, m_simTime, t, n, mat, obj, true);
            if (hit && tHit) *tHit = t;
            return hit;
        };
        if (slot == 0) {
            v.centre = blocked(rx, m_pos, &v.tNear);
            v.offsets = 0;
        } else if (slot == 1 && v.centre) {
            if (!blocked(m_pos, rx, &v.tFar)) v.tFar = 0;
        } else if (slot >= 2 && slot <= 5) {
            Vec3 a = normalize(cross(u, {0, 1, 0}), {1, 0, 0});
            Vec3 b = cross(a, u);
            float r1 = std::sqrt(kVideoWavelength * dist * 0.25f);  // first Fresnel zone at the middle
            Vec3 off = (slot == 2 ? a : slot == 3 ? -a : slot == 4 ? b : -b) * (0.6f * r1);
            Vec3 mid = (rx + m_pos) * 0.5f + off;
            if (blocked(rx, mid, nullptr) || blocked(mid, m_pos, nullptr)) v.offsets++;
        } else if (slot == 6) {
            float depth = v.centre ? std::max(0.0f, dist - v.tNear - v.tFar) : 0.0f;
            float targetLoss, targetK;
            videoObstruction(v.centre, v.offsets, depth, targetLoss, targetK);
            m_videoTarget[0] = targetLoss;
            m_videoTarget[1] = targetK;
            // the RC link sees the same obstacles at its own band (D36)
            m_rcLossTarget = rcObstructionDb(rcLinkMode(m_rcLink.config().mode).band, v.centre, v.offsets, depth);
        }
        // the game's world, probed through its raycast on the host thread:
        // whichever blocks more
        float hostLoss = m_hostVideoLoss;
        if (hostLoss >= 0) {
            m_videoTarget[0] = std::max(m_videoTarget[0], hostLoss);
            m_videoTarget[1] = std::min(m_videoTarget[1], (float)m_hostVideoK);
        }
        float hostRcLoss = m_hostRcLoss;
        if (hostRcLoss >= 0) m_rcLossTarget = std::max(m_rcLossTarget, hostRcLoss);
    } else {
        m_videoTarget[0] = 0;
        m_videoTarget[1] = 10;
        m_rcLossTarget = 0;
    }
    float a = 1 - std::exp(-std::max(dt, 0.0f) / 0.12f);
    v.lossDb += (m_videoTarget[0] - v.lossDb) * a;
    v.kFactor += (m_videoTarget[1] - v.kFactor) * a;
    m_rcLossDb += (m_rcLossTarget - m_rcLossDb) * a;

    const VtxSettings &vtx = m_preset.vtx;
    float powerDbm = 10.0f * std::log10(std::max(vtx.powerMw, 0.01f));
    Vec3 toPilotLocal = rotate(conj(m_rot), -d);
    float elevation = dist > 0.01f ? std::asin(std::clamp(d.y / dist, -1.0f, 1.0f)) : 0.0f;
    float height = m_snap.altitude >= 0 ? m_snap.altitude : 10.0f;
    float rxDbm = powerDbm + txAntennaGainDb(toPilotLocal, m_preset.vtxShadowDb) + rxAntennaGainDb(elevation) - freeSpaceLossDb(dist) -
                  groundLossDb(dist, std::max(height, 0.1f), kVideoRxHeight) - v.lossDb - kVideoSystemLossDb -
                  kVtxDamageLossDb * m_vtxDamage;
    // low over the ground the reflection is a strong second path
    float k = v.kFactor * std::clamp(height / 3.0f, 0.3f, 1.0f);
    m_snap.videoRxDbm = rxDbm;
    m_snap.videoK = k;
    m_snap.videoDoppler = length(m_vel) / kVideoWavelength;
    // fpv_video_state: 0..1 summaries against the receiver's threshold
    float cnr = vtx.digital ? rxDbm - vtx.sensitivityDbm + 10.0f : rxDbm - kVideoNoiseFloorDbm;
    float signal = std::clamp((cnr + 6.0f) / 26.0f, 0.0f, 1.0f);
    m_snap.videoSignal = signal;
    m_snap.videoNoise = 1.0f - signal;
    m_snap.videoBreakup = std::clamp((10.0f - cnr) / 12.0f, 0.0f, 1.0f);

    // The RC link's path (D36), for the next step's packets: the radio at
    // the pilot's chest, the receiver's antenna on the frame, the same
    // ground and obstacles at the link's wavelength.
    const RcBand band = rcLinkMode(m_rcLink.config().mode).band;
    const float lambda = rcWavelength(band);
    Vec3 dr = m_pos - (m_pilotPos + Vec3{0, kRcTxHeight, 0});
    float rdist = length(dr);
    m_rcPathGain = kRcTxGainDbi + rcRxAntennaGainDb(band, rotate(conj(m_rot), -dr)) - freeSpaceLossDb(rdist, lambda) -
                   groundLossDb(rdist, std::max(height, 0.1f), kRcTxHeight, lambda) - m_rcLossDb - kRcSystemLossDb;
    m_rcK = k;
    m_rcDoppler = length(m_vel) / lambda;
}

void Flight::setRcLinkConfig(const RcLinkConfig &cfg) {
    std::lock_guard<std::mutex> lock(m_rcMutex);
    m_rcIn = cfg;
    m_rcChanged = true;
}

// The simulated link decides each packet SimITL's receiver delivers (D36).
static Flight *g_rcFlight = nullptr;
static bool rcPacketLost() { return g_rcFlight && !g_rcFlight->rcPacketArrives(); }

// Before Betaflight's step: the player's setting, the packet rate, and the
// inputs the packets of this step see.
void Flight::updateRcLink() {
    bool changed;
    {
        std::lock_guard<std::mutex> lock(m_rcMutex);
        changed = m_rcChanged;
        if (changed) m_rcLink.configure(m_rcIn);
        m_rcChanged = false;
    }
    if (changed) {
        g_rcFlight = this;
        SimITL::BF::setRcPacketLossHook(&rcPacketLost);
        const RcLinkConfig &c = m_rcLink.config();
        SimITL::BF::setRcPacketRate(c.enabled ? rcLinkMode(c.mode).rateHz : 500.0f);
    }
    m_rcInputs = RcLinkInputs{};
    m_rcInputs.pathGainDb = m_rcPathGain;
    m_rcInputs.kFactor = m_rcK;
    m_rcInputs.dopplerHz = m_rcDoppler;
    m_rcInputs.armed = bfb_is_armed();
    m_rcInputs.immune = m_simTime < m_rcImmuneUntil;
#if defined(LIBFPV_KAMIKAZE)
    m_rcInputs.alive = !m_warhead.detonated();
#endif
    if (!m_rcLink.config().enabled || !m_active) m_rcLink.packet(RcLinkInputs{});  // a perfect link's status
}

// SimITL's packet slot (inside simitl_update): whether this packet gets
// through. Lost: Betaflight keeps the last sticks; long enough, RXLOSS.
bool Flight::rcPacketArrives() {
    if (!m_rcLink.config().enabled || !m_active) return true;
    float elapsed = m_rcLastPacket >= 0 ? (float)(m_simTime - m_rcLastPacket) : 0.0f;
    m_rcLastPacket = m_simTime;
    return m_rcLink.packet(m_rcInputs, elapsed);
}

// The link statistics an ELRS receiver reports over CRSF, for Betaflight's
// OSD (link quality as RFMD:LQ, RSSI dBm, RSNR, TX power) and warnings.
void Flight::pushRcLinkStats() {
    if (!m_rcLink.config().enabled || !m_active) {
        if (m_rcPushed) bfb_rc_link_release();
        m_rcPushed = false;
        return;
    }
    const RcLinkStatus &st = m_rcLink.status();
    bfb_rc_link_stats((int)std::lround(st.lq), rcLinkMode(m_rcLink.config().mode).rfMode, (int)std::lround(st.rssiDbm),
                      (int)std::lround(st.snrDb), st.txPowerMw);
    m_rcPushed = true;
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
    {
        std::lock_guard<std::mutex> lock(m_portalMutex);
        if (m_portalsChanged) {
            m_portals = m_portalsIn;
            m_portalsChanged = false;
        }
    }
    // --- inputs for SimITL ----------------------------------------------
    updateRcLink();
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
#if defined(LIBFPV_KAMIKAZE)
    // the fuse is lit: the motors are cut, no last-moment thrust
    if (m_active && m_warhead.igniting())
        for (float &d : m_input.propDamage) d = 1.0f;
#endif

    // internal X right, Y up, Z forward = east, up, north
    bfb_gps_update(m_pos.x, m_pos.z, m_pos.y, m_vel.x, m_vel.z, m_vel.y);
    bfb_baro_update(m_pos.y);

    const bool drain = m_batteryDrain;
    const double charge = SimITL::Sim::getInstance().mSimState.batteryState.batCapacity;
    if (!m_damage) keepMotors();
    simitl_update(m_input);
    pushRcLinkStats();
    if (!drain) keepBattery(charge);
    if (!m_damage) keepMotors();
    // after the update: arming inside it may have just moved home
    {
        std::lock_guard<std::mutex> lock(m_homeMutex);
        if (m_hasHome) {
            bfb_gps_set_home(m_home.x, m_home.z, m_home.y);
        } else {
            bfb_gps_release_home();
        }
    }
    const StateOutput &out = SimITL::Sim::getInstance().getStateUpdate();

    // --- integrate pose with collisions -----------------------------------
#if defined(LIBFPV_KAMIKAZE)
    if (m_active && m_warhead.detonated()) {
        // the wreck stays where it exploded until repaired, its motors dead:
        // with the receiver gone (simulated RC link), Betaflight's failsafe
        // still holds the last sticks, and SimITL would spin them up
        m_vel = {};
        m_angVel = {};
        stopMotors();
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
            Vec3 before = m_pos;
            movePos(m_vel * h);
            if (!m_portals.empty()) passPortals(before);
            resolveContacts(h);
        }
        propStrikes(dt);
        if (hanging()) stepCargo(dt);
    }
#if defined(LIBFPV_KAMIKAZE)
    if (m_active && m_warhead.update(dt, m_simTime, m_input.rcData, bfb_is_armed(), m_pos, m_vel)) onDetonated();
    // the fuse is lit (an impact or the explode switch): the motors are cut
    // until the blast (no noise, no escape in the last milliseconds); it
    // still falls and bounces
    if (m_active && m_warhead.igniting()) stopMotors();
    // the warhead state is Betaflight's custom message 1 element: placed,
    // layered and hidden (OSD disable switch) like any other element
    char bombText[24] = "";
    if (m_active) m_warhead.statusText(bombText, sizeof bombText, m_simTime, bfb_is_armed());
    bfb_osd_set_host_message(0, bombText);
    // a live bomb: no crosshairs, no artificial horizon; libfpv draws the
    // aiming dot (fpv_drone_tick), unless the game draws the warhead's OSD
    WarheadState ws = m_warhead.snapshot().state;
    bfb_osd_set_aim_dot(m_active && m_warhead.config().osd && (ws == WarheadState::Armed || ws == WarheadState::Igniting));
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
    m_snap.teleports = m_teleports;
    m_snap.portalPasses = m_portalPasses;
    m_snap.lastPortal = m_lastPortal;
    m_snap.cargo = m_active && m_cargo.mass > 0;
    m_snap.cargoHanging = m_snap.cargo && hanging();
    m_snap.cargoLineTaut = m_snap.cargoHanging && m_lineTaut;
    if (m_snap.cargoHanging) {
        m_snap.cargoPos = m_cargoPos;
        m_snap.cargoVel = m_cargoVel;
    } else if (m_snap.cargo) {
        m_snap.cargoPos = m_pos + toMat3(m_rot) * m_cargo.offset;
        m_snap.cargoVel = m_vel + cross(m_angVel, m_snap.cargoPos - m_pos);
    }
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
        if ((out.motorStatus[i] & MotorStatus::MotorBurnedOut) && m_damage) {
            m.flags |= 1u;
            anyBurned = true;
        }
        if ((out.motorStatus[i] & MotorStatus::MotorDamaged && m_damage) || m_propDamage[i] > 0.05f) m.flags |= 2u;
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
    m_snap.beeperSwitch = m_snap.beeper && bfb_beeper_switch();
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
    m_snap.vtxDamage = m_vtxDamage;
    m_snap.rcLinkOn = m_active && m_rcLink.config().enabled;
    m_snap.rcLink = m_rcLink.status();
    if (m_active) {
        float t;
        Vec3 n;
        uint8_t mat;
        uint32_t obj;
        if ((m_videoTick & 7) == 0) {
            m_snap.altitude = m_world->raycast(m_pos, {0, -1, 0}, 1000.0f, m_simTime, t, n, mat, obj) ? t : -1.0f;
        }
        updateVideo(dt);
    }
#if defined(LIBFPV_KAMIKAZE)
    m_snap.warhead = m_warhead.snapshot();
    m_snap.payloadMass = m_payloadMass;
    m_snap.totalMass = m_preset.init.quadMass;
    if (m_active && m_warhead.detonated()) {
        // the video transmitter went with it: no signal
        m_snap.videoNoise = m_snap.videoBreakup = 1.0f;
        m_snap.videoSignal = 0.0f;
        m_snap.videoRxDbm = -200.0f;
        m_snap.crashed = true;
        // and it makes no sound: no motors, no buzzer
        for (MotorSnapshot &m : m_snap.motors) {
            m.rpm = m.thrust = m.current = m.command = 0;
        }
        m_snap.beeper = m_snap.beeperSwitch = false;
    } else if (m_active && m_warhead.igniting()) {
        // the motors are cut while the fuse burns: silent, no thrust
        for (MotorSnapshot &m : m_snap.motors) {
            m.rpm = m.thrust = m.current = m.command = 0;
        }
    }
#endif
}

}  // namespace fpv
