// libfpv - guided radio setup.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "overlay/radio_wizard.h"

#include <algorithm>
#include <cmath>

namespace fpv {

namespace {
constexpr float kMoveThreshold = 0.35f;  // raw units (-1..1) a stick must move to count
}

void RadioWizard::start(const RawDevice &dev) {
    m_active = true;
    m_bombOnly = false;
    m_controls.clear();
    m_controlIndex = 0;
    m_step = Step::Rest;
    m_mapping = DeviceMapping{};
    m_mapping.guid = dev.guid;
    m_mapping.name = dev.name;
    m_rest = dev.axes;
    m_restButtons = dev.buttons;
    m_min = dev.axes;
    m_max = dev.axes;
    m_candidate = -1;
    m_momentary = false;
    m_conflict = -1;
    m_replace = false;
    m_snapped = false;
    m_lastAxes.clear();
    m_lastButtons.clear();
    m_replaced.clear();
}

void RadioWizard::setMomentary(bool on) {
    m_momentary = on;
    m_candidate = -1;
}

#if defined(LIBFPV_KAMIKAZE)
void RadioWizard::startBombSwitches(const RawDevice &dev, const DeviceMapping &current) {
    start(dev);
    m_bombOnly = true;
    m_mapping = current;
    m_mapping.guid = dev.guid;
    m_mapping.name = dev.name;
    m_mapping.channels[kAux7] = ChannelBinding{};
    m_mapping.channels[kAux8] = ChannelBinding{};
}

void RadioWizard::mergeBombSwitches(DeviceMapping &into, const DeviceMapping &from, bool onlyIfFree) {
    for (int c : {kAux7, kAux8}) {
        const ChannelBinding &b = from.channels[c];
        into.channels[c] = ChannelBinding{};
        if (b.axis < 0 && b.button < 0) continue;
        bool free = true;
        if (onlyIfFree) {
            for (int o = 0; o < kChannelCount; o++) {
                const ChannelBinding &ob = into.channels[o];
                if (o == c) continue;
                if (b.button >= 0 && ob.button == b.button) free = false;
                // a stick axis, or a switch in the same position
                if (b.axis >= 0 && ob.axis == b.axis &&
                    (o < kAux1 || std::fabs(ob.switchOn() - b.switchOn()) < kMoveThreshold))
                    free = false;
            }
        }
        if (free) into.channels[c] = b;
    }
}
#endif

void RadioWizard::startControls(const RawDevice &dev, const DeviceMapping &current,
                                const std::vector<ControlDesc> &controls) {
    start(dev);
    m_mapping = current;
    m_mapping.guid = dev.guid;
    m_mapping.name = dev.name;
    m_controls = controls;
    for (const ControlDesc &c : m_controls) m_mapping.controls.erase(c.name);
}

const ControlDesc *RadioWizard::currentControl() const {
    if (m_step != Step::GameControl || m_controlIndex >= m_controls.size()) return nullptr;
    return &m_controls[m_controlIndex];
}

void RadioWizard::enterControl() {
    m_candidate = -1;
    const ControlDesc &c = m_controls[m_controlIndex];
    m_controlTitle = c.label;
    // a button usually is one on the radio: it is kept after the release
    m_momentary = c.kind != ControlKind::Switch;
    if (c.kind == ControlKind::Instant)
        m_controlText = "Optional: press the button (or flip the switch) that should do \"" + c.label +
                        "\", or Skip. It happens once per press. On a switch (untick the box below) it happens on "
                        "every flip, to ON and back to OFF, so you never flip it back for nothing. A position of a "
                        "3-position switch that no other mode uses works too.";
    else if (c.kind == ControlKind::Hold)
        m_controlText = "Optional: press the button (or flip the switch) that should do \"" + c.label +
                        "\", or Skip. It acts while held; a momentary switch or a push button works best. A "
                        "position of a 3-position switch that no other mode uses works too.";
    else
        m_controlText = "Optional: flip the switch that should turn \"" + c.label +
                        "\" on to its ON position, or Skip. A push button works too: tick the box below, and "
                        "each press turns it on or off.";
}

void RadioWizard::mergeControls(DeviceMapping &into, const DeviceMapping &from, const std::vector<std::string> &names,
                                bool onlyIfFree) {
    for (const std::string &name : names) {
        into.controls.erase(name);
        auto it = from.controls.find(name);
        if (it == from.controls.end()) continue;
        const ChannelBinding &b = it->second;
        if (b.axis < 0 && b.button < 0) continue;
        bool free = true;
        if (onlyIfFree) {
            auto clash = [&](const ChannelBinding &ob, bool stick) {
                if (b.button >= 0 && ob.button == b.button) return true;
                return b.axis >= 0 && ob.axis == b.axis &&
                       (stick || std::fabs(ob.switchOn() - b.switchOn()) < kMoveThreshold);
            };
            for (int o = 0; o < kChannelCount; o++)
                if (clash(into.channels[o], o < kAux1)) free = false;
            for (auto &kv : into.controls)
                if (clash(kv.second, false)) free = false;
        }
        if (free) into.controls[name] = b;
    }
}

std::vector<std::string> RadioWizard::controlNames(const DeviceMapping &m) {
    std::vector<std::string> out;
    for (auto &kv : m.controls) out.push_back(kv.first);
    return out;
}

int RadioWizard::channelForStep(Step s) const {
    switch (s) {
    case Step::Throttle: return kThrottle;
    case Step::Yaw: return kYaw;
    case Step::Pitch: return kPitch;
    case Step::Roll: return kRoll;
    case Step::ArmSwitch: return kAux1;
    case Step::AngleSwitch: return kAux2;
    case Step::BeeperSwitch: return kAux3;
    case Step::TurtleSwitch: return kAux4;
    case Step::AirModeSwitch: return kAux5;
    case Step::OsdSwitch: return kAux6;
#if defined(LIBFPV_KAMIKAZE)
    case Step::BombArmSwitch: return kAux7;
    case Step::ExplodeSwitch: return kAux8;
#endif
    default: return -1;
    }
}

RadioWizard::Step RadioWizard::lastSwitch() const {
#if defined(LIBFPV_KAMIKAZE)
    if (m_bombOnly) return Step::ExplodeSwitch;
#endif
    return Step::OsdSwitch;
}

bool RadioWizard::axisUsed(int axis) const {
    for (auto &b : m_mapping.channels)
        if (b.axis == axis) return true;
    for (auto &kv : m_mapping.controls)
        if (kv.second.axis == axis) return true;
    return false;
}

bool RadioWizard::stickAxisUsed(int axis) const {
    for (int c = kRoll; c <= kYaw; c++)
        if (m_mapping.channels[c].axis == axis) return true;
    return false;
}

// A switch may share an axis with switches mapped earlier (a 3-position
// switch: beeper in the middle, turtle at the top), but only in a position
// none of them uses.
bool RadioWizard::positionFree(int axis, float value) const {
    for (int c = kAux1; c < kChannelCount; c++) {
        const ChannelBinding &b = m_mapping.channels[c];
        if (b.axis == axis && std::fabs(value - b.switchOn()) < kMoveThreshold) return false;
    }
    for (auto &kv : m_mapping.controls)
        if (kv.second.axis == axis && std::fabs(value - kv.second.switchOn()) < kMoveThreshold) return false;
    return true;
}

int RadioWizard::sharedWith(int axis) const {
    for (int c = kAux1; c < kChannelCount; c++)
        if (m_mapping.channels[c].axis == axis) return c;
    return -1;
}

bool RadioWizard::buttonUsed(int button) const {
    for (auto &b : m_mapping.channels)
        if (b.button == button) return true;
    for (auto &kv : m_mapping.controls)
        if (kv.second.button == button) return true;
    return false;
}

// The axis (or button) that moved furthest from its rest position and is
// not assigned yet. Switches may reuse a switch axis in a new position.
// On a switch step, a switch position or button that is bound already but
// moved in this step is remembered as a conflict (the UI offers it).
void RadioWizard::detectAxis(const RawDevice &dev, bool allowButtons) {
    m_candidate = -1;
    float best = kMoveThreshold;
    int conflict = -1;
    float conflictValue = 0, conflictBest = kMoveThreshold;
    for (size_t i = 0; i < dev.axes.size() && i < m_rest.size(); i++) {
        if (allowButtons ? stickAxisUsed((int)i) : axisUsed((int)i)) continue;
        float d = std::fabs(dev.axes[i] - m_rest[i]);
        if (allowButtons && !positionFree((int)i, dev.axes[i])) {
            bool movedNow = i < m_axisMoved.size() && m_axisMoved[i];
            std::vector<int> chs;
            std::vector<std::string> ctls;
            if (movedNow && d > conflictBest) {
                ownersOf((int)i, dev.axes[i], chs, ctls);
                if (!chs.empty() || !ctls.empty()) {
                    conflictBest = d;
                    conflict = (int)i;
                    conflictValue = dev.axes[i];
                }
            }
            continue;
        }
        if (d > best) {
            best = d;
            m_candidate = (int)i;
            m_candidateValue = dev.axes[i];
        }
    }
    if (m_candidate < 0 && allowButtons) {
        for (size_t i = 0; i < dev.buttons.size() && i < m_restButtons.size(); i++) {
            if (dev.buttons[i] == m_restButtons[i]) continue;
            if (buttonUsed((int)i)) {
                bool movedNow = i < m_buttonMoved.size() && m_buttonMoved[i];
                std::vector<int> chs;
                std::vector<std::string> ctls;
                if (movedNow && conflict < 0) {
                    ownersOf(1000 + (int)i, 0, chs, ctls);
                    if (!chs.empty() || !ctls.empty()) {
                        conflict = 1000 + (int)i;
                        conflictValue = dev.buttons[i] ? 1.0f : 0.0f;
                    }
                }
                continue;
            }
            m_candidate = 1000 + (int)i;
            m_candidateValue = dev.buttons[i] ? 1.0f : 0.0f;
            break;
        }
    }
    if (m_candidate >= 0) {
        m_conflict = -1;
    } else if (conflict >= 0) {
        m_conflict = conflict;
        m_conflictValue = conflictValue;
    }
}

bool RadioWizard::owns(const ChannelBinding &b, int input, float value) {
    if (input >= 1000) return b.button == input - 1000;
    return input >= 0 && b.axis == input && std::fabs(value - b.switchOn()) < kMoveThreshold;
}

void RadioWizard::ownersOf(int input, float value, std::vector<int> &channels,
                           std::vector<std::string> &controls) const {
    int self = channelForStep(m_step);
    const ControlDesc *current = currentControl();
    for (int c = kAux1; c < kChannelCount; c++)
        if (c != self && owns(m_mapping.channels[c], input, value)) channels.push_back(c);
    for (auto &kv : m_mapping.controls)
        if ((!current || kv.first != current->name) && owns(kv.second, input, value)) controls.push_back(kv.first);
}

namespace {

const char *auxName(int c) {
    switch (c) {
    case kAux1: return "AUX1 (arm)";
    case kAux2: return "AUX2 (angle)";
    case kAux3: return "AUX3 (beeper)";
    case kAux4: return "AUX4 (turtle)";
    case kAux5: return "AUX5 (air mode)";
    case kAux6: return "AUX6 (OSD disable)";
#if defined(LIBFPV_KAMIKAZE)
    case kAux7: return "AUX7 (bomb arm)";
    case kAux8: return "AUX8 (explode)";
#endif
    default: return "an AUX channel";
    }
}

}  // namespace

std::string RadioWizard::conflictText(const std::vector<ControlDesc> &known) const {
    if (!hasConflict()) return "";
    std::vector<int> chs;
    std::vector<std::string> ctls;
    ownersOf(m_conflict, m_conflictValue, chs, ctls);
    std::vector<std::string> names;
    for (int c : chs) names.push_back(auxName(c));
    for (const std::string &n : ctls) {
        std::string label = n;
        for (const ControlDesc &d : known)
            if (d.name == n && !d.label.empty()) label = d.label;
        for (const ControlDesc &d : m_controls)
            if (d.name == n && !d.label.empty()) label = d.label;
        names.push_back("\"" + label + "\"");
    }
    std::string who;
    for (size_t i = 0; i < names.size(); i++) who += (i == 0 ? "" : i + 1 == names.size() ? " and " : ", ") + names[i];
    std::string what = m_conflict >= 1000 ? "Button " + std::to_string(m_conflict - 1000 + 1)
                                          : "That position of axis " + std::to_string(m_conflict + 1);
    return what + " is already used by " + who + ".";
}

bool RadioWizard::takeConflict(bool replace, std::string &hint) {
    if (!hasConflict()) return false;
    m_candidate = m_conflict;
    m_candidateValue = m_conflictValue;
    m_replace = replace;
    bool ok = next(hint);
    m_replace = false;
    return ok;
}

void RadioWizard::applyReplaced(DeviceMapping &into) const {
    auto same = [](const ChannelBinding &a, const ChannelBinding &b) {
        return a.axis == b.axis && a.button == b.button && std::fabs(a.switchOn() - b.switchOn()) < 1e-3f;
    };
    for (const Replaced &r : m_replaced) {
        if (r.channel >= 0) {
            if (same(into.channels[r.channel], r.binding)) into.channels[r.channel] = ChannelBinding{};
        } else {
            auto it = into.controls.find(r.control);
            if (it != into.controls.end() && same(it->second, r.binding)) into.controls.erase(it);
        }
    }
}

// The step just went back to: what its binding replaced is bound again.
void RadioWizard::restoreReplaced() {
    for (auto it = m_replaced.begin(); it != m_replaced.end();) {
        if (it->step == m_step && (m_step != Step::GameControl || it->controlIndex == m_controlIndex)) {
            if (it->channel >= 0) m_mapping.channels[it->channel] = it->binding;
            else m_mapping.controls[it->control] = it->binding;
            it = m_replaced.erase(it);
        } else {
            ++it;
        }
    }
}

void RadioWizard::update(const RawDevice &dev) {
    if (!m_active) return;
    // a new step: its baseline is the state when it began (Next pressed)
    if (!m_snapped || m_snapStep != m_step || m_snapIndex != m_controlIndex) {
        m_snapped = true;
        m_snapStep = m_step;
        m_snapIndex = m_controlIndex;
        m_stepAxes = m_lastAxes.empty() ? dev.axes : m_lastAxes;
        m_stepButtons = m_lastAxes.empty() ? dev.buttons : m_lastButtons;
        m_conflict = -1;
        m_axisMoved.assign(m_stepAxes.size(), 0);
        m_buttonMoved.assign(m_stepButtons.size(), 0);
    }
    for (size_t i = 0; i < dev.axes.size() && i < m_stepAxes.size(); i++)
        if (std::fabs(dev.axes[i] - m_stepAxes[i]) > kMoveThreshold) m_axisMoved[i] = 1;
    for (size_t i = 0; i < dev.buttons.size() && i < m_stepButtons.size(); i++)
        if (dev.buttons[i] != m_stepButtons[i]) m_buttonMoved[i] = 1;
    m_lastAxes = dev.axes;
    m_lastButtons = dev.buttons;
    if (m_step == Step::Rest) {
        m_rest = dev.axes;
        m_restButtons = dev.buttons;
        m_min = m_max = dev.axes;
        return;
    }
    if (m_step == Step::Calibrate) {
        for (size_t i = 0; i < dev.axes.size() && i < m_min.size(); i++) {
            m_min[i] = std::min(m_min[i], dev.axes[i]);
            m_max[i] = std::max(m_max[i], dev.axes[i]);
        }
        return;
    }
    if (channelForStep(m_step) >= 0 || m_step == Step::GameControl) {
        bool sw = m_step >= Step::ArmSwitch;
        int prev = m_candidate;
        float prevValue = m_candidateValue;
        detectAxis(dev, sw);
        if (sw && momentary() && prev >= 0) {
            // a button springs back: keep what was pressed, at its furthest
            if (m_candidate < 0 ||
                (m_candidate == prev && prev < 1000 &&
                 std::fabs(prevValue - m_rest[prev]) > std::fabs(m_candidateValue - m_rest[prev]))) {
                m_candidate = prev;
                m_candidateValue = prevValue;
            }
        }
        // keep endpoints up to date for sticks while they are moved
        for (size_t i = 0; i < dev.axes.size() && i < m_min.size(); i++) {
            m_min[i] = std::min(m_min[i], dev.axes[i]);
            m_max[i] = std::max(m_max[i], dev.axes[i]);
        }
    }
}

bool RadioWizard::next(std::string &hint) {
    hint.clear();
    switch (m_step) {
    case Step::PickDevice:
        return false;
    case Step::Rest:
        if (m_rest.size() < 4) {
            hint = "This device has fewer than 4 axes; it cannot be used as a radio.";
            return false;
        }
#if defined(LIBFPV_KAMIKAZE)
        if (m_bombOnly) {
            m_step = Step::BombArmSwitch;
            m_candidate = -1;
            return true;
        }
#endif
        if (controlsOnly()) {
            m_step = Step::GameControl;
            m_controlIndex = 0;
            enterControl();
            return true;
        }
        m_step = Step::Throttle;
        return true;
    case Step::Calibrate: {
        // Sticks: endpoints from the calibration sweep; centre from rest
        // (throttle has no centre: use the middle of its travel).
        for (int c : {kRoll, kPitch, kYaw, kThrottle}) {
            ChannelBinding &b = m_mapping.channels[c];
            if (b.axis < 0) continue;
            float lo = m_min[b.axis], hi = m_max[b.axis];
            if (hi - lo < 0.5f) {
                hint = "Move every stick all the way to its corners, then press Next.";
                return false;
            }
            b.min = lo;
            b.max = hi;
            b.center = (c == kThrottle) ? 0.5f * (lo + hi) : m_rest[b.axis];
        }
        m_step = Step::ArmSwitch;
        m_candidate = -1;
        return true;
    }
    case Step::Done:
        return false;
    default:
        break;
    }
    int ch = channelForStep(m_step);
    if (ch < 0 && m_step != Step::GameControl) return false;
    if (m_candidate < 0) {
        hint = explodeStep()       ? "Press the explode button (or flip its switch) first: the wizard has not seen it yet."
               : m_step >= Step::ArmSwitch ? (m_momentary ? "Press the button first (the wizard has not seen it yet)."
                                                        : "Flip the switch first (the wizard has not seen it move yet).")
                                         : "Move the stick as shown and hold it there.";
        return false;
    }
    const ControlDesc *control = currentControl();
    if (m_replace) {
        // the others that use this switch position or button lose it
        std::vector<int> chs;
        std::vector<std::string> ctls;
        ownersOf(m_candidate, m_candidateValue, chs, ctls);
        for (int c : chs) {
            m_replaced.push_back({m_step, m_controlIndex, c, "", m_mapping.channels[c]});
            m_mapping.channels[c] = ChannelBinding{};
        }
        for (const std::string &n : ctls) {
            m_replaced.push_back({m_step, m_controlIndex, -1, n, m_mapping.controls[n]});
            m_mapping.controls.erase(n);
        }
        m_replace = false;
    }
    m_conflict = -1;
    ChannelBinding &b = control ? m_mapping.controls[control->name] : m_mapping.channels[ch];
    b = ChannelBinding{};
#if defined(LIBFPV_KAMIKAZE)
    b.toggle = m_momentary && m_step != Step::ExplodeSwitch;
#else
    b.toggle = m_momentary;
#endif
    // an instant or hold control on a push button is just that: no toggling;
    // on a switch, an instant one acts on each flip
    if (control && control->kind != ControlKind::Switch) {
        b.toggle = false;
        b.isSwitch = !m_momentary;
    }
    if (m_candidate >= 1000) {
        b.button = m_candidate - 1000;
        // the switch is "on" in its current (flipped) position
        b.inverted = m_candidateValue < 0.5f;
    } else {
        b.axis = m_candidate;
        float rest = m_rest[b.axis];
        bool positive = m_candidateValue > rest;
        if (m_step >= Step::ArmSwitch) {
            // switch: "on" where it is now; endpoints are rest and now
            b.inverted = !positive;
            b.min = std::min(rest, m_candidateValue);
            b.max = std::max(rest, m_candidateValue);
            b.center = 0.5f * (b.min + b.max);
        } else {
            // stick: requested direction (up/right) must read +1
            b.inverted = !positive;
            b.min = -1;
            b.max = 1;
            b.center = rest;
        }
    }
    m_candidate = -1;
    m_momentary = false;
    if (control) {
        if (++m_controlIndex < m_controls.size()) {
            enterControl();
        } else {
            m_step = Step::Done;  // `complete` stays as the radio's setup had it
        }
        return true;
    }
    switch (m_step) {
    case Step::Throttle: m_step = Step::Yaw; break;
    case Step::Yaw: m_step = Step::Pitch; break;
    case Step::Pitch: m_step = Step::Roll; break;
    case Step::Roll: m_step = Step::Calibrate; break;
    case Step::ArmSwitch: m_step = Step::AngleSwitch; break;
    case Step::AngleSwitch: m_step = Step::BeeperSwitch; break;
    case Step::BeeperSwitch: m_step = Step::TurtleSwitch; break;
    case Step::TurtleSwitch: m_step = Step::AirModeSwitch; break;
    case Step::AirModeSwitch: m_step = Step::OsdSwitch; break;
    default:
        if (m_step == lastSwitch()) {
            m_step = Step::Done;
            m_mapping.complete = true;
        } else if (m_step > Step::ArmSwitch && m_step < lastSwitch()) {
            m_step = (Step)((int)m_step + 1);
        }
        break;
    }
    return true;
}

void RadioWizard::skip() {
    if (!canSkip()) return;
    if (const ControlDesc *c = currentControl()) {
        m_mapping.controls.erase(c->name);
        m_candidate = -1;
        m_momentary = false;
        if (++m_controlIndex < m_controls.size()) enterControl();
        else m_step = Step::Done;
        return;
    }
    m_mapping.channels[channelForStep(m_step)] = ChannelBinding{};
    m_candidate = -1;
    m_momentary = false;
    if (m_step == lastSwitch()) {
        m_step = Step::Done;
        m_mapping.complete = true;
    } else {
        m_step = (Step)((int)m_step + 1);
    }
}

void RadioWizard::back() {
    if (m_step <= Step::Rest) return;
    if (controlsOnly()) {
        // Done -> the last control, a control -> the one before, the first -> Rest
        if (m_step == Step::Done) m_controlIndex = m_controls.size();
        if (m_controlIndex == 0) {
            m_step = Step::Rest;
            m_candidate = -1;
            return;
        }
        m_controlIndex--;
        m_step = Step::GameControl;
        m_mapping.controls.erase(m_controls[m_controlIndex].name);
        restoreReplaced();
        enterControl();
        return;
    }
#if defined(LIBFPV_KAMIKAZE)
    if (m_bombOnly && m_step == Step::BombArmSwitch) {
        m_step = Step::Rest;
        m_candidate = -1;
        return;
    }
#endif
    m_step = m_step == Step::Done ? lastSwitch() : (Step)((int)m_step - 1);
    int ch = channelForStep(m_step);
    if (ch >= 0) m_mapping.channels[ch] = ChannelBinding{};
    restoreReplaced();
    m_candidate = -1;
    m_momentary = false;
    m_mapping.complete = false;
}

const char *RadioWizard::title() const {
    switch (m_step) {
    case Step::PickDevice: return "Choose your radio";
    case Step::Rest: return "Starting position";
    case Step::Throttle: return "Throttle";
    case Step::Yaw: return "Yaw";
    case Step::Pitch: return "Pitch";
    case Step::Roll: return "Roll";
    case Step::Calibrate: return "Stick endpoints";
    case Step::ArmSwitch: return "Arm switch";
    case Step::AngleSwitch: return "Angle (self-level) switch";
    case Step::BeeperSwitch: return "Beeper switch";
    case Step::TurtleSwitch: return "Turtle (crash flip) switch";
    case Step::AirModeSwitch: return "Air mode switch";
    case Step::OsdSwitch: return "OSD disable switch";
#if defined(LIBFPV_KAMIKAZE)
    case Step::BombArmSwitch: return "Bomb arm switch";
    case Step::ExplodeSwitch: return "Explode switch";
#endif
    case Step::GameControl: return m_controlTitle.c_str();
    case Step::Done: return "All set";
    }
    return "";
}

const char *RadioWizard::instruction() const {
    switch (m_step) {
    case Step::PickDevice: return "Select the radio you want to fly with.";
    case Step::Rest:
        if (controlsOnly())
            return "Put every switch in its OFF position, then press Next. Only the game's own controls are set up "
                   "here; the rest of your radio setup stays as it is.";
#if defined(LIBFPV_KAMIKAZE)
        if (m_bombOnly)
            return "Put every switch in its OFF position (the bomb switches too), then press Next. Only the bomb arm "
                   "and explode switches are set up here; the rest of your radio setup stays as it is.";
#endif
        return "Centre the right stick and the yaw, pull the throttle all the way down, and put every switch in "
               "its OFF position. Then press Next.";
    case Step::Throttle: return "Push the THROTTLE stick all the way UP and hold it there.";
    case Step::Yaw: return "Centre the throttle-side stick, then push YAW all the way to the RIGHT and hold it.";
    case Step::Pitch: return "Push PITCH all the way UP (forward) and hold it.";
    case Step::Roll: return "Push ROLL all the way to the RIGHT and hold it.";
    case Step::Calibrate:
        return "Move both sticks around their full range a few times (corners included), then centre them and press "
               "Next.";
    case Step::ArmSwitch: return "Flip the switch you want to ARM with to its ARMED position.";
    case Step::AngleSwitch:
        return "Optional: flip a switch to the position that should turn on ANGLE (self-level) mode, or Skip.";
    case Step::BeeperSwitch:
        return "Optional: flip a switch for the BEEPER (find a crashed drone by sound), or Skip. A 3-position switch "
               "can carry two modes: use one position here and another one for the next switch.";
    case Step::TurtleSwitch:
        return "Optional: flip a switch to the position that should turn on TURTLE mode (flip over after crash), or "
               "Skip. It can be another position of the beeper's switch. To use it upside down: switch on, arm, hold "
               "the stick towards the side to roll over, then switch it off to fly away.";
    case Step::AirModeSwitch:
        return "Optional: flip a switch to the position that should force AIR MODE on (full stick authority at zero "
               "throttle), or Skip. It only matters when the AIRMODE feature is off, as on quads that keep air mode "
               "on a switch. A 3-position switch can carry this and the next switch.";
    case Step::OsdSwitch:
        return "Optional: flip a switch to the position that should DISABLE THE OSD (hide everything drawn over the "
               "video), or Skip. It can be another position of the air mode switch.";
#if defined(LIBFPV_KAMIKAZE)
    case Step::BombArmSwitch:
        return "Flip the switch that should ARM THE BOMB to its armed position. Use a switch of its own, away from "
               "the arm switch. The motors only arm with it off: take off, then flip it; the warhead is live after "
               "the arming delay and explodes on impact. Skip it and the bomb can never be armed.";
    case Step::ExplodeSwitch:
        return "Optional: flip a switch (a momentary one is best) that should EXPLODE the bomb on command, or Skip. "
               "It only works while the bomb is armed and live.";
#endif
    case Step::GameControl: return m_controlText.c_str();
    case Step::Done:
        if (controlsOnly())
            return "The game's controls are set; the rest of your radio setup is unchanged. Try them below.";
#if defined(LIBFPV_KAMIKAZE)
        if (m_bombOnly)
            return "The bomb switches are set; the rest of your radio setup is unchanged. Check the bars below.";
#endif
        return "Your radio is mapped. Check the bars below, then close this window and fly. Throttle down and flip "
               "the arm switch to arm.";
    }
    return "";
}

std::string RadioWizard::detectedLabel() const {
    if (m_candidate < 0) return "";
    if (m_candidate >= 1000) return "button " + std::to_string(m_candidate - 1000 + 1);
    if (m_momentary) return "axis " + std::to_string(m_candidate + 1) + ", button";
    float rest = m_rest[m_candidate];
    std::string label = "axis " + std::to_string(m_candidate + 1);
    if (m_step >= Step::ArmSwitch) {
        int other = sharedWith(m_candidate);
        if (other < 0 && axisUsed(m_candidate)) return label + ", same switch as another control in another position";
        if (other >= 0) label += ", same switch as AUX" + std::to_string(other - kAux1 + 1) + " in another position";
        return label;
    }
    return label + (m_candidateValue > rest ? "" : ", reversed");
}

float RadioWizard::progress() const {
    if (m_step == Step::Done) return 1;
    if (controlsOnly())
        return m_step == Step::Rest ? 0.0f : (float)(m_controlIndex + 1) / (float)(m_controls.size() + 1);
#if defined(LIBFPV_KAMIKAZE)
    if (m_bombOnly) return m_step == Step::Rest ? 0.0f : m_step == Step::BombArmSwitch ? 1 / 3.0f : 2 / 3.0f;
#endif
    return (float)m_step / (float)((int)lastSwitch() + 1);
}

}  // namespace fpv
