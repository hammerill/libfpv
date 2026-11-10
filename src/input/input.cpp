// libfpv - input manager.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "input/input.h"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "libfpv.h"
#include "nlohmann/json.hpp"
#include "util/fs.h"
#include "util/log.h"

namespace fpv {

using json = nlohmann::json;

bool nameLooksLikeRadio(const std::string &name) {
    std::string n = name;
    for (char &c : n) c = (char)std::tolower((unsigned char)c);
    static const char *kWords[] = {"edgetx",  "opentx",  "radiomaster", "jumper", "frsky",   "taranis",
                                   "tbs",     "tango",   "mambo",       "crossfire", "elrs", "expresslrs",
                                   "boxer",   "zorro",   "tx16",        "tx12",   "pocket",  "betafpv",
                                   "literadio", "flysky", "fs-i6",      "spektrum", "dongle", "simulator",
                                   "rc transmitter", "joystick (edgetx)", "radioking", "commando"};
    for (const char *w : kWords)
        if (n.find(w) != std::string::npos) return true;
    return false;
}

InputManager::InputManager() {
    setIdleChannels(m_host);
    setIdleChannels(m_lastOut);
}
InputManager::~InputManager() { stopDevices(); }

void InputManager::setDataDir(const std::string &dir) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_dataDir = dir;
    loadMappings();
}

bool InputManager::startDevices(std::string &error) {
#if defined(__EMSCRIPTEN__)
    auto backend = createWebBackend();
#else
    auto backend = createSdlBackend();
#endif
    if (!backend) {
        error = "device input not compiled in";
        return false;
    }
    if (!backend->init(error)) return false;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_backend = std::move(backend);
    return true;
}

void InputManager::stopDevices() {
    std::unique_ptr<DeviceBackend> b;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        b = std::move(m_backend);
        m_devices.clear();
    }
    if (b) b->shutdown();
}

void InputManager::setHostChannels(const float *ch, uint32_t count, double wallTime) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (uint32_t i = 0; i < kChannelCount; i++) m_host[i] = (i < count && std::isfinite(ch[i])) ? std::clamp(ch[i], -1.0f, 1.0f) : idleChannel((int)i);
    m_hostTime = wallTime;
}

void InputManager::clearHostChannels() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hostTime = -1e9;
}

void InputManager::key(int key, bool down) {
    if (key < 0 || key >= (int)m_keys.size()) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_keys[key] = down;
}

// --- persistence ---------------------------------------------------------

void InputManager::loadMappings() {
    m_mappings.clear();
    if (m_dataDir.empty()) return;
    std::string text;
    if (!readTextFile(pathFromUtf8(m_dataDir) / "radio_mappings.json", text)) return;
    try {
        json j = json::parse(text);
        for (auto &jm : j.at("devices")) {
            DeviceMapping m;
            m.guid = jm.value("guid", "");
            m.name = jm.value("name", "");
            m.complete = jm.value("complete", false);
            auto readBinding = [](const json &jb) {
                ChannelBinding b;
                b.axis = jb.value("axis", -1);
                b.button = jb.value("button", -1);
                b.inverted = jb.value("inverted", false);
                b.toggle = jb.value("toggle", false);
                b.min = jb.value("min", -1.0f);
                b.center = jb.value("center", 0.0f);
                b.max = jb.value("max", 1.0f);
                return b;
            };
            auto &chs = jm.at("channels");
            for (size_t i = 0; i < (size_t)kChannelCount && i < chs.size(); i++) m.channels[i] = readBinding(chs[i]);
            auto ctl = jm.find("controls");
            if (ctl != jm.end() && ctl->is_object())
                for (auto it = ctl->begin(); it != ctl->end(); ++it) m.controls[it.key()] = readBinding(it.value());
            if (!m.guid.empty()) m_mappings.push_back(m);
        }
    } catch (const std::exception &e) {
        FPV_LOGW("radio_mappings.json ignored: %s", e.what());
    }
}

void InputManager::saveMappings() {
    if (m_dataDir.empty()) return;
    json j;
    j["version"] = 1;
    j["devices"] = json::array();
    for (auto &m : m_mappings) {
        json jm;
        jm["guid"] = m.guid;
        jm["name"] = m.name;
        jm["complete"] = m.complete;
        auto binding = [](const ChannelBinding &b) {
            return json{{"axis", b.axis},     {"button", b.button}, {"inverted", b.inverted}, {"toggle", b.toggle},
                        {"min", b.min},       {"center", b.center}, {"max", b.max}};
        };
        jm["channels"] = json::array();
        for (auto &b : m.channels) jm["channels"].push_back(binding(b));
        if (!m.controls.empty()) {
            jm["controls"] = json::object();
            for (auto &kv : m.controls) jm["controls"][kv.first] = binding(kv.second);
        }
        j["devices"].push_back(jm);
    }
    writeFileAtomic(pathFromUtf8(m_dataDir) / "radio_mappings.json", j.dump(2));
}

bool InputManager::getMapping(const std::string &guid, DeviceMapping &out) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto &m : m_mappings) {
        if (m.guid == guid) {
            out = m;
            return true;
        }
    }
    return false;
}

void InputManager::setMapping(const DeviceMapping &mapping) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = std::find_if(m_mappings.begin(), m_mappings.end(), [&](auto &m) { return m.guid == mapping.guid; });
    if (it != m_mappings.end()) {
        *it = mapping;
    } else {
        m_mappings.push_back(mapping);
    }
    m_toggleGuid.clear();  // toggles start off with the new mapping
    saveMappings();
}

void InputManager::forgetMapping(const std::string &guid) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_mappings.erase(std::remove_if(m_mappings.begin(), m_mappings.end(), [&](auto &m) { return m.guid == guid; }),
                     m_mappings.end());
    saveMappings();
}

void InputManager::forgetAllMappings() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_mappings.clear();
    saveMappings();
}

// --- game controls ------------------------------------------------------------

void InputManager::setControls(const std::vector<ControlDesc> &controls) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<ControlState> state(controls.size());
    for (size_t i = 0; i < controls.size(); i++)
        for (size_t j = 0; j < m_controls.size(); j++)
            if (m_controls[j].id == controls[i].id && m_controls[j].name == controls[i].name) state[i] = m_controlState[j];
    m_controls = controls;
    m_controlState = std::move(state);
}

std::vector<ControlDesc> InputManager::controls() {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<ControlDesc> out = m_controls;
    for (ControlDesc &c : out) {
        auto it = m_controlKeys.find(c.name);
        if (it != m_controlKeys.end()) c.key = it->second;
    }
    return out;
}

void InputManager::setControlKey(const std::string &name, int key) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (key < 0) m_controlKeys.erase(name);  // back to the game's default
    else m_controlKeys[name] = key;
}

bool InputManager::pollControl(uint32_t id, uint32_t &presses) {
    std::lock_guard<std::mutex> lock(m_mutex);
    presses = 0;
    for (size_t i = 0; i < m_controls.size(); i++) {
        if (m_controls[i].id != id) continue;
        presses = m_controlState[i].presses;
        m_controlState[i].presses = 0;
        return m_controlState[i].on;
    }
    return false;
}

bool InputManager::controlOn(uint32_t id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (size_t i = 0; i < m_controls.size(); i++)
        if (m_controls[i].id == id) return m_controlState[i].on;
    return false;
}

// Called with m_mutex held, at the simulation rate, so a short tap of a
// button is never missed between two host frames. Every source counts at
// once: the radio's binding, the key and the gamepad button.
void InputManager::updateControls(const RawDevice *radio, const DeviceMapping *map) {
    uint32_t padButtons = 0;
    float padLT = 0, padRT = 0;
    for (const RawDevice &d : m_devices) {
        if (!d.isGamepad || &d == radio) continue;
        padButtons |= d.padButtons;
        padLT = std::max(padLT, d.pad[4]);
        padRT = std::max(padRT, d.pad[5]);
    }
    for (size_t i = 0; i < m_controls.size(); i++) {
        const ControlDesc &c = m_controls[i];
        ControlState &st = m_controlState[i];
        // the radio: a switch position (or a held button) is a level; a push
        // button standing in for a switch (toggle) is a press
        bool level = false, levelValid = false, push = false;
        if (radio && map) {
            auto it = map->controls.find(c.name);
            if (it != map->controls.end()) {
                const ChannelBinding &b = it->second;
                bool known = true, raw = false;
                if (b.axis >= 0 && b.axis < (int)radio->axes.size())
                    raw = switchPositionOf(*map, b, radio->axes[b.axis]) > 0;
                else if (b.button >= 0 && b.button < (int)radio->buttons.size())
                    raw = (radio->buttons[b.button] != 0) != b.inverted;
                else
                    known = false;
                if (known && b.toggle && c.kind == ControlKind::Switch) {
                    push = raw && !st.pushPrev;
                    st.pushPrev = raw;
                } else if (known) {
                    level = raw;
                    levelValid = true;
                }
            }
        }
        int key = c.key;
        auto ko = m_controlKeys.find(c.name);
        if (ko != m_controlKeys.end()) key = ko->second;
        bool keyDown = key > 0 && key < (int)m_keys.size() && m_keys[key];
        bool padDown = false;
        if (c.padButton == kPadLeftTrigger) padDown = padLT > 0.5f;
        else if (c.padButton == kPadRightTrigger) padDown = padRT > 0.5f;
        else if (c.padButton >= 0 && c.padButton < 32) padDown = (padButtons >> c.padButton) & 1u;

        bool before = st.on;
        if (c.kind == ControlKind::Button) {
            st.on = (levelValid && level) || keyDown || padDown;
        } else {
            if (levelValid && !st.radioValid) {
                before = level;  // the first reading of a switch is where it is, not a flip
                st.on = level;
            } else if (levelValid && level != st.radioPrev) {
                st.on = level;
            }
            if (push) st.on = !st.on;
            if (keyDown && !st.keyPrev) st.on = !st.on;
            if (padDown && !st.padPrev) st.on = !st.on;
        }
        st.radioValid = levelValid;
        st.radioPrev = level;
        st.keyPrev = keyDown;
        st.padPrev = padDown;
        if (st.on && !before) st.presses++;
    }
}

// --- mapping ---------------------------------------------------------------

static float applyCalibration(const ChannelBinding &b, float raw) {
    float v;
    if (raw >= b.center) {
        float span = b.max - b.center;
        v = span > 1e-4f ? (raw - b.center) / span : 0.0f;
    } else {
        float span = b.center - b.min;
        v = span > 1e-4f ? (raw - b.center) / span : 0.0f;
    }
    v = std::clamp(v, -1.0f, 1.0f);
    return b.inverted ? -v : v;
}

bool InputManager::mapRadio(const RawDevice &d, const DeviceMapping &m, float out[kChannelCount]) {
    if (m_toggleGuid != m.guid) {
        m_toggleGuid = m.guid;
        m_togglePressed.fill(false);
        m_toggleOn.fill(false);
    }
    for (int c = 0; c < kChannelCount; c++) {
        const ChannelBinding &b = m.channels[c];
        float v = (c == kThrottle || c >= kAux1) ? -1.0f : 0.0f;
        if (b.axis >= 0 && b.axis < (int)d.axes.size()) {
            v = c >= kAux1 ? switchPosition(m, c, d.axes[b.axis]) : applyCalibration(b, d.axes[b.axis]);
        } else if (b.button >= 0 && b.button < (int)d.buttons.size()) {
            bool pressed = d.buttons[b.button] != 0;
            v = (pressed != b.inverted) ? 1.0f : -1.0f;
        }
        if (b.toggle && c >= kAux1) {
            // a press (not the release) flips the switch
            bool pressed = v > 0;
            if (pressed && !m_togglePressed[c]) m_toggleOn[c] = !m_toggleOn[c];
            m_togglePressed[c] = pressed;
            v = m_toggleOn[c] ? 1.0f : -1.0f;
        }
        out[c] = v;
    }
    return true;
}

static float deadband(float v, float db) {
    if (std::fabs(v) < db) return 0;
    return (v - std::copysign(db, v)) / (1.0f - db);
}

// Gamepad (degraded): mode 2 sticks, throttle from the left stick where the
// centre is idle, A toggles arm, Y toggles turtle, ANGLE mode on (AUX2 high).
void InputManager::mapGamepad(const RawDevice &d, float dt, float out[kChannelCount]) {
    (void)dt;
    const float db = 0.08f;
    float thr = std::max(0.0f, deadband(-d.pad[1], db));
    out[kThrottle] = thr * 2.0f - 1.0f;
    out[kYaw] = deadband(d.pad[0], db);
    out[kRoll] = deadband(d.pad[2], db);
    out[kPitch] = deadband(-d.pad[3], db);
    bool armBtn = (d.padButtons & 1u) != 0;  // SDL_GAMEPAD_BUTTON_SOUTH
    if (armBtn && !m_padArmPrev) m_padArmed = !m_padArmed;
    m_padArmPrev = armBtn;
    bool turtleBtn = (d.padButtons & 8u) != 0;  // SDL_GAMEPAD_BUTTON_NORTH
    if (turtleBtn && !m_padTurtlePrev) m_padTurtle = !m_padTurtle;
    m_padTurtlePrev = turtleBtn;
    out[kAux1] = m_padArmed ? 1.0f : -1.0f;
    out[kAux2] = 1.0f;
    out[kAux3] = (d.padButtons & 2u) ? 1.0f : -1.0f;  // EAST = beeper
    out[kAux4] = m_padTurtle ? 1.0f : -1.0f;
#if defined(LIBFPV_KAMIKAZE)
    // WEST (X / Square) toggles the bomb arm switch, right shoulder explodes
    bool bombBtn = (d.padButtons & 4u) != 0;  // SDL_GAMEPAD_BUTTON_WEST
    if (bombBtn && !m_padBombPrev) m_padBomb = !m_padBomb;
    m_padBombPrev = bombBtn;
    out[kAux7] = m_padBomb ? 1.0f : -1.0f;
    out[kAux8] = (d.padButtons & (1u << 10)) ? 1.0f : -1.0f;  // SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER
#endif
}

// Keyboard hover hold (see mapKeyboard).
constexpr float kHoverSeed = 0.25f;    // first guess for a preset not flown yet (0.16..0.31)
constexpr float kHoverDamping = 3.0f;  // 1/s, wanted climb deceleration per m/s
constexpr float kHoverLearn = 6.0f;    // 1/s^2, how fast the hover throttle adapts
constexpr float kHoverLearnMaxClimb = 2.0f;  // m/s, limits the learning right after W/S
constexpr float kYawStart = 0.15f;  // keyboard yaw right after A/D, grows to full

void InputManager::setHoverFeedback(const HoverFeedback &f) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hover = f;
}

// Keyboard (degraded): A/D yaw (from a small rate, growing exponentially to
// full while held), arrows pitch/roll, Enter arms, T toggles turtle, ANGLE
// mode on. W/S move the throttle up and down, or, with the automatic
// hover throttle on (in auto: while no radio is detected), no key (or W
// and S together) hovers, S cuts the throttle, and W raises it
// exponentially up to full. The hover holds the climb rate at zero and
// learns, per preset, the throttle that does it, so it follows the quad,
// its battery sag and payload. On the ground, upside down or in turtle mode
// no key means idle, so arming never lifts off by itself.
void InputManager::mapKeyboard(float dt, bool radioDetected, float out[kChannelCount]) {
    auto k = [&](int code) { return m_keys[code]; };
    constexpr float g = 9.81f;
    float &hover = m_hoverThrottle.try_emplace(m_hover.preset, kHoverSeed).first->second;
    bool hovering = m_kbArmed && !m_kbTurtle && m_hover.airborne && m_hover.upright > 0.3f;
    bool up = k('W'), down = k('S');
    KeyboardHover mode = m_kbHover;
    bool hoverOn = mode == KeyboardHover::On || (mode == KeyboardHover::Auto && !radioDetected);
    if (!hoverOn) {
        const float rate = 0.6f;  // per second
        if (up) m_kbThrottle += rate * dt;
        if (down) m_kbThrottle -= rate * dt;
        m_kbThrottle = std::clamp(m_kbThrottle, 0.0f, 1.0f);
    } else if (up && !down) {
        // from at least the hover, so W climbs at once (also off the ground)
        float expo = std::max(0.0f, m_kbThrottleExpo.load());
        m_kbThrottle = std::min(1.0f, std::max(m_kbThrottle, hover) * std::exp(expo * dt));
    } else if (down && !up) {
        m_kbThrottle = 0;
    } else if (hovering) {
        // Thrust grows about with the throttle squared, so a relative
        // throttle change x changes the acceleration by about 2 g x.
        float climb = m_hover.climbRate;
        float tilt = 1.0f / std::sqrt(std::max(m_hover.upright, 0.5f));
        float thr = hover * tilt * (1.0f - kHoverDamping * climb / (2 * g));
        // no learning while the throttle is pinned (coasting up after W)
        if (thr > 0.0f && thr < 1.0f) {
            float learn = std::clamp(climb, -kHoverLearnMaxClimb, kHoverLearnMaxClimb);
            hover = std::clamp(hover * (1.0f - kHoverLearn * learn / (2 * g) * dt), 0.05f, 0.95f);
        }
        m_kbThrottle = std::clamp(thr, 0.0f, 1.0f);
    } else {
        m_kbThrottle = 0;
    }
    out[kThrottle] = m_kbThrottle * 2.0f - 1.0f;
    float yawDir = (k('D') ? 1.0f : 0.0f) - (k('A') ? 1.0f : 0.0f);
    if (yawDir == 0) {
        m_kbYaw = 0;
    } else if (m_kbYaw * yawDir <= 0) {
        m_kbYaw = yawDir * kYawStart;  // pressed, or reversed
    } else {
        float expo = std::max(0.0f, m_kbYawExpo.load());
        m_kbYaw = yawDir * std::min(1.0f, std::fabs(m_kbYaw) * std::exp(expo * dt));
    }
    out[kYaw] = m_kbYaw;
    out[kRoll] = (k(FPV_KEY_RIGHT) ? 0.6f : 0.0f) - (k(FPV_KEY_LEFT) ? 0.6f : 0.0f);
    out[kPitch] = (k(FPV_KEY_UP) ? 0.6f : 0.0f) - (k(FPV_KEY_DOWN) ? 0.6f : 0.0f);
    bool armKey = k(FPV_KEY_ENTER);
    if (armKey && !m_kbArmKeyPrev) {
        m_kbArmed = !m_kbArmed;
        if (m_kbArmed) m_kbThrottle = 0;
    }
    m_kbArmKeyPrev = armKey;
    bool turtleKey = k('T');
    if (turtleKey && !m_kbTurtleKeyPrev) m_kbTurtle = !m_kbTurtle;
    m_kbTurtleKeyPrev = turtleKey;
    out[kAux1] = m_kbArmed ? 1.0f : -1.0f;
    out[kAux2] = 1.0f;
    out[kAux3] = k('B') ? 1.0f : -1.0f;
    out[kAux4] = m_kbTurtle ? 1.0f : -1.0f;
#if defined(LIBFPV_KAMIKAZE)
    // K toggles the bomb arm switch, X held is the explode switch
    bool bombKey = k('K');
    if (bombKey && !m_kbBombKeyPrev) m_kbBomb = !m_kbBomb;
    m_kbBombKeyPrev = bombKey;
    out[kAux7] = m_kbBomb ? 1.0f : -1.0f;
    out[kAux8] = k('X') ? 1.0f : -1.0f;
#endif
}

void InputManager::update(double wallTime, float dt, float out[kChannelCount], bool &radioLost) {
    std::lock_guard<std::mutex> lock(m_mutex);
    radioLost = false;
    if (m_backend) m_backend->poll(m_devices);

    InputStatus st;
    st.deviceCount = (int)m_devices.size();
    float ch[kChannelCount];
    setIdleChannels(ch);

    const RawDevice *controlRadio = nullptr;  // the game controls' radio, if one flies
    const DeviceMapping *controlMap = nullptr;
    // Host channels win while fresh.
    if (wallTime - m_hostTime < 0.5) {
        std::copy(m_host, m_host + kChannelCount, ch);
        st.source = InputSource::Host;
        st.deviceName = "host";
    } else {
        // A mapped radio, then any unmapped radio (needs setup), then a gamepad.
        const RawDevice *radio = nullptr;
        const DeviceMapping *radioMap = nullptr;
        const RawDevice *unmapped = nullptr;
        const RawDevice *pad = nullptr;
        for (const RawDevice &d : m_devices) {
            const DeviceMapping *m = nullptr;
            for (auto &mm : m_mappings) {
                if (mm.guid == d.guid && mm.complete) m = &mm;
            }
            if (m || d.looksLikeRadio) st.radioDetected = true;
            if (m && !radio) {
                radio = &d;
                radioMap = m;
            } else if (!m && d.looksLikeRadio && !unmapped) {
                unmapped = &d;
            } else if (!m && d.isGamepad && !pad) {
                pad = &d;
            }
        }
        if (radio) {
            controlRadio = radio;
            controlMap = radioMap;
            mapRadio(*radio, *radioMap, ch);
            st.source = InputSource::Radio;
            st.deviceName = radio->name;
            m_activeGuid = radio->guid;
            m_radioWasActive = true;
        } else {
            if (m_radioWasActive) {
                // The radio that was flying disappeared: let Betaflight
                // failsafe, exactly like a real receiver losing link.
                radioLost = true;
                std::copy(m_lastOut, m_lastOut + kChannelCount, ch);
                st.source = InputSource::Radio;
                st.deviceName = "radio disconnected";
            } else if (pad) {
                mapGamepad(*pad, dt, ch);
                st.source = InputSource::Gamepad;
                st.deviceName = pad->name;
            } else if (m_keyboardEnabled) {
                mapKeyboard(dt, st.radioDetected, ch);
                st.source = InputSource::Keyboard;
                st.deviceName = "keyboard";
            }
            if (unmapped) {
                st.needsSetup = true;
                if (st.source != InputSource::Gamepad && st.source != InputSource::Radio) st.deviceName = unmapped->name;
            }
        }
    }
    if (m_wizardActive && st.source != InputSource::Host) {
        // hold idle and disarmed while the player wiggles sticks in the wizard
        ch[kRoll] = ch[kPitch] = ch[kYaw] = 0;
        ch[kThrottle] = -1;
        ch[kAux1] = -1;
        // presses while setting up do not flip toggles for the flight after
        m_toggleGuid.clear();
    }
    // the wizard's presses are not the game's
    if (!m_wizardActive) updateControls(controlRadio, controlMap);
    std::copy(ch, ch + kChannelCount, out);
    if (!radioLost) std::copy(ch, ch + kChannelCount, m_lastOut);
    m_status = st;
}

void InputManager::resetRadioLatch() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_radioWasActive = false;
}

InputStatus InputManager::status() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_status;
}

std::vector<RawDevice> InputManager::devicesSnapshot() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_devices;
}

}  // namespace fpv
