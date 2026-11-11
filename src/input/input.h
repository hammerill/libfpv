// libfpv - input manager: radios, gamepads, keyboard fallback and host
// channels, mapped to Betaflight's AETR + AUX1..6 (AUX1..8 in kamikaze builds).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace fpv {

enum class InputSource { None = 0, Radio = 1, Gamepad = 2, Keyboard = 3, Host = 4 };

// Channel indices as Betaflight sees them (default map AETR1234).
enum Channel { kRoll = 0, kPitch = 1, kThrottle = 2, kYaw = 3, kAux1 = 4, kAux2 = 5, kAux3 = 6, kAux4 = 7, kAux5 = 8, kAux6 = 9 };
#if defined(LIBFPV_KAMIKAZE)
// kamikaze builds: bomb arm switch on AUX7, explode switch on AUX8
constexpr int kAux7 = 10, kAux8 = 11;
constexpr int kChannelCount = 12;
#else
constexpr int kChannelCount = 10;
#endif

// Channel value at rest: sticks centred, throttle idle, every switch off.
constexpr float idleChannel(int c) { return (c == kThrottle || c >= kAux1) ? -1.0f : 0.0f; }
inline void setIdleChannels(float *ch) {
    for (int c = 0; c < kChannelCount; c++) ch[c] = idleChannel(c);
}

// Mapping of one raw device axis/button to a channel.
struct ChannelBinding {
    int axis = -1;          // raw axis index, -1 = none
    bool inverted = false;
    float min = -1, center = 0, max = 1;  // calibrated raw endpoints
    // switches can also be bound to buttons: value = +1 while pressed
    int button = -1;
    // a push button (or momentary switch) that stands in for a switch: each
    // press flips the channel on or off
    bool toggle = false;
    // A game control's radio switch, not a button (the setup's "It's a
    // button" box unticked): an instant control acts on each flip of it,
    // to ON and back to OFF.
    bool isSwitch = false;
    // AUX switches on an axis are positions: "on" at one raw value, "off" at
    // rest. Several switches can share one multi-position switch.
    float switchOn() const { return inverted ? min : max; }
    float switchOff() const { return inverted ? max : min; }
};

struct DeviceMapping {
    std::string guid;
    std::string name;
    std::array<ChannelBinding, kChannelCount> channels{};
    // The game's own controls (fpv_controls_set) on this radio, by the
    // control's name. Every game's are kept: the mappings file is shared.
    std::map<std::string, ChannelBinding> controls;
    bool complete = false;
};

// A switch binding `b` of mapping `m` (an AUX channel or a game control)
// for the raw axis value: +1 when the switch is nearer its "on" position
// than half the way to any other known position of that axis (rest, or
// another switch's "on"), else -1.
// How far the "on" position of switch binding `b` is from the nearest other
// known position of its axis (rest, or another switch's "on").
inline float switchSpacing(const DeviceMapping &m, const ChannelBinding &b) {
    float on = b.switchOn();
    float nearest = std::fabs(b.switchOff() - on);
    auto other = [&](const ChannelBinding &ob) {
        if (&ob == &b || ob.axis != b.axis) return;
        float d = std::fabs(ob.switchOn() - on);
        if (d > 1e-3f) nearest = std::min(nearest, d);
    };
    for (int o = kAux1; o < kChannelCount; o++) other(m.channels[o]);
    for (auto &kv : m.controls) other(kv.second);
    return nearest;
}

inline float switchPositionOf(const DeviceMapping &m, const ChannelBinding &b, float raw) {
    return std::fabs(raw - b.switchOn()) < 0.5f * switchSpacing(m, b) ? 1.0f : -1.0f;
}

// AUX channel `c` (bound to an axis), see switchPositionOf.
inline float switchPosition(const DeviceMapping &m, int c, float raw) { return switchPositionOf(m, m.channels[c], raw); }

// --- game controls (fpv_controls_set) ------------------------------------------

// fpv_control_kind. An instant action happens once per press (on a radio
// switch: per flip, either way); a switch is on or off, and a key, gamepad
// button or radio push button flips it; a hold control acts while held.
enum class ControlKind { Instant = 0, Switch = 1, Hold = 2 };

// A game control's radio switch has thresholds with a gap between them: it
// turns on within kSwitchOnShare of the spacing to the nearest other position
// and off only beyond kSwitchOffShare, so jitter around the middle never
// flips it. Then a change must hold kControlDebounce seconds: one noisy USB
// report (a spike to the other position) is never a flip.
constexpr float kSwitchOnShare = 0.35f, kSwitchOffShare = 0.65f;
constexpr float kControlDebounce = 0.03f;

inline bool switchOnWithHysteresis(const DeviceMapping &m, const ChannelBinding &b, float raw, bool wasOn) {
    float share = wasOn ? kSwitchOffShare : kSwitchOnShare;
    return std::fabs(raw - b.switchOn()) < share * switchSpacing(m, b);
}

// One game control's state, updated at the simulation rate.
struct ControlState {
    bool on = false;
    uint32_t presses = 0;
    bool radioValid = false, radioPrev = false;  // the radio's level last update
    bool pushPrev = false;  // a radio push button (toggle binding)
    bool keyPrev = false, padPrev = false;
    // the radio before the debounce: the raw reading (for the hysteresis),
    // and the level it settled on
    bool rawOn = false, rawValid = false, settled = false;
    float pending = 0;  // seconds the raw reading has differed from `settled`
};

// The radio's reading of a control after the debounce (see kControlDebounce).
inline bool debounceControl(ControlState &st, bool raw, float dt) {
    st.rawOn = raw;
    if (!st.rawValid) {
        st.rawValid = true;
        st.settled = raw;
        st.pending = 0;
    } else if (raw == st.settled) {
        st.pending = 0;
    } else if ((st.pending += dt) >= kControlDebounce) {
        st.settled = raw;
        st.pending = 0;
    }
    return st.settled;
}

// What one update read for a control: its radio binding (a level: a switch
// position or a held button; or a push of a button standing in for a
// switch), its key and its gamepad button.
struct ControlReading {
    bool levelValid = false, level = false;
    bool push = false;
    bool bothWays = false;  // an instant control on a radio switch: each flip is a press
    bool key = false, pad = false;
};

inline void updateControlState(ControlKind kind, ControlState &st, const ControlReading &r) {
    bool before = st.on;
    if (kind != ControlKind::Switch) {
        // the first reading of a radio switch is where it is, not a press
        if (r.levelValid && !st.radioValid && r.level) before = true;
        st.on = (r.levelValid && r.level) || r.key || r.pad;
        if (r.bothWays && r.levelValid && st.radioValid && st.radioPrev && !r.level) st.presses++;
    } else {
        if (r.levelValid && !st.radioValid) {
            before = r.level;  // the first reading of a switch is where it is, not a flip
            st.on = r.level;
        } else if (r.levelValid && r.level != st.radioPrev) {
            st.on = r.level;
        }
        if (r.push) st.on = !st.on;
        if (r.key && !st.keyPrev) st.on = !st.on;
        if (r.pad && !st.padPrev) st.on = !st.on;
    }
    st.radioValid = r.levelValid;
    st.radioPrev = r.level;
    st.keyPrev = r.key;
    st.padPrev = r.pad;
    if (st.on && !before) st.presses++;
}

// Gamepad buttons are SDL_GamepadButton indices; the triggers count as
// buttons past half travel.
constexpr int kPadLeftTrigger = 32, kPadRightTrigger = 33;

struct ControlDesc {
    uint32_t id = 0;
    std::string name;   // key in radio_mappings.json and settings.json
    std::string label;
    ControlKind kind = ControlKind::Instant;
    int key = 0;         // keyboard key (fpv_key / ASCII), 0 = none
    int padButton = -1;  // gamepad button, -1 = none
};

// Raw state of one connected device, produced by the backend.
struct RawDevice {
    uint32_t instanceId = 0;
    std::string guid;
    std::string name;
    bool isGamepad = false;   // recognised as a gamepad (fixed layout)
    bool looksLikeRadio = false;
    std::vector<float> axes;  // -1..1
    std::vector<uint8_t> buttons;
    // gamepad layout (when isGamepad): leftX, leftY, rightX, rightY, lt, rt
    float pad[6] = {0, 0, 0, 0, 0, 0};
    uint32_t padButtons = 0;  // bit per SDL_GamepadButton
};

class DeviceBackend {
public:
    virtual ~DeviceBackend() = default;
    virtual bool init(std::string &error) = 0;
    virtual void shutdown() = 0;
    // Polls devices; called at the simulation rate on the simulation thread.
    virtual void poll(std::vector<RawDevice> &devices) = 0;
};

std::unique_ptr<DeviceBackend> createSdlBackend();
// Browsers: the Gamepad API (radios in USB joystick mode show up there too).
std::unique_ptr<DeviceBackend> createWebBackend();

// Device names of radio handsets and simulator dongles.
bool nameLooksLikeRadio(const std::string &name);

// What the keyboard's hover hold needs to know about the drone, taken from
// the previous simulation step.
struct HoverFeedback {
    std::string preset;     // the learned hover throttle is kept per preset
    float climbRate = 0;    // m/s, up positive
    float upright = 1;      // cosine of the tilt from level (< 0 upside down)
    bool airborne = false;  // a drone exists and nothing touches the ground
};

// Automatic hover throttle for the keyboard (see mapKeyboard). Auto turns it
// on while no radio is detected, mapped or not.
enum class KeyboardHover { Auto = 0, On = 1, Off = 2 };

// Keyboard throttle and yaw exponent defaults (1/s), see mapKeyboard.
constexpr float kDefaultKeyboardThrottleExpo = 3.0f;
constexpr float kDefaultKeyboardYawExpo = 4.0f;

struct InputStatus {
    InputSource source = InputSource::None;
    std::string deviceName;
    bool needsSetup = false;
    bool radioDetected = false;  // a mapped or unmapped radio is connected
    int deviceCount = 0;
};

class InputManager {
public:
    InputManager();
    ~InputManager();

    void setDataDir(const std::string &dir);
    bool startDevices(std::string &error);  // optional; without it only host/keyboard input
    void stopDevices();

    // --- called from any thread -----------------------------------------
    void setHostChannels(const float *ch, uint32_t count, double wallTime);
    void clearHostChannels();
    void key(int key, bool down);
    void setKeyboardEnabled(bool enabled) { m_keyboardEnabled = enabled; }
    // Automatic hover throttle (see mapKeyboard); off = W/S move the throttle.
    void setKeyboardHover(KeyboardHover mode) { m_kbHover = mode; }
    // How fast W raises the throttle: it grows by e^(expo * seconds held).
    void setKeyboardThrottleExpo(float expo) { m_kbThrottleExpo = expo; }
    // How fast A/D raise the yaw, the same way from a small start.
    void setKeyboardYawExpo(float expo) { m_kbYawExpo = expo; }

    // --- simulation thread ----------------------------------------------
    void setHoverFeedback(const HoverFeedback &f);

    // Polls devices and produces Betaflight channels. `radioLost` is true
    // when the active radio disappeared while it was the input source.
    void update(double wallTime, float dt, float out[kChannelCount], bool &radioLost);

    // After the radio vanished, failsafe holds until Betaflight disarmed;
    // then other sources (gamepad, keyboard) may take over.
    void resetRadioLatch();

    InputStatus status();
    std::vector<RawDevice> devicesSnapshot();
    // Mapping access for the setup wizard (any thread).
    bool getMapping(const std::string &guid, DeviceMapping &out);
    void setMapping(const DeviceMapping &m);  // persists to data dir
    void forgetMapping(const std::string &guid);
    void forgetAllMappings();
    // While the wizard is reading raw sticks, flight input is held at idle.
    void setWizardActive(bool active) { m_wizardActive = active; }

    // Game controls (any thread). The key of each is the game's default
    // unless setControlKey overrides it (the player's choice).
    void setControls(const std::vector<ControlDesc> &controls);
    std::vector<ControlDesc> controls();
    void setControlKey(const std::string &name, int key);
    // On/off now, and the presses (off to on) since the last poll.
    bool pollControl(uint32_t id, uint32_t &presses);
    bool controlOn(uint32_t id);

private:
    void loadMappings();
    void saveMappings();
    bool mapRadio(const RawDevice &d, const DeviceMapping &m, float out[kChannelCount]);
    void mapGamepad(const RawDevice &d, float dt, float out[kChannelCount]);
    void mapKeyboard(float dt, bool radioDetected, float out[kChannelCount]);
    void updateControls(const RawDevice *radio, const DeviceMapping *map, float dt);

    std::mutex m_mutex;
    std::string m_dataDir;
    std::unique_ptr<DeviceBackend> m_backend;
    std::vector<RawDevice> m_devices;
    std::vector<DeviceMapping> m_mappings;

    float m_host[kChannelCount];
    double m_hostTime = -1e9;
    std::array<bool, 512> m_keys{};
    std::atomic<bool> m_keyboardEnabled{true};
    std::atomic<bool> m_wizardActive{false};
    std::atomic<KeyboardHover> m_kbHover{KeyboardHover::Auto};
    std::atomic<float> m_kbThrottleExpo{kDefaultKeyboardThrottleExpo};
    std::atomic<float> m_kbYawExpo{kDefaultKeyboardYawExpo};

    // fallback state
    float m_kbThrottle = 0;  // 0..1
    float m_kbYaw = 0;
    HoverFeedback m_hover;
    std::unordered_map<std::string, float> m_hoverThrottle;  // learned, per preset
    bool m_kbArmed = false, m_kbArmKeyPrev = false;
    bool m_kbTurtle = false, m_kbTurtleKeyPrev = false;
    bool m_padArmed = false, m_padArmPrev = false;
    bool m_padTurtle = false, m_padTurtlePrev = false;
#if defined(LIBFPV_KAMIKAZE)
    bool m_kbBomb = false, m_kbBombKeyPrev = false;
    bool m_padBomb = false, m_padBombPrev = false;
#endif
    float m_lastOut[kChannelCount];
    // toggle bindings of the active radio: pressed last update, and on/off
    std::array<bool, kChannelCount> m_togglePressed{}, m_toggleOn{};
    std::string m_toggleGuid;

    std::vector<ControlDesc> m_controls;
    std::vector<ControlState> m_controlState;  // parallel to m_controls
    std::map<std::string, int> m_controlKeys;  // the player's keys, by name

    InputStatus m_status;
    std::string m_activeGuid;
    bool m_radioWasActive = false;
};

}  // namespace fpv
