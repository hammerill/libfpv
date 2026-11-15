// libfpv - guided radio setup: move each stick, the wizard finds the axis,
// its direction and endpoints, then the switches for ARM and modes. Kamikaze
// builds set up the bomb switches separately (startBombSwitches).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <string>
#include <vector>

#include "input/input.h"

namespace fpv {

class RadioWizard {
public:
    enum class Step {
        PickDevice,
        Rest,
        Throttle,
        Yaw,
        Pitch,
        Roll,
        Calibrate,
        ArmSwitch,
        AngleSwitch,
        BeeperSwitch,
        TurtleSwitch,
        AirModeSwitch,
        OsdSwitch,
#if defined(LIBFPV_KAMIKAZE)
        BombArmSwitch,
        ExplodeSwitch,
#endif
        GameControl,  // the game's controls, one after the other (startControls)
        Done
    };
    // The full setup: sticks, then AUX1..6. Never the bomb switches.
    void start(const RawDevice &dev);
#if defined(LIBFPV_KAMIKAZE)
    // Only the bomb arm (AUX7) and explode (AUX8) switches, on a radio set
    // up already: the other bindings of `current` stay as they are, and the
    // new switches avoid the positions they use. Merge the result with
    // mergeBombSwitches.
    void startBombSwitches(const RawDevice &dev, const DeviceMapping &current);
    bool bombSwitchesOnly() const { return m_bombOnly; }
    // Copies the bomb switches (AUX7, AUX8) of `from` into `into`, leaving
    // its other bindings. With `onlyIfFree`, a switch whose axis position or
    // button `into` uses already is left out (after a new full setup).
    static void mergeBombSwitches(DeviceMapping &into, const DeviceMapping &from, bool onlyIfFree);
#endif
    // Only the game's controls (fpv_controls_set), each one optional, on a
    // radio set up already: the other bindings of `current` stay, and the
    // new ones avoid the positions and buttons they use. Merge the result
    // with mergeControls.
    void startControls(const RawDevice &dev, const DeviceMapping &current, const std::vector<ControlDesc> &controls);
    bool controlsOnly() const { return !m_controls.empty(); }
    const std::vector<ControlDesc> &controlsBeingSetUp() const { return m_controls; }
    // The control being set up (GameControl step), or nullptr.
    const ControlDesc *currentControl() const;
    // Copies the controls named in `names` from `from` into `into` (a name
    // `from` has no binding for is removed from `into`). With `onlyIfFree`,
    // a binding whose axis position or button `into` uses already is left out.
    static void mergeControls(DeviceMapping &into, const DeviceMapping &from, const std::vector<std::string> &names,
                              bool onlyIfFree);
    // Every control name bound in `m`.
    static std::vector<std::string> controlNames(const DeviceMapping &m);
    void cancel() { m_active = false; }
    bool active() const { return m_active; }
    Step step() const { return m_step; }
    const std::string &guid() const { return m_mapping.guid; }

    // Feeds the latest raw state of the device being set up.
    void update(const RawDevice &dev);
    // "Next" button; returns false with `hint` set when it cannot proceed.
    bool next(std::string &hint);
    void skip();  // optional switches only
    void back();

    const char *title() const;
    const char *instruction() const;
    bool canSkip() const {
        return (m_step >= Step::AngleSwitch && m_step <= lastSwitch()) || m_step == Step::GameControl;
    }
    // Live detection feedback for the UI.
    bool detected() const { return m_candidate >= 0; }
    std::string detectedLabel() const;  // e.g. "stick 3, reversed"
    float progress() const;             // 0..1 over the steps

    const DeviceMapping &mapping() const { return m_mapping; }

    // Switch steps: the control is a push button (or a momentary switch),
    // pressed rather than held. Detection keeps what it saw after the
    // release, and the channel toggles on each press (the explode switch
    // stays on while pressed).
    bool isSwitchStep() const {
        return (m_step >= Step::ArmSwitch && m_step <= lastSwitch()) || m_step == Step::GameControl;
    }
    // The explode switch is pressed, not held, by nature: its step always
    // keeps a press after the release, and it never toggles, so it has no
    // "It's a button" choice (momentaryChoice() is false).
    bool momentary() const { return m_momentary || explodeStep(); }
    bool momentaryChoice() const { return isSwitchStep() && !explodeStep(); }
    void setMomentary(bool on);

    // A switch step saw a switch position or button move that another
    // binding uses already, and nothing free moved: the UI offers to take
    // it anyway.
    bool hasConflict() const { return m_conflict >= 0 && m_candidate < 0; }
    // "Button 5 is already used by Fire orange." `known` gives the labels of
    // game controls (by name); a control not in it shows its name.
    std::string conflictText(const std::vector<ControlDesc> &known) const;
    // Takes the conflicting switch or button for this step and goes on, as
    // Next does. `replace` unbinds it from what used it; otherwise both
    // keep it.
    bool takeConflict(bool replace, std::string &hint);
    // Removes from `into` the bindings this setup replaced (takeConflict)
    // that `into` still has: for the merges of a partial setup.
    void applyReplaced(DeviceMapping &into) const;

private:
    struct Candidate {
        int axis = -1, button = -1;
        float value = 0;  // raw value at the detected position
    };
    bool explodeStep() const {
#if defined(LIBFPV_KAMIKAZE)
        return m_step == Step::ExplodeSwitch;
#else
        return false;
#endif
    }
    void detectAxis(const RawDevice &dev, bool allowButtons);
    bool axisUsed(int axis) const;
    bool stickAxisUsed(int axis) const;
    bool positionFree(int axis, float value) const;
    int sharedWith(int axis) const;  // first AUX channel on this axis, or -1
    bool buttonUsed(int button) const;
    int channelForStep(Step s) const;
    // the last switch step; every switch after the arm switch is optional
    Step lastSwitch() const;

    bool m_active = false;
    bool m_bombOnly = false;  // startBombSwitches: AUX7 and AUX8 only
    Step m_step = Step::PickDevice;
    DeviceMapping m_mapping;
    std::vector<float> m_rest;
    std::vector<uint8_t> m_restButtons;
    std::vector<float> m_min, m_max;
    int m_candidate = -1;  // axis index, or 1000 + button
    float m_candidateValue = 0;
    bool m_momentary = false;  // the current switch step is a push button
    std::vector<ControlDesc> m_controls;  // startControls: the ones to set up
    size_t m_controlIndex = 0;
    std::string m_controlTitle, m_controlText;  // title() / instruction() of a control step
    void enterControl();  // m_controlIndex: its texts, a fresh detection

    // A used input moved in this step (axis index, or 1000 + button; -1 =
    // none), kept until something free moves or the step changes.
    int m_conflict = -1;
    float m_conflictValue = 0;
    bool m_replace = false;  // the candidate unbinds the others that use it
    // The raw state when the current step began: only what moves after it
    // counts as a conflict, not a switch left on in an earlier step.
    std::vector<float> m_stepAxes;
    std::vector<uint8_t> m_stepButtons;
    std::vector<uint8_t> m_axisMoved, m_buttonMoved;  // left the baseline in this step
    std::vector<float> m_lastAxes;                     // the last update's raw state
    std::vector<uint8_t> m_lastButtons;
    Step m_snapStep = Step::PickDevice;
    size_t m_snapIndex = 0;
    bool m_snapped = false;
    // Bindings unbound by a replace, to restore when its step is undone.
    struct Replaced {
        Step step;
        size_t controlIndex;
        int channel;          // or -1
        std::string control;  // or empty
        ChannelBinding binding;
    };
    std::vector<Replaced> m_replaced;
    static bool owns(const ChannelBinding &b, int input, float value);
    // The other channels (AUX) and controls bound to `input` at `value`,
    // the current step's own one left out.
    void ownersOf(int input, float value, std::vector<int> &channels, std::vector<std::string> &controls) const;
    void restoreReplaced();
};

}  // namespace fpv
