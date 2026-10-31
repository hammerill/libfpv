// libfpv - built-in Dear ImGui overlay.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "overlay/overlay.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/bf_bridge.h"
#include "imgui.h"
#include "lib.h"
#include "osd/osd.h"
#include "util/log.h"
#include "web/web.h"

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#endif

namespace fpv {

OsdRenderer &osdRenderer();  // api_present.cpp

Overlay &overlay() {
    Lib &L = lib();
    if (!L.overlay) L.overlay = new Overlay();
    return *L.overlay;
}

namespace {

ImGuiKey toImGuiKey(int k) {
    if (k >= 'A' && k <= 'Z') return (ImGuiKey)(ImGuiKey_A + (k - 'A'));
    if (k >= 'a' && k <= 'z') return (ImGuiKey)(ImGuiKey_A + (k - 'a'));
    if (k >= '0' && k <= '9') return (ImGuiKey)(ImGuiKey_0 + (k - '0'));
    switch (k) {
    case FPV_KEY_TAB: return ImGuiKey_Tab;
    case FPV_KEY_ENTER: return ImGuiKey_Enter;
    case FPV_KEY_ESCAPE: return ImGuiKey_Escape;
    case FPV_KEY_SPACE: return ImGuiKey_Space;
    case FPV_KEY_LEFT: return ImGuiKey_LeftArrow;
    case FPV_KEY_RIGHT: return ImGuiKey_RightArrow;
    case FPV_KEY_UP: return ImGuiKey_UpArrow;
    case FPV_KEY_DOWN: return ImGuiKey_DownArrow;
    case FPV_KEY_PAGE_UP: return ImGuiKey_PageUp;
    case FPV_KEY_PAGE_DOWN: return ImGuiKey_PageDown;
    case FPV_KEY_HOME: return ImGuiKey_Home;
    case FPV_KEY_END: return ImGuiKey_End;
    case FPV_KEY_INSERT: return ImGuiKey_Insert;
    case FPV_KEY_DELETE: return ImGuiKey_Delete;
    case FPV_KEY_BACKSPACE: return ImGuiKey_Backspace;
    case FPV_KEY_LSHIFT: return ImGuiKey_LeftShift;
    case FPV_KEY_RSHIFT: return ImGuiKey_RightShift;
    case FPV_KEY_LCTRL: return ImGuiKey_LeftCtrl;
    case FPV_KEY_RCTRL: return ImGuiKey_RightCtrl;
    case FPV_KEY_LALT: return ImGuiKey_LeftAlt;
    case FPV_KEY_RALT: return ImGuiKey_RightAlt;
    default: break;
    }
    if (k >= FPV_KEY_F1 && k <= FPV_KEY_F12) return (ImGuiKey)(ImGuiKey_F1 + (k - FPV_KEY_F1));
    return ImGuiKey_None;
}

const char *keyName(int k) {
    static char buf[16];
    if ((k >= 'A' && k <= 'Z') || (k >= '0' && k <= '9')) {
        buf[0] = (char)k;
        buf[1] = 0;
        return buf;
    }
    if (k >= FPV_KEY_F1 && k <= FPV_KEY_F12) {
        snprintf(buf, sizeof buf, "F%d", k - FPV_KEY_F1 + 1);
        return buf;
    }
    switch (k) {
    case FPV_KEY_TAB: return "Tab";
    case FPV_KEY_ENTER: return "Enter";
    case FPV_KEY_ESCAPE: return "Esc";
    case FPV_KEY_SPACE: return "Space";
    case FPV_KEY_INSERT: return "Insert";
    case FPV_KEY_DELETE: return "Delete";
    case FPV_KEY_HOME: return "Home";
    case FPV_KEY_END: return "End";
    case FPV_KEY_PAGE_UP: return "Page Up";
    case FPV_KEY_PAGE_DOWN: return "Page Down";
    default: break;
    }
    snprintf(buf, sizeof buf, "key %d", k);
    return buf;
}

// Plain-language reasons for Betaflight's arming-disable flags.
const char *armingHelp(const char *flag) {
    struct H {
        const char *f, *h;
    } static const help[] = {
        {"THROTTLE", "Throttle is not at the bottom."},
        {"ARM_SWITCH", "The arm switch was on when the drone became armable: switch it off, then on again."},
        {"ANGLE", "The drone is tilted too much (more than small_angle). Reset it upright."},
        {"RXLOSS", "No radio signal. Check the Radio tab."},
        {"FAILSAFE", "Failsafe is active: the radio signal was lost."},
        {"CALIB", "Sensors are calibrating; keep the drone still for a moment."},
        {"CLI", "The Betaflight Configurator is in its CLI tab."},
        {"MSP", "The Configurator is connected and blocking arming (normal while connected)."},
        {"BOOTGRACE", "Betaflight's power-on delay."},
        {"NOPREARM", "A PREARM switch is configured and not on."},
        {"CRASH", "Crash detection triggered: disarm and re-arm."},
        {"RUNAWAY", "Runaway takeoff prevention triggered: disarm and re-arm."},
        {"REBOOT_REQD", "A setting needs a reboot: libfpv reboots Betaflight when you import; otherwise press Reboot in the Configurator tab."},
        {"MOTOR_PROTO", "Motor protocol was changed: only PWM works in the simulator (libfpv restores it on next start)."},
        {"PARALYZE", "PARALYZE mode is active."},
        {"LOAD", "CPU load too high."},
        {"BOMB", "The bomb arm switch is on (or the drone was destroyed): the motors only arm with the bomb safe. "
                 "Switch it off, then switch the arm switch off and on again."},
    };
    for (auto &x : help)
        if (std::strcmp(x.f, flag) == 0) return x.h;
    return "";
}

void sectionTitle(const char *t) {
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "%s", t);
    ImGui::Separator();
}

}  // namespace

Overlay::Overlay() {
    m_bf.watch("rates_type");
    m_bf.watch("_rc_rate");
    m_bf.watch("_srate");
    m_bf.watch("_expo");
    m_bf.watch("_rate_limit");
    m_bf.watch("quickrates_rc_expo");
    m_bf.watch("thr_mid");
    m_bf.watch("thr_expo");
    m_bf.watch("simplified_");
    m_bf.watch("angle_limit");
    m_bf.watch("osd_");
    m_bf.watch("yaw_motors_reversed");
    m_bf.watch("craft_name");
}

void Overlay::ensureContext() {
    if (m_ctx) return;
    IMGUI_CHECKVERSION();
    ImGuiContext *prev = ImGui::GetCurrentContext();
    m_ctx = ImGui::CreateContext();
    ImGui::SetCurrentContext(m_ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr;  // nothing written next to the game
    io.LogFilename = nullptr;
    io.BackendPlatformName = "libfpv";
    io.BackendRendererName = "libfpv-host";
    io.MouseDrawCursor = true;  // games usually hide the OS cursor
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
#if defined(__EMSCRIPTEN__)
    // Copy goes to the system clipboard. Browsers only read it
    // asynchronously, so Ctrl+V is handled in key() and ImGui's own paste
    // finds nothing.
    ImGuiPlatformIO &pio = ImGui::GetPlatformIO();
    pio.Platform_SetClipboardTextFn = [](ImGuiContext *, const char *text) { libfpv_web_clipboard_write(text); };
    pio.Platform_GetClipboardTextFn = [](ImGuiContext *) -> const char * { return ""; };
#endif
    ImGui::StyleColorsDark();
    ImGuiStyle &st = ImGui::GetStyle();
    st.WindowRounding = 6;
    st.FrameRounding = 4;
    st.GrabRounding = 4;
    rebuildFonts();
    ImGui::SetCurrentContext(prev);
}

void Overlay::rebuildFonts() {
    ImGuiIO &io = ImGui::GetIO();
    ImFontConfig cfg;
    cfg.SizePixels = 32.0f;  // baked large, drawn at 16 * ui_scale
    io.Fonts->AddFontDefaultVector(&cfg);
    unsigned char *px = nullptr;
    io.Fonts->GetTexDataAsRGBA32(&px, &m_fontW, &m_fontH);
    m_fontPixels.assign(px, px + (size_t)m_fontW * m_fontH * 4);
    io.Fonts->SetTexID((ImTextureID)FPV_TEXTURE_OVERLAY_FONT);
    ImGui::GetStyle().FontSizeBase = 16.0f;
    m_fontRevision++;
}

bool Overlay::fontTexture(fpv_texture &out) {
    ensureContext();
    if (m_fontPixels.empty()) return false;
    out.width = (uint32_t)m_fontW;
    out.height = (uint32_t)m_fontH;
    out.rgba = m_fontPixels.data();
    out.revision = m_fontRevision;
    return true;
}

void Overlay::setOpen(bool open) {
    if (open == m_open) return;
    m_open = open;
    if (open) m_bf.invalidate();
    if (!open && m_wizard.active() && m_wizard.step() != RadioWizard::Step::Done) {
        m_wizard.cancel();
        lib().input.setWizardActive(false);
    }
}

void Overlay::setDisplay(float w, float h, float scale) {
    if (w > 0) m_w = w;
    if (h > 0) m_h = h;
    if (scale > 0 && std::isfinite(scale)) m_scale = std::clamp(scale, 0.5f, 4.0f);
}

#define WITH_CTX                                     \
    ensureContext();                                 \
    ImGuiContext *prev_ = ImGui::GetCurrentContext(); \
    ImGui::SetCurrentContext(m_ctx)
#define END_CTX ImGui::SetCurrentContext(prev_)

void Overlay::mouseMove(float x, float y) {
    WITH_CTX;
    ImGui::GetIO().AddMousePosEvent(x, y);
    END_CTX;
}
void Overlay::mouseButton(int b, bool down) {
    if (b < 0 || b > 4) return;
    WITH_CTX;
    ImGui::GetIO().AddMouseButtonEvent(b, down);
    END_CTX;
}
void Overlay::mouseWheel(float dx, float dy) {
    WITH_CTX;
    ImGui::GetIO().AddMouseWheelEvent(dx, dy);
    END_CTX;
}
void Overlay::text(const char *utf8) {
    if (!m_open) return;
    WITH_CTX;
    ImGui::GetIO().AddInputCharactersUTF8(utf8);
    END_CTX;
}

bool Overlay::capturingKey() const { return m_rebindingToggle; }

void Overlay::key(int k, bool down) {
    if (m_rebindingToggle && down) {
        if (k != FPV_KEY_ESCAPE) {
            lib().settings.overlayToggleKey = k;
            saveSettings();
        }
        m_rebindingToggle = false;
        return;
    }
    if (k == FPV_KEY_LCTRL || k == FPV_KEY_RCTRL) m_ctrl = down;
#if defined(__EMSCRIPTEN__)
    if (m_open && down && m_ctrl && k == 'V') libfpv_web_clipboard_read(0);
#endif
    WITH_CTX;
    ImGuiIO &io = ImGui::GetIO();
    // Esc closes the overlay, unless a popup or a text field takes it first
    bool close = m_open && down && k == FPV_KEY_ESCAPE && !io.WantTextInput &&
                 !ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId);
    ImGuiKey ik = toImGuiKey(k);
    if (ik != ImGuiKey_None) io.AddKeyEvent(ik, down);
    if (k == FPV_KEY_LCTRL || k == FPV_KEY_RCTRL) io.AddKeyEvent(ImGuiMod_Ctrl, down);
    if (k == FPV_KEY_LSHIFT || k == FPV_KEY_RSHIFT) io.AddKeyEvent(ImGuiMod_Shift, down);
    if (k == FPV_KEY_LALT || k == FPV_KEY_RALT) io.AddKeyEvent(ImGuiMod_Alt, down);
    END_CTX;
    if (close) setOpen(false);
}

void Overlay::paste(const char *utf8, bool toImport) {
    if (toImport) m_importText = utf8;
    else text(utf8);
}

void Overlay::notifyUnmappedRadio(const std::string &guid) {
    if (std::find(m_promptedRadios.begin(), m_promptedRadios.end(), guid) != m_promptedRadios.end()) return;
    m_promptedRadios.push_back(guid);
    setOpen(true);
    m_requestTab = &Overlay::tabRadio;
}

// --- tabs ------------------------------------------------------------------------

void Overlay::channelBars(const float *ch) {
    static const char *names[kChannelCount] = {"Roll",        "Pitch",       "Throttle",     "Yaw",
                                               "AUX1 arm",    "AUX2 angle",  "AUX3 beeper",  "AUX4 turtle",
                                               "AUX5 air mode", "AUX6 OSD off"
#if defined(LIBFPV_KAMIKAZE)
                                               , "AUX7 bomb arm", "AUX8 explode"
#endif
    };
    for (int i = 0; i < kChannelCount; i++) {
        char label[32];
        snprintf(label, sizeof label, "%d", (int)std::lround(1500 + ch[i] * 500));
        ImGui::ProgressBar((ch[i] + 1) * 0.5f, ImVec2(ImGui::GetFontSize() * 14, 0), label);
        ImGui::SameLine();
        ImGui::TextUnformatted(names[i]);
    }
}

void Overlay::tabFly() {
    Lib &L = lib();
    const fpv_frame &f = L.lastFrame;
    if (!L.droneExists) {
        ImGui::TextWrapped("No drone is spawned. The game spawns it; settings below still apply.");
    } else {
        bool armed = f.flags & FPV_DRONE_ARMED;
        ImGui::TextColored(armed ? ImVec4(1, 0.4f, 0.3f, 1) : ImVec4(0.5f, 1, 0.5f, 1), "%s",
                           armed ? "ARMED" : "Disarmed");
        if (!armed && f.arming_disable_text[0]) {
            sectionTitle("Why it does not arm");
            char flags[96];
            snprintf(flags, sizeof flags, "%s", f.arming_disable_text);
            for (char *tok = std::strtok(flags, " "); tok; tok = std::strtok(nullptr, " ")) {
                const char *h = armingHelp(tok);
                ImGui::BulletText("%s%s%s", tok, h[0] ? ": " : "", h);
            }
        } else if (!armed) {
            ImGui::TextWrapped("Ready: throttle down, flip the arm switch.");
        }
        if (f.flags & FPV_DRONE_FAILSAFE) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "FAILSAFE");
#if defined(LIBFPV_KAMIKAZE)
        warheadStatus();  // with the bomb (or its wreck); settings: the Kamikaze tab
#endif
        if (f.flags & FPV_DRONE_CRASHED)
            ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "Crashed (broken props or burned motor): press Repair.");
        sectionTitle("Drone");
        ImGui::Text("Speed %.0f km/h   Battery %.2f V (%.2f V/cell), %.0f mAh used", f.speed_kmh, f.battery.voltage,
                    f.battery.cell_voltage, f.battery.mah_drawn);
        for (int i = 0; i < 4; i++) {
            ImGui::Text("Motor %d: %5.0f rpm  %4.1f A  %3.0f C  prop %3.0f%%", i + 1, f.motors[i].rpm,
                        f.motors[i].current_amp, f.motors[i].temperature_c, (1 - f.motors[i].prop_damage) * 100);
        }
        if (ImGui::Button("Repair")) fpv_drone_repair(&L.drone);
        ImGui::SameLine();
        if (ImGui::Button("Respawn")) {
            float p[3], q[4];
            Vec3 hp = L.conv.posToHost(L.spawnPos);
            Quat hq = L.conv.rotToHost(L.spawnRot);
            p[0] = hp.x, p[1] = hp.y, p[2] = hp.z;
            q[0] = hq.x, q[1] = hq.y, q[2] = hq.z, q[3] = hq.w;
            fpv_drone_repair(&L.drone);
            fpv_drone_reset(&L.drone, p, q);
        }
        ImGui::SameLine();
        bool paused = L.runner.paused();
        if (ImGui::Checkbox("Pause", &paused)) L.runner.setPaused(paused);
    }
    if (!L.hostActions.empty()) {
        sectionTitle("Game");
        // wrapped to the window width, however many the game adds
        const ImGuiStyle &style = ImGui::GetStyle();
        float right = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
        for (size_t i = 0; i < L.hostActions.size(); i++) {
            const Lib::HostAction &a = L.hostActions[i];
            float w = ImGui::CalcTextSize(a.label.c_str()).x + style.FramePadding.x * 2;
            if (i && ImGui::GetItemRectMax().x + style.ItemSpacing.x + w <= right) ImGui::SameLine();
            ImGui::PushID((int)a.id);
            if (ImGui::Button(a.label.c_str())) L.clickedActions.push_back(a.id);
            ImGui::PopID();
        }
    }
    windSection();
    sectionTitle("Input");
    InputStatus st = L.input.status();
    static const char *src[] = {"none", "radio", "gamepad (degraded)", "keyboard (degraded)", "game/script"};
    ImGui::Text("Source: %s - %s", src[(int)st.source], st.deviceName.c_str());
    if (st.needsSetup) {
        ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "A radio is connected but not set up.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Set it up")) m_requestTab = &Overlay::tabRadio;
    }
    ImGui::TextDisabled("Overlay: %s", keyName(L.settings.overlayToggleKey));
}

// Wind on/off and strength, and what the drone feels now.
void Overlay::windSection() {
    Lib &L = lib();
    sectionTitle("Wind");
    bool changed = ImGui::Checkbox("Wind", &L.settings.windEnabled);
    ImGui::SameLine();
    ImGui::BeginDisabled(!L.settings.windEnabled);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    ImGui::SliderFloat("Weather wind strength", &L.settings.windStrength, 0, 2, "%.2fx");
    changed |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::EndDisabled();
    if (changed) saveSettings();
    if (!L.settings.windEnabled) {
        ImGui::TextDisabled("Off: the air is still, whatever the game's weather.");
        return;
    }
    if (!L.windSet && L.windSources.empty()) {
        ImGui::TextDisabled("This game gives no wind.");
        return;
    }
    if (!L.droneExists || !L.lastFrameValid) return;
    // a wind is named by where it comes from
    Vec3 w = L.conv.dirToInternal({L.lastFrame.wind[0], L.lastFrame.wind[1], L.lastFrame.wind[2]}) /
             L.conv.unitsPerMeter();
    float speed = length(Vec3{w.x, 0, w.z});
    static const char *const kCompass[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    float from = std::atan2(-w.x, -w.z) * 180.0f / kPi;
    int sector = ((int)std::lround(from / 45.0f) % 8 + 8) % 8;
    ImGui::Text("At the drone: %.1f m/s from %s, %+.1f m/s vertical", speed, kCompass[sector], w.y);
    float shelter = L.windProbe.shelter(), enclosed = L.windProbe.enclosure();
    if (enclosed > 0.3f)
        ImGui::TextDisabled("Indoors or under cover: little of the weather gets in.");
    else if (shelter > 0.2f)
        ImGui::TextDisabled("In the lee of obstacles: %.0f%% of the wind held off, gustier.",
                            std::min(shelter, 1.0f) * 100);
}

// The setup wizard in progress (the full one in the Radio tab, the bomb
// switches in the Kamikaze tab).
void Overlay::wizardUi(const std::vector<RawDevice> &devs) {
    Lib &L = lib();
    const RawDevice *dev = nullptr;
    for (auto &d : devs)
        if (d.guid == m_wizard.guid()) dev = &d;
    if (!dev) {
        ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "The radio was disconnected. Plug it back in to continue.");
        if (ImGui::Button("Cancel setup")) {
            m_wizard.cancel();
            L.input.setWizardActive(false);
        }
        return;
    }
    m_wizard.update(*dev);
    ImGui::ProgressBar(m_wizard.progress(), ImVec2(-1, 0), "");
    ImGui::TextColored(ImVec4(0.55f, 0.8f, 1, 1), "%s", m_wizard.title());
    ImGui::TextWrapped("%s", m_wizard.instruction());
    ImGui::Spacing();
    if (m_wizard.step() != RadioWizard::Step::Done && m_wizard.step() != RadioWizard::Step::Rest &&
        m_wizard.step() != RadioWizard::Step::Calibrate) {
        if (m_wizard.detected())
            ImGui::TextColored(ImVec4(0.5f, 1, 0.5f, 1), "Detected%s%s", m_showRawAxes ? ": " : "",
                               m_showRawAxes ? m_wizard.detectedLabel().c_str() : "");
        else
            ImGui::TextDisabled("Waiting for movement...");
    }
    if (m_wizard.isSwitchStep()) {
        bool momentary = m_wizard.momentary();
        if (ImGui::Checkbox("It's a button, not a switch (press it, don't hold it)", &momentary))
            m_wizard.setMomentary(momentary);
        if (ImGui::IsItemHovered()) {
#if defined(LIBFPV_KAMIKAZE)
            if (m_wizard.step() == RadioWizard::Step::ExplodeSwitch)
                ImGui::SetTooltip("For a push button or a spring-loaded switch: press it once and let go, the wizard "
                                  "remembers it. It explodes while pressed.");
            else
#endif
                ImGui::SetTooltip("For a push button or a spring-loaded switch, which only reads as flipped while "
                                  "held: press it once and let go, the wizard remembers it. In flight each press "
                                  "turns this switch on or off.");
        }
    }
    if (m_wizard.step() == RadioWizard::Step::Done) {
        if (!m_wizardSaved) {
            // persisted once; flight input switches to the mapped radio
            DeviceMapping m = m_wizard.mapping();
#if defined(LIBFPV_KAMIKAZE)
            if (m_wizard.bombSwitchesOnly()) {
                // only AUX7 / AUX8 change, whatever else is set up now
                DeviceMapping cur;
                if (L.input.getMapping(m.guid, cur)) {
                    RadioWizard::mergeBombSwitches(cur, m, false);
                    m = cur;
                }
            } else {
                // a new full setup keeps the bomb switches that still fit
                RadioWizard::mergeBombSwitches(m, m_previousMapping, true);
            }
#endif
            L.input.setMapping(m);
            L.input.setWizardActive(false);
            m_wizardSaved = true;
        }
        channelBars(L.lastFrame.channels);
        if (ImGui::Button("Finish")) m_wizard.cancel();
        ImGui::SameLine();
        if (ImGui::Button("Start over")) {
#if defined(LIBFPV_KAMIKAZE)
            if (m_wizard.bombSwitchesOnly()) {
                DeviceMapping cur;
                L.input.getMapping(dev->guid, cur);
                m_wizard.startBombSwitches(*dev, cur);
            } else
#endif
            {
                L.input.forgetMapping(m_wizard.guid());
                m_wizard.start(*dev);
            }
            m_wizardSaved = false;
            L.input.setWizardActive(true);
        }
    } else {
        if (ImGui::Button("Back")) m_wizard.back();
        ImGui::SameLine();
        if (ImGui::Button("Next")) m_wizard.next(m_wizardHint);
        if (m_wizard.canSkip()) {
            ImGui::SameLine();
            if (ImGui::Button("Skip")) m_wizard.skip();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            m_wizard.cancel();
            L.input.setWizardActive(false);
        }
        if (!m_wizardHint.empty()) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "%s", m_wizardHint.c_str());
    }
    ImGui::Checkbox("Show raw axes", &m_showRawAxes);
    if (m_showRawAxes) {
        for (size_t i = 0; i < dev->axes.size(); i++) {
            char lbl[16];
            snprintf(lbl, sizeof lbl, "axis %zu", i + 1);
            ImGui::ProgressBar((dev->axes[i] + 1) * 0.5f, ImVec2(ImGui::GetFontSize() * 10, 0), lbl);
        }
    }
}

void Overlay::tabRadio() {
    Lib &L = lib();
    std::vector<RawDevice> devs = L.input.devicesSnapshot();
#if defined(LIBFPV_KAMIKAZE)
    // a finished (saved) bomb switch setup only waits for "Finish": it doesn't hold this tab
    if (m_wizard.active() && m_wizard.bombSwitchesOnly() && m_wizard.step() == RadioWizard::Step::Done &&
        m_wizardSaved)
        m_wizard.cancel();
#endif
    if (m_wizard.active()) {
#if defined(LIBFPV_KAMIKAZE)
        if (m_wizard.bombSwitchesOnly()) {
            ImGui::TextWrapped("The bomb switches are being set up in the Kamikaze tab.");
            if (ImGui::SmallButton("Go there")) m_requestTab = &Overlay::tabKamikaze;
            return;
        }
#endif
        wizardUi(devs);
        return;
    }

    if (devs.empty()) {
        ImGui::TextWrapped(
            "No radio or gamepad detected. Connect your radio with USB and choose \"USB Joystick (HID)\" on the radio "
            "(EdgeTX/OpenTX), or plug in a simulator dongle. Keyboard works as a fallback: W/S throttle, A/D yaw, "
            "arrow keys pitch/roll, Enter arms.");
#if defined(__EMSCRIPTEN__)
        ImGui::TextWrapped(
            "In a web browser, a connected radio or gamepad only shows up here after you move a stick or press a "
            "button on it.");
#endif
    }
    for (auto &d : devs) {
        ImGui::PushID(d.guid.c_str());
        DeviceMapping m;
        bool mapped = L.input.getMapping(d.guid, m) && m.complete;
        ImGui::Text("%s", d.name.c_str());
        ImGui::SameLine();
        if (mapped) ImGui::TextColored(ImVec4(0.5f, 1, 0.5f, 1), "(set up)");
        else if (d.isGamepad) ImGui::TextDisabled("(gamepad: default layout, degraded)");
        else ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "(not set up)");
        if (ImGui::Button(mapped ? "Set up again" : "Set up")) {
#if defined(LIBFPV_KAMIKAZE)
            // the bomb switches have their own setup (Kamikaze tab): keep them
            m_previousMapping = DeviceMapping{};
            L.input.getMapping(d.guid, m_previousMapping);
#endif
            m_wizard.start(d);
            m_wizardHint.clear();
            m_wizardSaved = false;
            L.input.setWizardActive(true);
        }
        if (mapped) {
            ImGui::SameLine();
            if (ImGui::Button("Forget")) L.input.forgetMapping(d.guid);
        }
        ImGui::PopID();
        ImGui::Separator();
    }
    sectionTitle("Channels Betaflight receives");
    channelBars(L.lastFrame.channels);
}

#if defined(LIBFPV_KAMIKAZE)
// The warhead's state, while the bomb is installed or the drone destroyed.
void Overlay::warheadStatus() {
    Lib &L = lib();
    fpv_kamikaze_state ks{};
    ks.struct_size = sizeof ks;
    if (!L.droneExists || fpv_kamikaze_get_state(&L.drone, &ks) != FPV_OK) return;
    if (!(ks.flags & FPV_KAMIKAZE_INSTALLED) && ks.warhead != FPV_WARHEAD_DETONATED) return;
    static const char *names[] = {"SAFE", "ARMING", "ARMED (live)", "DETONATED"};
    ImVec4 col = ks.warhead == FPV_WARHEAD_SAFE ? ImVec4(0.5f, 1, 0.5f, 1) : ImVec4(1, 0.4f, 0.3f, 1);
    ImGui::TextColored(col, "Warhead: %s", names[ks.warhead]);
    if (ks.warhead == FPV_WARHEAD_ARMING) {
        ImGui::SameLine();
        ImGui::Text("(%.1f s)", ks.arming_remaining);
    }
    if (ks.warhead == FPV_WARHEAD_DETONATED) {
        ImGui::SameLine();
        ImGui::TextUnformatted("- press Repair (Fly tab) for a new drone");
    }
}

namespace {

std::string bindingText(const ChannelBinding &b) {
    if (b.button >= 0) return "button " + std::to_string(b.button + 1);
    if (b.axis >= 0) return "axis " + std::to_string(b.axis + 1);
    return "not set";
}

}  // namespace

void Overlay::tabKamikaze() {
    Lib &L = lib();
    Settings &st = L.settings;
    bool installed = fpv_kamikaze_is_installed() != 0;
    if (ImGui::Checkbox("Bomb installed", &installed)) fpv_kamikaze_set_installed(installed);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Carry the warhead: its weight, its OSD line and the bomb arm (AUX7) and explode (AUX8) "
                          "switches. Kept for this session, also on respawned drones.");
    warheadStatus();
    bool changed = false;
    const float w = ImGui::GetFontSize() * 14;

    sectionTitle("Warhead");
    fpv_kamikaze_state ks{};
    ks.struct_size = sizeof ks;
    if (installed && L.droneExists && fpv_kamikaze_get_state(&L.drone, &ks) == FPV_OK)
        ImGui::Text("Payload %.0f g, all-up weight %.0f g", ks.payload_mass * 1000, ks.total_mass * 1000);
    // the payload is real weight; the player can tune it
    bool autoMass = st.kamikazePayloadKg < 0;
    if (ImGui::Checkbox("Automatic payload (the quad's preset)", &autoMass)) {
        st.kamikazePayloadKg = autoMass ? -1.0f : (ks.payload_mass > 0 ? ks.payload_mass : 0.3f);
        changed = true;
    }
    if (!autoMass) {
        float g = st.kamikazePayloadKg * 1000;
        ImGui::SetNextItemWidth(w);
        if (ImGui::SliderFloat("Payload (g)", &g, 0, 3000, "%.0f")) {
            st.kamikazePayloadKg = g / 1000;
            changed = true;
        }
    }
    ImGui::SetNextItemWidth(w);
    if (ImGui::SliderFloat("Arming delay (s)", &st.kamikazeArmingDelay, 0, 10, "%.1f")) changed = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("From the bomb arm switch (motors armed) to a live warhead. 0 = live at once.");
    ImGui::SetNextItemWidth(w);
    if (ImGui::SliderFloat("Impact fuse (m/s)", &st.kamikazeMinImpactSpeed, 0.1f, 30.0f, "%.2f",
                           ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp))
        changed = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The slowest touch that sets the live warhead off. 0.25 = a light tap on the ground; higher "
                          "needs a real hit.");
    if (ImGui::Checkbox("Warhead state on the OSD", &st.kamikazeOsd)) changed = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Its place: \"Bomb status\" in the OSD tab (\"Custom message 1\" in the Betaflight "
                          "Configurator).");

    // the three steps of the ending, as far as the game can do them
    sectionTitle("After the explosion");
    static const char *views[] = {"Signal lost (static)", "Cutscene (the blast seen by the game camera)"};
    static const char *thens[] = {"Respawn the drone", "Recall: end the flight, back to the pilot",
                                  "Nothing: keep the view until you act"};
    uint32_t view = kamikazeEndingView(), then = kamikazeEndingThen();
    // 0 s skips the view, unless nothing follows: then it is held
    bool skipView = st.kamikazeEndingDuration <= 0 && then != FPV_KAMIKAZE_THEN_NOTHING;
    ImGui::TextUnformatted("1. View");
    ImGui::Indent();
    ImGui::BeginDisabled(skipView);
    for (uint32_t i = 0; i < 2; i++) {
        if (!(L.kamikazeViewMask & (1u << i))) continue;
        if (ImGui::RadioButton(views[i], view == i) && view != i) {
            st.kamikazeEndingView = (int)i;
            changed = true;
        }
    }
    ImGui::EndDisabled();
    ImGui::Unindent();
    ImGui::TextUnformatted("2.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(w);
    if (ImGui::SliderFloat("View duration (s)", &st.kamikazeEndingDuration, 0, 30, "%.1f",
                           ImGuiSliderFlags_AlwaysClamp))
        changed = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("How long the game shows the view before step 3. 0 = straight to step 3 (with \"Nothing\", "
                          "the view still shows and stays).");
    if (then == FPV_KAMIKAZE_THEN_NOTHING)
        ImGui::TextDisabled("With \"Nothing\" the view stays until you act, whatever the duration.");
    else if (skipView)
        ImGui::TextDisabled("0 s: the view is skipped, step 3 comes right after the blast.");
    ImGui::TextUnformatted("3. Then");
    ImGui::Indent();
    for (uint32_t i = 0; i < 3; i++) {
        if (!(L.kamikazeThenMask & (1u << i))) continue;
        if (ImGui::RadioButton(thens[i], then == i) && then != i) {
            st.kamikazeEndingThen = (int)i;
            changed = true;
        }
    }
    ImGui::Unindent();
    ImGui::TextWrapped("While the view shows, respawning, recalling the drone or moving the pilot to it yourself works "
                       "at once and replaces step 3.");
    if (changed) {
        applyKamikazeSettings();
        saveSettings();
    }

    sectionTitle("Bomb switches (radio)");
    ImGui::TextWrapped("Bomb arm (AUX7) and explode (AUX8) on your radio. This only sets up these two switches; the "
                       "rest of the radio setup (Radio tab) stays as it is.");
    std::vector<RawDevice> devs = L.input.devicesSnapshot();
    // a finished (saved) radio setup only waits for "Finish" (closing the
    // overlay keeps it): it doesn't hold the bomb switch setup
    if (m_wizard.active() && !m_wizard.bombSwitchesOnly() && m_wizard.step() == RadioWizard::Step::Done &&
        m_wizardSaved)
        m_wizard.cancel();
    if (m_wizard.active()) {
        if (m_wizard.bombSwitchesOnly()) {
            wizardUi(devs);
        } else {
            ImGui::TextWrapped("A radio is being set up in the Radio tab.");
            if (ImGui::SmallButton("Go there")) m_requestTab = &Overlay::tabRadio;
        }
        return;
    }
    if (devs.empty()) ImGui::TextDisabled("No radio connected.");
    for (auto &d : devs) {
        ImGui::PushID(d.guid.c_str());
        DeviceMapping m;
        bool mapped = L.input.getMapping(d.guid, m) && m.complete;
        ImGui::TextUnformatted(d.name.c_str());
        if (mapped) {
            ImGui::Text("Bomb arm: %s, explode: %s", bindingText(m.channels[kAux7]).c_str(),
                        bindingText(m.channels[kAux8]).c_str());
            if (ImGui::Button("Set up bomb switches")) {
                m_wizard.startBombSwitches(d, m);
                m_wizardHint.clear();
                m_wizardSaved = false;
                L.input.setWizardActive(true);
            }
            if (m.channels[kAux7].axis >= 0 || m.channels[kAux7].button >= 0 || m.channels[kAux8].axis >= 0 ||
                m.channels[kAux8].button >= 0) {
                ImGui::SameLine();
                if (ImGui::Button("Clear")) {
                    RadioWizard::mergeBombSwitches(m, DeviceMapping{}, false);
                    L.input.setMapping(m);
                }
            }
        } else {
            ImGui::TextDisabled(d.isGamepad ? "Gamepad default layout: X / Square bomb arm, right shoulder explode."
                                            : "Not set up yet: set it up in the Radio tab first.");
            if (!d.isGamepad) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Radio tab")) m_requestTab = &Overlay::tabRadio;
            }
        }
        ImGui::PopID();
        ImGui::Separator();
    }
    ImGui::TextDisabled("Keyboard: K bomb arm switch on / off, X (held) explode.");
}
#endif

void Overlay::tabDrone() {
    Lib &L = lib();
    bool armed = L.droneExists && (L.lastFrame.flags & FPV_DRONE_ARMED);
    for (size_t i = 0; i < L.presets.size(); i++) {
        const DronePreset &p = L.presets[i];
        ImGui::PushID((int)i);
        bool sel = (int)i == L.presetIndex;
        if (ImGui::RadioButton(p.name.c_str(), sel) && !sel) {
            if (armed) {
                m_status = "Disarm before switching drones.";
            } else {
                selectPreset(p.fileName, true);
                m_status = "Switched to " + p.name;
            }
        }
        ImGui::PopID();
    }
    if (!m_status.empty()) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "%s", m_status.c_str());
    const DronePreset &p = currentPreset();
    sectionTitle(p.name.c_str());
    ImGui::TextWrapped("%s", p.description.c_str());
    const StateInit &s = p.init;
    float arm = std::sqrt(s.quadMotorPos[0].x * s.quadMotorPos[0].x + s.quadMotorPos[0].z * s.quadMotorPos[0].z);
    ImGui::Text("All-up weight   %.0f g", s.quadMass * 1000);
    ImGui::Text("Motors          %.0f kV", s.motorKV[0]);
    ImGui::Text("Propellers      %.1f\" %d-blade", p.collision.propRadius * 2 / 0.0254f, (int)p.propBladeCount);
    ImGui::Text("Battery         %dS %.0f mAh", (int)s.quadBatCellCount, s.quadBatCapacity);
    ImGui::Text("Motor spacing   %.0f mm diagonal", arm * 2000);
    ImGui::Text("Video           %s", p.vtx.digital ? "digital (HD)" : "analog");
    ImGui::Spacing();
#if defined(__EMSCRIPTEN__)
    ImGui::TextWrapped("Each drone keeps its own Betaflight configuration, saved in this browser.");
    (void)L;
#else
    ImGui::TextWrapped("Physical parameters live in JSON files you can copy and edit: %s/quad (restart the game to "
                       "reload). Each drone keeps its own Betaflight configuration.",
                       L.presetDir.c_str());
#endif

    sectionTitle("Reset");
    // confirmation popups; `what` returns the status line
    auto confirm = [&](const char *button, const char *title, const char *text, fpv_result (*what)(),
                       const char *done) {
        if (ImGui::Button(button)) {
            if (armed) m_status = "Disarm before resetting.";
            else ImGui::OpenPopup(title);
        }
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30);
            ImGui::TextWrapped("%s", text);
            ImGui::PopTextWrapPos();
            ImGui::Spacing();
            if (ImGui::Button("Reset")) {
                what();
                m_bf.invalidate();
                m_status = done;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel") || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
    };
    std::string droneText = "Erase the Betaflight configuration (EEPROM) of \"" + p.name +
                            "\": rates, tuning, OSD, modes and everything changed in the CLI or the Configurator. "
                            "It comes back as this game ships it, and so do its preset files. This cannot be undone.";
    confirm("Reset drone ROM", "Reset drone ROM?", droneText.c_str(), &resetCurrentPreset,
            "Drone reset to libfpv's configuration.");
    ImGui::SameLine();
    confirm("Reset everything", "Reset everything?",
            "Put every libfpv setting back as on the first launch: the Betaflight configuration of every drone, "
            "the shipped drone presets, radio setups, camera, sound, controls and the selected drone. Drone files "
            "you added yourself are kept. This cannot be undone.",
            &resetEverything, "Everything reset to libfpv's defaults.");
}

void Overlay::tabRates() {
    Lib &L = lib();
    m_bf.refresh(wallSeconds());
    static const char *types[] = {"BETAFLIGHT", "RACEFLIGHT", "KISS", "ACTUAL", "QUICK"};
    std::string cur = m_bf.str("rates_type", "ACTUAL");
    int t = 3;
    for (int i = 0; i < 5; i++)
        if (cur == types[i]) t = i;
    if (ImGui::Combo("Rates type", &t, types, 5)) m_bf.set("rates_type", types[t], true);
    static const char *axes[] = {"roll", "pitch", "yaw"};
    ImGui::RadioButton("Roll", &m_rateAxis, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Pitch", &m_rateAxis, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Yaw", &m_rateAxis, 2);
    std::string ax = axes[m_rateAxis];
    // labels and ranges per rates type (as in Betaflight Configurator)
    const char *l1 = "RC rate", *l2 = "Super rate", *l3 = "Expo";
    int max1 = 255, max2 = 100, max3 = 100;
    if (t == 3) l1 = "Center sensitivity (x10 deg/s)", l2 = "Max rate (x10 deg/s)", max2 = 200;
    if (t == 4) l1 = "RC rate (x2 deg/s)", l2 = "Max rate (x10 deg/s)", max2 = 200;
    if (t == 1) l1 = "Rate (x10 deg/s)", l2 = "Acro+", max2 = 255;
    int rc = m_bf.num(ax + "_rc_rate", 7), sr = m_bf.num(ax + "_srate", 67), ex = m_bf.num(ax + "_expo", 0);
    auto slider = [&](const char *label, const std::string &name, int &v, int maxv) {
        if (ImGui::SliderInt(label, &v, 0, maxv)) m_bf.set(name, v, false);
        if (ImGui::IsItemDeactivatedAfterEdit()) m_bf.set(name, v, true);
    };
    slider(l1, ax + "_rc_rate", rc, max1);
    slider(l2, ax + "_srate", sr, max2);
    slider(l3, ax + "_expo", ex, max3);
    if (!m_bf.lastError().empty()) ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "%s", m_bf.lastError().c_str());

    RateParams rp;
    rp.type = t;
    rp.rcRate = (float)rc;
    rp.rate = (float)sr;
    rp.expo = (float)ex;
    rp.limit = (float)m_bf.num(ax + "_rate_limit", 1998);
    rp.quickRcExpo = m_bf.str("quickrates_rc_expo", "OFF") == "ON";
    float maxRate = betaflightRate(rp, 1.0f);
    ImGui::Text("Max rate: %.0f deg/s", maxRate);

    // live curve with the current stick position
    ImVec2 size(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 12);
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImDrawList *dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), IM_COL32(20, 24, 30, 255));
    float top = std::max(200.0f, std::ceil(maxRate / 200.0f) * 200.0f);
    for (float g = 200; g < top; g += 200) {
        float y = p0.y + size.y * (1 - g / top);
        dl->AddLine(ImVec2(p0.x, y), ImVec2(p0.x + size.x, y), IM_COL32(60, 60, 70, 255));
    }
    ImVec2 prevPt;
    for (int i = 0; i <= 100; i++) {
        float x = i / 100.0f;
        ImVec2 pt(p0.x + x * size.x, p0.y + size.y * (1 - betaflightRate(rp, x) / top));
        if (i) dl->AddLine(prevPt, pt, IM_COL32(90, 200, 255, 255), 2.0f);
        prevPt = pt;
    }
    int chIdx[3] = {0, 1, 3};
    float stick = std::fabs(L.lastFrame.channels[chIdx[m_rateAxis]]);
    ImVec2 dot(p0.x + stick * size.x, p0.y + size.y * (1 - betaflightRate(rp, stick) / top));
    dl->AddCircleFilled(dot, 5, IM_COL32(255, 200, 60, 255));
    ImGui::Dummy(size);
    ImGui::TextDisabled("Grid: 200 deg/s. The dot follows your %s stick.", axes[m_rateAxis]);

    sectionTitle("Throttle");
    int mid = m_bf.num("thr_mid", 50), expo = m_bf.num("thr_expo", 0);
    slider("Throttle mid", "thr_mid", mid, 100);
    slider("Throttle expo", "thr_expo", expo, 100);
}

void Overlay::tabTuning() {
    m_bf.refresh(wallSeconds());
    ImGui::TextWrapped("Betaflight's simplified tuning. Sliders apply instantly; 100 is the default.");
    struct S {
        const char *label, *name;
        int lo, hi;
    } static const sliders[] = {
        {"Master multiplier", "simplified_master_multiplier", 0, 200},
        {"Damping (D gain)", "simplified_d_gain", 0, 200},
        {"Tracking (P & I gain)", "simplified_pi_gain", 0, 200},
        {"Stick response (feedforward)", "simplified_feedforward_gain", 0, 200},
        {"Dynamic damping (D max)", "simplified_d_max_gain", 0, 200},
        {"Drift / wobble (I gain)", "simplified_i_gain", 0, 200},
        {"Pitch damping", "simplified_pitch_d_gain", 0, 200},
        {"Pitch tracking", "simplified_pitch_pi_gain", 0, 200},
    };
    bool changed = false, release = false;
    for (auto &s : sliders) {
        int v = m_bf.num(s.name, 100);
        if (ImGui::SliderInt(s.label, &v, s.lo, s.hi)) {
            m_bf.set(s.name, v, false);
            changed = true;
        }
        if (ImGui::IsItemDeactivatedAfterEdit()) release = true;
    }
    sectionTitle("Filters");
    int gyro = m_bf.num("simplified_gyro_filter_multiplier", 100), dterm = m_bf.num("simplified_dterm_filter_multiplier", 100);
    if (ImGui::SliderInt("Gyro filter multiplier", &gyro, 10, 200)) {
        m_bf.set("simplified_gyro_filter_multiplier", gyro, false);
        changed = true;
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) release = true;
    if (ImGui::SliderInt("D-term filter multiplier", &dterm, 10, 200)) {
        m_bf.set("simplified_dterm_filter_multiplier", dterm, false);
        changed = true;
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) release = true;
    if (changed || release) m_bf.run(std::string("simplified_tuning apply\n") + (release ? "save\n" : ""));
    sectionTitle("Angle mode");
    int angle = m_bf.num("angle_limit", 60);
    if (ImGui::SliderInt("Max angle", &angle, 10, 85)) m_bf.set("angle_limit", angle, false);
    if (ImGui::IsItemDeactivatedAfterEdit()) m_bf.set("angle_limit", angle, true);
    if (ImGui::Button("Reset tuning to defaults")) {
        for (auto &s : sliders) m_bf.set(s.name, 100, false);
        m_bf.set("simplified_gyro_filter_multiplier", 100, false);
        m_bf.set("simplified_dterm_filter_multiplier", 100, false);
        m_bf.run("simplified_tuning apply\nsave\n");
    }
    ImGui::TextDisabled("Full PID and filter control: Betaflight Configurator (see its tab).");
}

void Overlay::tabOsd() {
    Lib &L = lib();
    m_bf.refresh(wallSeconds());
    int style = L.settings.osdStyle;
    if (ImGui::RadioButton("Analog look", style == FPV_OSD_STYLE_ANALOG)) style = FPV_OSD_STYLE_ANALOG;
    ImGui::SameLine();
    if (ImGui::RadioButton("HD look", style == FPV_OSD_STYLE_HD)) style = FPV_OSD_STYLE_HD;
    if (style != L.settings.osdStyle) {
        L.settings.osdStyle = style;
        saveSettings();
    }
    struct E {
        const char *label, *name;
    } static const elems[] = {
        {"Battery voltage", "osd_vbat_pos"},     {"Cell voltage", "osd_avg_cell_voltage_pos"},
        {"Current", "osd_current_pos"},          {"mAh used", "osd_mah_drawn_pos"},
        {"Flight time", "osd_tim_2_pos"},        {"On time", "osd_tim_1_pos"},
        {"Flight mode", "osd_flymode_pos"},      {"Throttle", "osd_throttle_pos"},
        {"Crosshairs", "osd_crosshairs_pos"},    {"Artificial horizon", "osd_ah_pos"},
        {"Horizon sidebars", "osd_ah_sbar_pos"}, {"Warnings", "osd_warnings_pos"},
        {"Craft name", "osd_craft_name_pos"},    {"Speed", "osd_gps_speed_pos"},
        {"Altitude", "osd_altitude_pos"},        {"Disarmed", "osd_disarmed_pos"},
        {"G-force", "osd_g_force_pos"},          {"Power", "osd_power_pos"},
        {"Stick overlay L", "osd_stick_overlay_left_pos"}, {"Stick overlay R", "osd_stick_overlay_right_pos"},
#if defined(LIBFPV_KAMIKAZE)
        {"Bomb status", "osd_custom_msg0_pos"},  // the warhead state (Flight::step)
#endif
    };
    const int n = (int)(sizeof(elems) / sizeof(elems[0]));
    ImGui::BeginChild("elems", ImVec2(ImGui::GetFontSize() * 13, 0), ImGuiChildFlags_Borders);
    for (int i = 0; i < n; i++) {
        int pos = m_bf.num(elems[i].name, 0);
        bool vis = (pos & 0x800) != 0;
        ImGui::PushID(i);
        if (ImGui::Checkbox("##v", &vis)) m_bf.set(elems[i].name, vis ? (pos | 0x800) : (pos & ~0x800), true);
        ImGui::SameLine();
        if (ImGui::Selectable(elems[i].label, m_osdSelected == i)) m_osdSelected = i;
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::TextWrapped("Select an element, then click where it should go. Live preview:");
    // preview grid drawn with the OSD font texture
    float cw = ImGui::GetFontSize() * 0.9f, chh = cw * 1.5f;
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 size(cw * FPV_OSD_COLS, chh * FPV_OSD_ROWS);
    ImDrawList *dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), IM_COL32(60, 70, 60, 255));
    const float aw = (float)OsdRenderer::kAtlasW, ah = (float)OsdRenderer::kAtlasH;
    int fontOff = style == FPV_OSD_STYLE_HD ? OsdRenderer::kGlyphW * 16 : 0;
    for (int r = 0; r < FPV_OSD_ROWS; r++) {
        for (int c = 0; c < FPV_OSD_COLS; c++) {
            uint8_t g = L.lastFrame.osd[r * FPV_OSD_COLS + c];
            if (g == 0 || g == ' ') continue;
            ImVec2 a(p0.x + c * cw, p0.y + r * chh), b(a.x + cw, a.y + chh);
            ImVec2 uv0(((g % 16) * OsdRenderer::kGlyphW + fontOff) / aw, (g / 16) * OsdRenderer::kGlyphH / ah);
            ImVec2 uv1(uv0.x + OsdRenderer::kGlyphW / aw, uv0.y + OsdRenderer::kGlyphH / ah);
            dl->AddImage(ImTextureRef((ImTextureID)FPV_TEXTURE_OSD_FONT), a, b, uv0, uv1);
        }
    }
    ImGui::InvisibleButton("osdgrid", size);
    if (ImGui::IsItemClicked()) {
        ImVec2 m = ImGui::GetIO().MousePos;
        int c = std::clamp((int)((m.x - p0.x) / cw), 0, FPV_OSD_COLS - 1);
        int r = std::clamp((int)((m.y - p0.y) / chh), 0, FPV_OSD_ROWS - 1);
        int pos = c | (r << 5) | 0x800;
        m_bf.set(elems[m_osdSelected].name, pos, true);
    }
    int pos = m_bf.num(elems[m_osdSelected].name, 0);
    ImGui::Text("%s: column %d, row %d%s", elems[m_osdSelected].label, pos & 31, (pos >> 5) & 31,
                (pos & 0x800) ? "" : " (hidden)");
    ImGui::EndGroup();
}

void Overlay::tabImport() {
    ImGui::TextWrapped(
        "Paste the output of `diff all` from your real quad's Betaflight CLI. Hardware lines (resources, serial, "
        "motor protocol, VTX...) are skipped, ARM moves to AUX1 and other switches to AUX2-4. The rest of your "
        "tune is applied to the current drone.");
    static char buf[64 * 1024];
    if (m_importText.size() < sizeof(buf)) std::memcpy(buf, m_importText.c_str(), m_importText.size() + 1);
    if (ImGui::InputTextMultiline("##diff", buf, sizeof buf, ImVec2(-1, ImGui::GetFontSize() * 10)))
        m_importText = buf;
    if (ImGui::Button("Import")) {
        char *report = nullptr;
        fpv_result r = fpv_import_betaflight_diff(m_importText.c_str(), &report);
        m_importReport = r == FPV_OK ? (report ? report : "") : std::string("Import failed: ") + fpv_last_error();
        fpv_free(report);
        m_bf.invalidate();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) m_importText.clear();
#if defined(__EMSCRIPTEN__)
    ImGui::SameLine();
    if (ImGui::Button("Paste")) libfpv_web_clipboard_read(1);
#endif
    if (!m_importReport.empty()) {
        ImGui::BeginChild("report", ImVec2(0, ImGui::GetFontSize() * 10), ImGuiChildFlags_Borders);
        ImGui::TextUnformatted(m_importReport.c_str());
        ImGui::EndChild();
    }
    sectionTitle("Betaflight CLI");
    static char line[256];
    bool enter = ImGui::InputText("##cli", line, sizeof line, ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if (ImGui::Button("Run") || enter) {
        char *out = nullptr;
        fpv_cli_exec(line, &out);
        m_cliOutput = out ? out : fpv_last_error();
        fpv_free(out);
        line[0] = 0;
        m_bf.invalidate();
    }
    if (!m_cliOutput.empty()) {
        ImGui::BeginChild("cliout", ImVec2(0, 0), ImGuiChildFlags_Borders);
        ImGui::TextUnformatted(m_cliOutput.c_str());
        ImGui::EndChild();
    }
}

void Overlay::tabCamera() {
    Lib &L = lib();
    const DronePreset &p = currentPreset();
    float tilt = L.settings.cameraUptilt >= 0 ? L.settings.cameraUptilt : p.camera.uptiltDeg;
    float fov = L.settings.cameraFov > 0 ? L.settings.cameraFov : p.camera.fovYDeg;
    bool changed = ImGui::SliderFloat("Camera uptilt (deg)", &tilt, 0, 70, "%.0f");
    bool done = ImGui::IsItemDeactivatedAfterEdit();
    changed |= ImGui::SliderFloat("Field of view (vertical, deg)", &fov, 60, 150, "%.0f");
    done |= ImGui::IsItemDeactivatedAfterEdit();
    if (changed) {
        L.settings.cameraUptilt = tilt;
        L.settings.cameraFov = fov;
    }
    if (L.config.flags & FPV_INIT_NO_LENS_DISTORTION) {
        ImGui::TextDisabled("Fisheye lens: not drawn by this game");
    } else {
        ImGui::SliderFloat("Fisheye lens", &L.settings.cameraFisheye, 0, 1, L.settings.cameraFisheye > 0 ? "%.2f" : "off");
        done |= ImGui::IsItemDeactivatedAfterEdit();
    }
    if (done) saveSettings();
    if (ImGui::Button("Use the drone's defaults")) {
        L.settings.cameraUptilt = -1;
        L.settings.cameraFov = -1;
        L.settings.cameraFisheye = Settings{}.cameraFisheye;
        saveSettings();
    }
    sectionTitle("Sound");
    if (ImGui::SliderFloat("Motor sound volume", &L.settings.audioVolume, 0, 1.5f)) fpv_audio_set_volume(L.settings.audioVolume);
    sectionTitle("Video link");
    const fpv_frame &f = L.lastFrame;
    ImGui::Text("%s link, signal %.0f%%, noise %.0f%%", f.video.is_digital ? "Digital" : "Analog",
                f.video.signal_strength * 100, f.video.noise * 100);
    ImGui::TextDisabled("Noise grows with distance from the pilot and behind obstacles.");
}

void Overlay::tabControls() {
    Lib &L = lib();
    ImGui::Text("Open/close this overlay: %s (Esc closes it too)", keyName(L.settings.overlayToggleKey));
    ImGui::SameLine();
    if (m_rebindingToggle) {
        ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "press a key (Esc cancels)");
    } else if (ImGui::Button("Change")) {
        m_rebindingToggle = true;
    }
    sectionTitle("Keyboard (degraded fallback)");
    bool radio = L.input.status().radioDetected;
    bool hoverOn = L.settings.keyboardHover == KeyboardHover::On ||
                   (L.settings.keyboardHover == KeyboardHover::Auto && !radio);
    if (hoverOn)
        ImGui::BulletText("W: climb, S: cut throttle, neither (or both): hover");
    else
        ImGui::BulletText("W / S: throttle up / down");
    ImGui::BulletText("A / D: yaw (grows while held)");
    ImGui::BulletText("Arrow keys: pitch and roll (self-level mode)");
    ImGui::BulletText("Enter: arm / disarm, B: beeper, T: turtle mode on / off");
    const char *hoverModes[] = {"Auto", "On", "Off"};
    int hoverMode = (int)L.settings.keyboardHover;
    if (ImGui::Combo("Automatic hover throttle", &hoverMode, hoverModes, 3)) {
        L.settings.keyboardHover = (KeyboardHover)hoverMode;
        L.input.setKeyboardHover(L.settings.keyboardHover);
        saveSettings();
    }
    if (L.settings.keyboardHover == KeyboardHover::Auto)
        ImGui::TextDisabled(radio ? "Auto: off, a radio is connected." : "Auto: on while no radio is connected.");
    if (hoverOn) {
        if (ImGui::SliderFloat("W throttle exponent", &L.settings.keyboardThrottleExpo, 0.5f, 10.0f, "%.1f"))
            L.input.setKeyboardThrottleExpo(L.settings.keyboardThrottleExpo);
        if (ImGui::IsItemDeactivatedAfterEdit()) saveSettings();
        ImGui::TextDisabled("How fast W raises the throttle from hover to full. The hover throttle is learned in flight.");
    }
    if (ImGui::SliderFloat("A / D yaw exponent", &L.settings.keyboardYawExpo, 0.5f, 20.0f, "%.1f"))
        L.input.setKeyboardYawExpo(L.settings.keyboardYawExpo);
    if (ImGui::IsItemDeactivatedAfterEdit()) saveSettings();
    ImGui::TextDisabled("How fast A / D raise the yaw from a slow turn to full.");
    sectionTitle("Gamepad (degraded fallback)");
    ImGui::BulletText("Left stick up: throttle (centre = idle), left stick sideways: yaw");
    ImGui::BulletText("Right stick: pitch and roll (self-level mode)");
    ImGui::BulletText("A / Cross: arm / disarm, B / Circle: beeper, Y / Triangle: turtle mode on / off");
    ImGui::TextWrapped("For real FPV flying use a radio in USB joystick mode: see the Radio tab.");
}

void Overlay::tabConfigurator() {
    fpv_configurator_info info{};
    info.struct_size = sizeof(info);
    fpv_configurator_get_info(&info);
    if (!info.enabled) {
#if defined(__EMSCRIPTEN__)
        ImGui::TextWrapped(
            "Betaflight Configurator cannot connect to a game running in a web browser: browsers do not let a page "
            "accept connections. Every setting is still Betaflight's own: use the other tabs, the Betaflight CLI or "
            "a `diff all` in Import & CLI, or the desktop version of this game for the Configurator.");
#else
        ImGui::TextWrapped("The Configurator connection is disabled by the game.");
#endif
        return;
    }
    ImGui::Text("Address: %s", info.url);
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy")) ImGui::SetClipboardText(info.url);
    ImGui::Text("Connected: %s", info.client_count > 0 ? "yes" : "no");
    sectionTitle("Connect Betaflight Configurator");
    ImGui::TextWrapped("1. Open Betaflight Configurator (app.betaflight.com or the desktop app).");
    ImGui::TextWrapped("2. In Options, enable \"manual connection mode\".");
    ImGui::TextWrapped("3. Choose \"Manual Selection\", enter %s as the port and press Connect.", info.url);
    ImGui::TextWrapped("Everything you change there is the same configuration as this overlay. Keep the motor "
                       "protocol on PWM and ARM on AUX1.");
    if (ImGui::Button("Reboot Betaflight")) {
        lib().runner.call([] { bfb_reboot(); });
    }
}

// --- frame -------------------------------------------------------------------------

void Overlay::buildUi() {
    Lib &L = lib();
    ImGuiIO &io = ImGui::GetIO();
    ImVec2 size(std::min(m_w - 20, 984 * m_scale), std::min(m_h - 20, 744 * m_scale));
    ImGui::SetNextWindowPos(ImVec2(m_w * 0.5f, m_h * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(size, ImGuiCond_Appearing);
    bool open = true;
    if (ImGui::Begin("libfpv - FPV drone", &open, ImGuiWindowFlags_NoCollapse)) {
        if (ImGui::BeginTabBar("tabs")) {
            struct Tab {
                const char *name;
                void (Overlay::*fn)();
            } static const tabs[] = {{"Fly", &Overlay::tabFly},
                                     {"Radio", &Overlay::tabRadio},
#if defined(LIBFPV_KAMIKAZE)
                                     {"Kamikaze", &Overlay::tabKamikaze},
#endif
                                     {"Drone", &Overlay::tabDrone},
                                     {"Rates", &Overlay::tabRates},
                                     {"Tuning", &Overlay::tabTuning},
                                     {"OSD", &Overlay::tabOsd},
                                     {"Import & CLI", &Overlay::tabImport},
                                     {"Camera & sound", &Overlay::tabCamera},
                                     {"Controls", &Overlay::tabControls},
                                     {"Configurator", &Overlay::tabConfigurator}};
            // a tab may ask for another one while it draws: kept for the next frame
            TabFn request = m_requestTab;
            m_requestTab = nullptr;
            for (const Tab &t : tabs) {
                ImGuiTabItemFlags fl = request == t.fn ? ImGuiTabItemFlags_SetSelected : 0;
                if (ImGui::BeginTabItem(t.name, nullptr, fl)) {
                    (this->*t.fn)();
                    ImGui::EndTabItem();
                }
            }
            ImGui::EndTabBar();
        }
        if (m_bf.busy())
            ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1),
                               "The Configurator is in CLI mode: values here are paused until you leave its CLI tab.");
    }
    ImGui::End();
    (void)io;
    if (!open) setOpen(false);
    (void)L;
}

const fpv_draw_data *Overlay::convert() {
    m_draw.clear();
    ImDrawData *dd = ImGui::GetDrawData();
    if (!dd) return m_draw.finish(m_w, m_h);
    for (int n = 0; n < dd->CmdLists.Size; n++) {
        const ImDrawList *cl = dd->CmdLists[n];
        DrawListStorage &s = m_draw.newList();
        s.vertices.resize(cl->VtxBuffer.Size);
        static_assert(sizeof(ImDrawVert) == sizeof(fpv_draw_vertex), "vertex layout");
        std::memcpy(s.vertices.data(), cl->VtxBuffer.Data, sizeof(ImDrawVert) * cl->VtxBuffer.Size);
        s.indices.resize(cl->IdxBuffer.Size);
        for (int i = 0; i < cl->IdxBuffer.Size; i++) s.indices[i] = cl->IdxBuffer.Data[i];
        for (int c = 0; c < cl->CmdBuffer.Size; c++) {
            const ImDrawCmd &pc = cl->CmdBuffer[c];
            if (pc.UserCallback || pc.ElemCount == 0) continue;
            fpv_draw_cmd o{};
            o.clip_rect[0] = pc.ClipRect.x - dd->DisplayPos.x;
            o.clip_rect[1] = pc.ClipRect.y - dd->DisplayPos.y;
            o.clip_rect[2] = pc.ClipRect.z - dd->DisplayPos.x;
            o.clip_rect[3] = pc.ClipRect.w - dd->DisplayPos.y;
            o.texture_id = (uint64_t)pc.GetTexID();
            o.index_offset = pc.IdxOffset;
            o.index_count = pc.ElemCount;
            o.vertex_offset = pc.VtxOffset;
            s.commands.push_back(o);
        }
    }
    return m_draw.finish(m_w, m_h);
}

const fpv_draw_data *Overlay::render(float dt) {
    Lib &L = lib();
    // a newly connected, unmapped radio opens the setup wizard once
    InputStatus st = L.input.status();
    if (st.needsSetup) {
        for (auto &d : L.input.devicesSnapshot()) {
            DeviceMapping m;
            if (d.looksLikeRadio && !L.input.getMapping(d.guid, m)) notifyUnmappedRadio(d.guid);
        }
    }
    if (!m_open) return nullptr;
    WITH_CTX;
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(m_w, m_h);
    io.DeltaTime = (dt > 0 && std::isfinite(dt)) ? std::min(dt, 0.25f) : 1.0f / 60.0f;
    ImGui::GetStyle().FontScaleMain = m_scale;
    m_time += io.DeltaTime;
    ImGui::NewFrame();
    buildUi();
    ImGui::Render();
    const fpv_draw_data *out = convert();
    END_CTX;
    return out;
}

}  // namespace fpv

#if defined(__EMSCRIPTEN__)
// Clipboard text from libfpv_web_clipboard_read (src/web/libfpv_web.js).
extern "C" FPV_API EMSCRIPTEN_KEEPALIVE void libfpv_web_on_paste(const char *utf8, int toImport) {
    fpv::Lib &L = fpv::lib();
    std::lock_guard<std::recursive_mutex> lock(L.mutex);
    if (L.initialized && utf8) fpv::overlay().paste(utf8, toImport != 0);
}
#endif
