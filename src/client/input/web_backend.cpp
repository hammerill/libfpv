// libfpv - browser device backend: the Gamepad API (Emscripten builds).
//
// Radios in USB joystick mode and gamepads are both HID devices to the
// browser. A device with the "standard" mapping has the fixed gamepad
// layout; anything else (radios, simulator dongles) gives raw axes, as SDL
// does for a joystick. Browsers only list a device after the player pressed
// a button or moved a stick on it, and only on the page's main thread,
// which is where cooperative timing polls it (sim/runner.h).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "input/input.h"

#include "util/log.h"

#if defined(__EMSCRIPTEN__)
#include <emscripten/html5.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <map>
#endif

namespace fpv {

#if defined(__EMSCRIPTEN__)

namespace {

// "Name (STANDARD GAMEPAD Vendor: 045e Product: 028e)" (Chromium) or
// "045e-028e-Name" (Firefox) -> "Name"
std::string displayName(const char *id) {
    std::string n = id;
    size_t paren = n.find(" (");
    if (paren != std::string::npos && n.find("Vendor:", paren) != std::string::npos) n.erase(paren);
    if (n.size() > 10 && n[4] == '-' && n[9] == '-') {
        bool hex = true;
        for (int i : {0, 1, 2, 3, 5, 6, 7, 8}) hex = hex && std::isxdigit((unsigned char)n[i]);
        if (hex) n.erase(0, 10);
    }
    return n.empty() ? "Gamepad" : n;
}

// W3C standard gamepad buttons -> SDL_GamepadButton bits (RawDevice::padButtons)
constexpr int kStandardToSdl[17] = {0, 1, 2, 3, 9, 10, -1, -1, 4, 6, 7, 8, 11, 12, 13, 14, 5};

class WebBackend : public DeviceBackend {
public:
    bool init(std::string &error) override {
        if (emscripten_sample_gamepad_data() != EMSCRIPTEN_RESULT_SUCCESS) {
            error = "this browser has no Gamepad API";
            return false;
        }
        FPV_LOGI("device input: browser Gamepad API (press a button or move a stick on a radio to connect it)");
        return true;
    }

    void shutdown() override { m_known.clear(); }

    void poll(std::vector<RawDevice> &out) override {
        out.clear();
        if (emscripten_sample_gamepad_data() != EMSCRIPTEN_RESULT_SUCCESS) return;
        int n = emscripten_get_num_gamepads();
        std::map<int, std::string> seen;
        for (int i = 0; i < n; i++) {
            EmscriptenGamepadEvent g;
            if (emscripten_get_gamepad_status(i, &g) != EMSCRIPTEN_RESULT_SUCCESS || !g.connected) continue;
            RawDevice r;
            r.instanceId = (uint32_t)i + 1;
            r.guid = std::string("web:") + g.id;
            r.name = displayName(g.id);
            bool standard = std::strcmp(g.mapping, "standard") == 0;
            r.looksLikeRadio = nameLooksLikeRadio(r.name) || (!standard && g.numAxes >= 4);
            r.isGamepad = standard && !r.looksLikeRadio;
            int na = std::clamp(g.numAxes, 0, 64), nb = std::clamp(g.numButtons, 0, 64);
            r.axes.resize(na);
            for (int a = 0; a < na; a++) r.axes[a] = std::clamp((float)g.axis[a], -1.0f, 1.0f);
            r.buttons.resize(nb);
            for (int b = 0; b < nb; b++) r.buttons[b] = g.digitalButton[b] ? 1 : 0;
            if (standard) {
                for (int a = 0; a < 4 && a < na; a++) r.pad[a] = r.axes[a];
                r.pad[4] = nb > 6 ? (float)g.analogButton[6] : 0.0f;
                r.pad[5] = nb > 7 ? (float)g.analogButton[7] : 0.0f;
                for (int b = 0; b < 17 && b < nb; b++)
                    if (g.digitalButton[b] && kStandardToSdl[b] >= 0) r.padButtons |= 1u << kStandardToSdl[b];
            }
            if (!m_known.count(i) || m_known[i] != r.guid) {
                FPV_LOGI("input device connected: %s (%d axes, %s)", r.name.c_str(), na,
                         r.looksLikeRadio ? "radio" : (r.isGamepad ? "gamepad" : "joystick"));
            }
            seen[i] = r.guid;
            out.push_back(std::move(r));
        }
        for (auto &kv : m_known)
            if (!seen.count(kv.first) || seen[kv.first] != kv.second)
                FPV_LOGI("input device disconnected: %s", displayName(kv.second.c_str() + 4).c_str());
        m_known = std::move(seen);
    }

private:
    std::map<int, std::string> m_known;  // gamepad index -> guid
};

}  // namespace

std::unique_ptr<DeviceBackend> createWebBackend() { return std::make_unique<WebBackend>(); }

#else

std::unique_ptr<DeviceBackend> createWebBackend() { return nullptr; }

#endif

}  // namespace fpv
