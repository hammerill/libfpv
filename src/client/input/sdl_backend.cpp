// libfpv - SDL3 device backend (radios in USB joystick mode, gamepads).
//
// SDL is statically linked and hidden inside libfpv; it is initialised and
// polled on the simulation thread only. Hints keep it from interfering
// with a game that has its own input stack:
//  - background events: the game window owns the focus, not us;
//  - no Raw Input / WGI / GameInput on Windows: Raw Input registrations are
//    per process, and a second SDL registering would steal the game's;
//    DirectInput (radios) + XInput/HIDAPI (gamepads) need no registration;
//  - no signal handlers.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "input/input.h"

#include <algorithm>
#include <map>

#include "util/log.h"

#if defined(LIBFPV_WITH_SDL)
#include <SDL3/SDL.h>
#endif

namespace fpv {

#if defined(LIBFPV_WITH_SDL)

namespace {

class SdlBackend : public DeviceBackend {
public:
    bool init(std::string &error) override {
        SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
        SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
        SDL_SetHint(SDL_HINT_JOYSTICK_RAWINPUT, "0");
        SDL_SetHint(SDL_HINT_JOYSTICK_WGI, "0");
        SDL_SetHint(SDL_HINT_JOYSTICK_GAMEINPUT, "0");
        if (!SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMEPAD)) {
            error = SDL_GetError();
            return false;
        }
        // We poll state; no events to pile up at 1 kHz.
        SDL_SetJoystickEventsEnabled(false);
        SDL_SetGamepadEventsEnabled(false);
        FPV_LOGI("device input: SDL %d.%d.%d", SDL_MAJOR_VERSION, SDL_MINOR_VERSION, SDL_MICRO_VERSION);
        return true;
    }

    void shutdown() override {
        for (auto &kv : m_open) close(kv.second);
        m_open.clear();
        SDL_QuitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMEPAD);
    }

    void poll(std::vector<RawDevice> &out) override {
        SDL_UpdateJoysticks();
        int n = 0;
        SDL_JoystickID *ids = SDL_GetJoysticks(&n);
        // open new devices
        for (int i = 0; i < n; i++) {
            if (m_open.count(ids[i])) continue;
            Dev d;
            if (SDL_IsGamepad(ids[i])) {
                d.pad = SDL_OpenGamepad(ids[i]);
                d.joy = d.pad ? SDL_GetGamepadJoystick(d.pad) : nullptr;
            } else {
                d.joy = SDL_OpenJoystick(ids[i]);
            }
            if (!d.joy) continue;
            char guid[64];
            SDL_GUIDToString(SDL_GetJoystickGUID(d.joy), guid, sizeof guid);
            d.guid = guid;
            const char *name = SDL_GetJoystickName(d.joy);
            d.name = name ? name : "Joystick";
            int axes = SDL_GetNumJoystickAxes(d.joy);
            d.radio = nameLooksLikeRadio(d.name) || (!d.pad && axes >= 4);
            FPV_LOGI("input device connected: %s (%d axes, %s)", d.name.c_str(), axes,
                     d.radio ? "radio" : (d.pad ? "gamepad" : "joystick"));
            m_open[ids[i]] = d;
        }
        // close removed devices
        for (auto it = m_open.begin(); it != m_open.end();) {
            bool present = false;
            for (int i = 0; i < n; i++) present |= ids[i] == it->first;
            if (!present || !SDL_JoystickConnected(it->second.joy)) {
                FPV_LOGI("input device disconnected: %s", it->second.name.c_str());
                close(it->second);
                it = m_open.erase(it);
            } else {
                ++it;
            }
        }
        SDL_free(ids);

        out.resize(m_open.size());
        size_t k = 0;
        for (auto &kv : m_open) {
            Dev &d = kv.second;
            RawDevice &r = out[k++];
            r.instanceId = kv.first;
            r.guid = d.guid;
            r.name = d.name;
            r.isGamepad = d.pad != nullptr && !d.radio;
            r.looksLikeRadio = d.radio;
            int na = SDL_GetNumJoystickAxes(d.joy), nb = SDL_GetNumJoystickButtons(d.joy);
            r.axes.resize(std::max(0, na));
            for (int a = 0; a < na; a++) r.axes[a] = std::clamp(SDL_GetJoystickAxis(d.joy, a) / 32767.0f, -1.0f, 1.0f);
            r.buttons.resize(std::max(0, nb));
            for (int b = 0; b < nb; b++) r.buttons[b] = SDL_GetJoystickButton(d.joy, b) ? 1 : 0;
            if (d.pad) {
                static const SDL_GamepadAxis ax[6] = {SDL_GAMEPAD_AXIS_LEFTX,  SDL_GAMEPAD_AXIS_LEFTY,
                                                      SDL_GAMEPAD_AXIS_RIGHTX, SDL_GAMEPAD_AXIS_RIGHTY,
                                                      SDL_GAMEPAD_AXIS_LEFT_TRIGGER, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER};
                for (int a = 0; a < 6; a++) r.pad[a] = std::clamp(SDL_GetGamepadAxis(d.pad, ax[a]) / 32767.0f, -1.0f, 1.0f);
                r.padButtons = 0;
                for (int b = 0; b < SDL_GAMEPAD_BUTTON_COUNT && b < 32; b++)
                    if (SDL_GetGamepadButton(d.pad, (SDL_GamepadButton)b)) r.padButtons |= 1u << b;
            }
        }
    }

private:
    struct Dev {
        SDL_Joystick *joy = nullptr;
        SDL_Gamepad *pad = nullptr;
        std::string guid, name;
        bool radio = false;
    };
    static void close(Dev &d) {
        if (d.pad) SDL_CloseGamepad(d.pad);
        else if (d.joy) SDL_CloseJoystick(d.joy);
        d.pad = nullptr;
        d.joy = nullptr;
    }
    std::map<SDL_JoystickID, Dev> m_open;
};

}  // namespace

std::unique_ptr<DeviceBackend> createSdlBackend() { return std::make_unique<SdlBackend>(); }

#else

std::unique_ptr<DeviceBackend> createSdlBackend() { return nullptr; }

#endif

}  // namespace fpv
