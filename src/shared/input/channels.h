// libfpv - flight channels as Betaflight sees them, and the interface the
// simulation reads them through (the input manager, on the client side).
// Shared by the client and the simulation server (docs/DECISIONS.md, D37).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <string>

namespace fpv {

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

// What the keyboard's hover hold needs to know about the drone, taken from
// the previous simulation step.
struct HoverFeedback {
    std::string preset;     // the learned hover throttle is kept per preset
    float climbRate = 0;    // m/s, up positive
    float upright = 1;      // cosine of the tilt from level (< 0 upside down)
    bool airborne = false;  // a drone exists and nothing touches the ground
};

// Where the simulation gets its channels each step. The simulation calls it
// on its own thread: devices are started, polled and stopped there.
class ChannelSource {
public:
    virtual ~ChannelSource() = default;
    virtual bool startDevices(std::string &error) = 0;
    virtual void stopDevices() = 0;
    // The previous step's drone, for the keyboard's hover hold.
    virtual void setHoverFeedback(const HoverFeedback &f) = 0;
    // Channels for one step of dt seconds; `radioLost` puts Betaflight into failsafe.
    virtual void update(double wallTime, float dt, float out[kChannelCount], bool &radioLost) = 0;
    // After the radio vanished, failsafe holds until Betaflight disarmed.
    virtual void resetRadioLatch() = 0;
};

}  // namespace fpv
