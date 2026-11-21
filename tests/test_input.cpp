// The input device the pilot picks in the Radio tab ("Fly with").
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <filesystem>
#include <memory>
#include <vector>

#include "input/input.h"

static int g_failures = 0, g_checks = 0;
#define CHECK(c, ...) \
    do { \
        g_checks++; \
        if (!(c)) { \
            g_failures++; \
            std::printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__); \
            std::printf("\n"); \
        } \
    } while (0)

namespace fpv {
// No SDL or browser input in this test.
std::unique_ptr<DeviceBackend> createSdlBackend() { return nullptr; }
std::unique_ptr<DeviceBackend> createWebBackend() { return nullptr; }
}  // namespace fpv

using namespace fpv;

struct FakeBackend : DeviceBackend {
    std::vector<RawDevice> *devices;
    explicit FakeBackend(std::vector<RawDevice> *d) : devices(d) {}
    bool init(std::string &) override { return true; }
    void shutdown() override {}
    void poll(std::vector<RawDevice> &out) override { out = *devices; }
};

int main() {
    namespace fs = std::filesystem;
    fs::path dir = fs::temp_directory_path() / "libfpv-input-test";
    fs::remove_all(dir);
    fs::create_directories(dir);

    RawDevice radio, pad;
    radio.guid = "radio-guid";
    radio.name = "FrSky Simulator";
    radio.looksLikeRadio = true;
    radio.axes.assign(8, 0.0f);
    pad.guid = "pad-guid";
    pad.name = "Virtual Gamepad";
    pad.isGamepad = true;
    std::vector<RawDevice> devices = {radio, pad};

    float out[kChannelCount];
    bool lost = false;
    {
        InputManager im;
        im.setDataDir(dir.string());
        im.setBackend(std::make_unique<FakeBackend>(&devices));
        im.update(1.0, 0.01f, out, lost);
        CHECK(im.status().source == InputSource::Gamepad, "automatic: the gamepad flies the unmapped radio's place");

        im.setPreferred(radio.guid, radio.name);
        im.update(1.1, 0.01f, out, lost);
        InputStatus st = im.status();
        CHECK(st.source != InputSource::Gamepad, "chosen radio: the gamepad no longer flies");
        CHECK(!st.preferredMissing, "chosen radio connected");
        CHECK(st.needsSetup, "chosen radio still needs setup");

        devices = {pad};  // the radio's dongle drops out
        im.update(1.2, 0.01f, out, lost);
        st = im.status();
        CHECK(st.preferredMissing && st.preferredName == "FrSky Simulator", "missing radio reported: '%s'",
              st.preferredName.c_str());
        CHECK(st.source == InputSource::Keyboard, "only the keyboard takes over, not the gamepad");
    }
    {
        InputManager im;  // the choice survives a restart
        im.setDataDir(dir.string());
        CHECK(im.preferredGuid() == "radio-guid", "choice remembered: '%s'", im.preferredGuid().c_str());
        im.setPreferred("", "");
        CHECK(im.preferredGuid().empty(), "back to automatic");
    }
    fs::remove_all(dir);
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
