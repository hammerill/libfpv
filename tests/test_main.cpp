// libfpv - headless test suite. Runs in CI on every platform.
//
// Betaflight is a process-wide singleton, so all tests share one process and
// one fpv_global_init; each test resets the world and recreates the drone.
// Timing is FPV_TIMING_MANUAL, so every test is deterministic.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "libfpv.h"

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

namespace {

int g_failures = 0;
int g_checks = 0;
std::string g_current;

#define CHECK(cond, ...)                                                                      \
    do {                                                                                      \
        g_checks++;                                                                           \
        if (!(cond)) {                                                                        \
            g_failures++;                                                                     \
            std::printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond);                   \
            std::printf(__VA_ARGS__);                                                         \
            std::printf("\n");                                                                \
        }                                                                                     \
    } while (0)

void logCb(void *, fpv_log_level level, const char *msg) {
    if (level >= FPV_LOG_WARN || std::getenv("LIBFPV_TEST_VERBOSE")) std::printf("    [libfpv] %s\n", msg);
}

struct Rc {
    float ch[FPV_CHANNEL_COUNT];
    Rc() {
        for (int i = 0; i < FPV_CHANNEL_COUNT; i++) ch[i] = (i == 2 || i >= 4) ? -1.0f : 0.0f;
    }
    void send() { fpv_input_set_channels(ch, FPV_CHANNEL_COUNT); }
};

// Simple test world in a given convention. Ground at "up" = 0 and a wall
// `wallDist` metres ahead along +forward (host convention).
struct World {
    std::vector<float> v;
    std::vector<uint32_t> idx;
    void quad(const float a[3], const float b[3], const float c[3], const float d[3]) {
        uint32_t base = (uint32_t)(v.size() / 3);
        for (const float *p : {a, b, c, d}) v.insert(v.end(), p, p + 3);
        uint32_t q[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
        idx.insert(idx.end(), q, q + 6);
    }
    fpv_mesh mesh() const {
        fpv_mesh m{};
        m.positions = v.data();
        m.vertex_count = (uint32_t)(v.size() / 3);
        m.indices = idx.data();
        m.triangle_count = (uint32_t)(idx.size() / 3);
        return m;
    }
};

// Builds axis vectors of the host convention for a fpv_coord_system.
void axisVec(fpv_axis a, float s, float out[3]) {
    out[0] = out[1] = out[2] = 0;
    out[a / 2] = (a % 2) ? -s : s;
}

struct Host {
    fpv_coord_system cs;
    float R[3], U[3], F[3];  // unit host vectors for right/up/forward
    float upm;
    explicit Host(fpv_coord_system c) : cs(c), upm(c.units_per_meter) {
        axisVec(c.right, 1, R);
        axisVec(c.up, 1, U);
        axisVec(c.forward, 1, F);
    }
    // host position from (right, up, forward) metres
    void pos(float r, float u, float f, float out[3]) const {
        for (int i = 0; i < 3; i++) out[i] = (R[i] * r + U[i] * u + F[i] * f) * upm;
    }
    float along(const float v[3], const float axis[3]) const { return (v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2]) / upm; }
    float dir(const float v[3], const float axis[3]) const { return v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2]; }
    float right(const float v[3]) const { return along(v, R); }
    float up(const float v[3]) const { return along(v, U); }
    float fwd(const float v[3]) const { return along(v, F); }
};

World makeLevel(const Host &h, float size, float wallFwd) {
    World w;
    float a[3], b[3], c[3], d[3];
    h.pos(-size, 0, -size, a);
    h.pos(size, 0, -size, b);
    h.pos(size, 0, size, c);
    h.pos(-size, 0, size, d);
    w.quad(a, b, c, d);
    if (wallFwd > 0) {
        // a thin (zero thickness) wall across the forward axis
        h.pos(-50, -1, wallFwd, a);
        h.pos(50, -1, wallFwd, b);
        h.pos(50, 60, wallFwd, c);
        h.pos(-50, 60, wallFwd, d);
        w.quad(a, b, c, d);
    }
    return w;
}

struct Session {
    fpv_drone *drone = nullptr;
    fpv_frame f{};
    Rc rc;
    Session() { f.struct_size = sizeof(f); }
    void spawn(const float pos[3], const char *preset = nullptr) {
        fpv_drone_desc d;
        fpv_drone_desc_defaults(&d);
        std::memcpy(d.position, pos, sizeof(d.position));
        d.preset = preset ? preset : "freestyler-5";
        fpv_result r = fpv_drone_create(&d, &drone);
        CHECK(r == FPV_OK, "drone_create: %s", fpv_last_error());
    }
    void run(float seconds, const std::function<void(float t)> &each = nullptr) {
        int n = (int)std::lround(seconds / 0.01f);
        for (int i = 0; i < n; i++) {
            if (each) each(i * 0.01f);
            rc.send();
            fpv_step(0.01f);
            fpv_drone_tick(drone, 0.01f, &f);
        }
    }
    bool armed() const { return (f.flags & FPV_DRONE_ARMED) != 0; }
    void arm() {
        rc.ch[2] = -1;
        rc.ch[4] = -1;
        run(0.3f);
        rc.ch[4] = 1;
        run(0.3f);
    }
    void destroy() {
        if (drone) fpv_drone_delete(drone);
        drone = nullptr;
    }
};

std::string g_dataDir;

bool init(fpv_coord_system cs) {
    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    cfg.data_dir = g_dataDir.c_str();
    cfg.coords = cs;
    cfg.timing = FPV_TIMING_MANUAL;
    cfg.flags = FPV_INIT_NO_DEVICE_INPUT | FPV_INIT_NO_CONFIGURATOR;
    cfg.log = logCb;
    fpv_result r = fpv_global_init(&cfg);
    CHECK(r == FPV_OK, "init: %s", fpv_last_error());
    return r == FPV_OK;
}

void loadWorld(const World &w) {
    fpv_mesh m = w.mesh();
    fpv_result r = fpv_static_surfaces_load(&m, 1);
    CHECK(r == FPV_OK, "static load: %s", fpv_last_error());
}

// --- tests ------------------------------------------------------------------------

void testVersionAndErrors() {
    CHECK(fpv_abi_version() == LIBFPV_ABI_VERSION, "abi %u", fpv_abi_version());
    CHECK(std::strcmp(fpv_version_string(), LIBFPV_VERSION_STRING) == 0, "%s", fpv_version_string());
    fpv_drone *d = nullptr;
    CHECK(fpv_drone_create(nullptr, &d) == FPV_ERROR_NOT_INITIALIZED, "create before init must fail");
    CHECK(std::strlen(fpv_last_error()) > 0, "last error set");
    fpv_global_config bad;
    fpv_global_config_defaults(&bad);
    CHECK(fpv_global_init(&bad) == FPV_ERROR_INVALID_ARGUMENT, "missing data_dir must fail");
    bad.data_dir = g_dataDir.c_str();
    bad.coords.up = bad.coords.right;
    CHECK(fpv_global_init(&bad) == FPV_ERROR_INVALID_ARGUMENT, "degenerate axes must fail");
}

static void setEnv(const char *name, const char *value) {
#if defined(_WIN32)
    _putenv_s(name, value);  // "" removes it
#else
    if (*value) setenv(name, value, 1); else unsetenv(name);
#endif
}

void testSharedDataDir() {
    char buf[1024];
    setEnv("LIBFPV_DATA_DIR", "");
#if defined(__EMSCRIPTEN__)
    CHECK(fpv_shared_data_dir(buf, sizeof buf) == FPV_ERROR_NOT_FOUND, "no default folder in a browser");
#else
    CHECK(fpv_shared_data_dir(buf, sizeof buf) == FPV_OK, "default: %s", fpv_last_error());
    CHECK(std::strstr(buf, "libfpv") != nullptr, "default folder is named libfpv: %s", buf);
    CHECK(fpv_shared_data_dir(buf, 3) == FPV_ERROR_INVALID_ARGUMENT, "a too small buffer must fail");
#endif
    setEnv("LIBFPV_DATA_DIR", g_dataDir.c_str());
    CHECK(fpv_shared_data_dir(buf, 3) == FPV_ERROR_INVALID_ARGUMENT, "a too small buffer must fail");
    CHECK(fpv_shared_data_dir(buf, sizeof buf) == FPV_OK && g_dataDir == buf, "LIBFPV_DATA_DIR override: %s", buf);
    setEnv("LIBFPV_DATA_DIR", "");
}

void testArmDisarm() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 100, 0));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(0.5f);
    CHECK(!s.armed(), "starts disarmed");
    CHECK(s.f.arming_disable_flags == 0, "armable right after spawn, got \"%s\"", s.f.arming_disable_text);
    CHECK(s.f.flags & FPV_DRONE_ON_GROUND, "rests on the ground");
    CHECK(std::fabs(h.up(s.f.position)) < 0.08f, "rest height %.3f", h.up(s.f.position));
    s.arm();
    CHECK(s.armed(), "arms on AUX1 (%s)", s.f.arming_disable_text);
    CHECK(s.f.motors[0].rpm > 500, "motors idle when armed: %.0f rpm", s.f.motors[0].rpm);
    s.rc.ch[4] = -1;
    s.run(0.2f);
    CHECK(!s.armed(), "disarms");
    // Arm switch held on while not armable must not arm (Betaflight safety).
    s.rc.ch[2] = 0.5f;
    s.run(0.2f);
    s.rc.ch[4] = 1;
    s.run(0.2f);
    CHECK(!s.armed(), "does not arm with throttle up");
    CHECK(std::strstr(s.f.arming_disable_text, "THROTTLE") != nullptr, "reports THROTTLE: \"%s\"", s.f.arming_disable_text);
    s.destroy();
}

// Upside down on the ground: it will not arm normally; in turtle mode
// (AUX4) it arms, the props spin backwards and a roll stick flips it over.
// libfpv's crashflip_rate cuts the power once it has turned over, so the
// stick can simply be held. Every preset must make it.
void testTurtle() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 100, 0));
    // body up, as a host vector, from the frame's quaternion
    auto bodyUp = [&](const fpv_frame &f) {
        float x = f.rotation[0], y = f.rotation[1], z = f.rotation[2], w = f.rotation[3];
        float u[3] = {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)};
        return h.dir(u, h.U);
    };
    for (uint32_t i = 0; i < fpv_preset_count(); i++) {
        const char *name = fpv_preset_name(i);
        Session s;
        fpv_drone_desc d;
        fpv_drone_desc_defaults(&d);
        h.pos(0, 0.1f, 0, d.position);
        float flip[4] = {h.F[0], h.F[1], h.F[2], 0};  // 180 degrees about forward
        std::memcpy(d.rotation, flip, sizeof(flip));
        d.preset = name;
        CHECK(fpv_drone_create(&d, &s.drone) == FPV_OK, "drone_create: %s", fpv_last_error());
        s.run(1.0f);
        CHECK(bodyUp(s.f) < -0.9f, "%s rests upside down, up %.2f", name, bodyUp(s.f));
        s.arm();
        CHECK(!s.armed(), "%s does not arm upside down without turtle", name);
        CHECK(std::strstr(s.f.arming_disable_text, "ANGLE") != nullptr, "reports ANGLE: \"%s\"",
              s.f.arming_disable_text);
        s.rc.ch[4] = -1;
        s.rc.ch[7] = 1;  // turtle switch
        s.run(0.3f);
        s.arm();
        CHECK(s.armed(), "%s arms in turtle mode (%s)", name, s.f.arming_disable_text);
        CHECK(s.f.flags & FPV_DRONE_TURTLE, "turtle flag set while armed");
        s.rc.ch[0] = 0.6f;
        float upright = -1, damage = 0;
        s.run(1.5f, [&](float t) {
            if (upright < 0 && bodyUp(s.f) > 0.9f) upright = t;
        });
        s.rc.ch[0] = 0;
        s.run(0.5f);
        for (auto &m : s.f.motors) damage = std::max(damage, m.prop_damage);
        std::printf("    %s: upright after %.2f s, final up %.2f, prop damage %.2f\n", name, upright, bodyUp(s.f),
                    damage);
        CHECK(bodyUp(s.f) > 0.9f, "%s flipped back upright, up %.2f", name, bodyUp(s.f));
        CHECK(damage < 0.3f, "%s props survive the flip, damage %.2f", name, damage);
        // after a successful flip, turning turtle off re-arms in normal mode
        // at once (Betaflight's quick restart): it can fly away
        s.rc.ch[7] = -1;
        s.run(0.3f);
        CHECK(s.armed(), "%s armed again after turtle off (%s)", name, s.f.arming_disable_text);
        CHECK(!(s.f.flags & FPV_DRONE_TURTLE), "turtle flag cleared");
        s.rc.ch[2] = 0.3f;
        s.run(1.0f);
        CHECK(h.up(s.f.position) > 0.5f, "%s flies away, %.2f m", name, h.up(s.f.position));
        s.destroy();
    }
}

// An EEPROM from before turtle mode gets the AUX4 switch and the crash flip
// cut-off once, one from before the air mode and OSD switches gets AUX5 and
// AUX6 once; after that the player's own edits stay.
void testEepromUpgrade() {
    namespace fs = std::filesystem;
    auto cli = [](const char *cmd) {
        char *o = nullptr;
        fpv_cli_exec(cmd, &o);
        std::string r = o ? o : "";
        fpv_free(o);
        return r;
    };
    auto activate = [](const char *preset) {
        Session s;
        float p[3] = {0, 0, 0};
        s.spawn(p, preset);
        s.run(0.05f);
        s.destroy();
    };
    std::string first = fpv_preset_name(0), other = fpv_preset_name(1);
    activate(first.c_str());
    fs::path marker = fs::path(g_dataDir) / "eeprom";
    for (auto &e : fs::directory_iterator(marker))
        if (e.path().extension() == ".upgrade") fs::remove(e.path());
    // an old EEPROM, with another mode on AUX6
    cli("aux 3 0 0 900 900 0 0\naux 4 0 0 900 900 0 0\naux 5 0 0 900 900 0 0\naux 6 2 5 1700 2100 0 0\n"
        "set crashflip_rate = 0\nsave");
    activate(other.c_str());
    activate(first.c_str());
    std::string aux = cli("aux");
    CHECK(aux.find("aux 3 35 3 1700 2100") != std::string::npos, "turtle switch added: %s", aux.c_str());
    CHECK(aux.find("aux 4 28 4 1700 2100") != std::string::npos, "air mode switch added: %s", aux.c_str());
    CHECK(aux.find(" 19 5 ") == std::string::npos, "OSD switch not put on a used AUX6: %s", aux.c_str());
    CHECK(cli("get crashflip_rate").find("crashflip_rate = 30") != std::string::npos, "crash flip cut-off set");
    cli("aux 3 0 0 900 900 0 0\naux 4 0 0 900 900 0 0\nsave");  // the player removes them again
    activate(other.c_str());
    activate(first.c_str());
    aux = cli("aux");
    CHECK(aux.find("aux 3 35 3") == std::string::npos, "removed turtle switch stays removed");
    CHECK(aux.find(" 28 4 ") == std::string::npos, "removed air mode switch stays removed");
    // an EEPROM upgraded for turtle only gets just the new switches
    cli("aux 6 0 0 900 900 0 0\nsave");
    activate(other.c_str());
    for (auto &e : fs::directory_iterator(marker)) {
        if (e.path().extension() != ".upgrade") continue;
        std::ofstream(e.path(), std::ios::binary | std::ios::trunc) << "turtle-2";
    }
    activate(first.c_str());
    aux = cli("aux");
    CHECK(aux.find("aux 3 35 3") == std::string::npos, "turtle not re-added after turtle-2");
    CHECK(aux.find(" 28 4 1700 2100") != std::string::npos, "air mode switch added after turtle-2: %s", aux.c_str());
    CHECK(aux.find(" 19 5 1700 2100") != std::string::npos, "OSD switch added after turtle-2: %s", aux.c_str());
    // the simulated GPS is always on (D22)
    cli("feature -GPS\nsave");
    activate(other.c_str());
    activate(first.c_str());
    std::string features = cli("feature");
    features = features.substr(0, features.find('\n'));
    CHECK(features.find("GPS") != std::string::npos, "GPS feature restored: %s", features.c_str());
}

// The game's default diff goes on top of a new EEPROM, keeping the preset's
// own tune; existing EEPROMs are left alone.
void testDefaultDiff() {
    namespace fs = std::filesystem;
    auto cli = [](const char *cmd) {
        char *o = nullptr;
        fpv_cli_exec(cmd, &o);
        std::string r = o ? o : "";
        fpv_free(o);
        return r;
    };
    auto activate = [](const char *preset) {
        Session s;
        float p[3] = {0, 0, 0};
        s.spawn(p, preset);
        s.run(0.05f);
        s.destroy();
    };
    const char *tuned = "betafpv-air65-ii", *other = "freestyler-5";
    activate(other);
    fs::path eeprom = fs::path(g_dataDir) / "eeprom";
    fs::remove(eeprom / (std::string(tuned) + ".eeprom"));
    fs::remove(eeprom / (std::string(tuned) + ".eeprom.upgrade"));
    fpv_set_default_betaflight_diff("# diff all\ndefaults nosave\nset craft_name = GAMECFG\nrateprofile 0\n"
                                    "set roll_srate = 53\nsave\n");
    activate(tuned);
    CHECK(cli("get craft_name").find("= GAMECFG") != std::string::npos, "default diff applied to a new EEPROM");
    CHECK(cli("get roll_srate").find("roll_srate = 53") != std::string::npos, "default diff rates applied");
    CHECK(cli("get gyro_lpf2_static_hz").find("= 600") != std::string::npos, "preset tune kept under the default diff");
    CHECK(cli("aux").find("aux 0 0 0 1700 2100") != std::string::npos, "ARM still on AUX1");
    activate(other);
    CHECK(cli("get craft_name").find("GAMECFG") == std::string::npos, "existing EEPROM left alone");
    fpv_set_default_betaflight_diff(nullptr);
    // later tests fly this quad on libfpv's own configuration
    fs::remove(eeprom / (std::string(tuned) + ".eeprom"));
    fs::remove(eeprom / (std::string(tuned) + ".eeprom.upgrade"));
}

void testSecondDroneRejected() {
    Session a;
    float p[3] = {0, 0, 0};
    a.spawn(p);
    fpv_drone *second = nullptr;
    fpv_result r = fpv_drone_create(nullptr, &second);
    CHECK(r == FPV_ERROR_DRONE_EXISTS, "second drone: %s", fpv_result_string(r));
    CHECK(second == nullptr, "no handle for the second drone");
    a.destroy();
    fpv_frame f{};
    f.struct_size = sizeof(f);
    CHECK(fpv_drone_tick(a.drone, 0.01f, &f) == FPV_ERROR_INVALID_ARGUMENT, "deleted handle rejected");
    Session b;
    b.spawn(p);
    CHECK(b.drone != nullptr, "a drone can be created again after delete");
    b.destroy();
}

// Finds the throttle that hovers by bisection in a clean world, then checks
// that this throttle holds altitude.
// Arms in the air at a given throttle, lets the motors settle, then
// teleports back to the start with zero velocity (staying armed). Returns
// the session ready to measure from rest.
void hoverFromRest(Session &s, const Host &h, float thr, const char *preset = nullptr) {
    float p[3];
    h.pos(0, 50, 0, p);  // high up: no ground effect
    s.spawn(p, preset);
    s.rc.ch[5] = 1;  // ANGLE mode (AUX2) keeps it level
    s.arm();
    s.rc.ch[2] = thr;
    s.run(1.0f);
    float rot[4] = {0, 0, 0, 1};
    fpv_drone_reset(s.drone, p, rot);
    s.run(0.01f);
}

// Hover throttle by bisection on the direction of motion from rest.
float findHoverThrottle(const Host &h, const char *preset = nullptr) {
    float lo = -1.0f, hi = 0.2f;
    for (int iter = 0; iter < 14; iter++) {
        float mid = 0.5f * (lo + hi);
        Session s;
        hoverFromRest(s, h, mid, preset);
        float y0 = h.up(s.f.position);
        s.run(1.0f);
        bool rising = h.up(s.f.position) > y0;
        s.destroy();
        (rising ? hi : lo) = mid;
    }
    return 0.5f * (lo + hi);
}

void testHover() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 200, 0));
    float thr = findHoverThrottle(h);
    std::printf("    hover throttle %.4f (stick), %.1f%%\n", thr, (thr + 1) * 50);
    CHECK(thr > -0.9f && thr < 0.2f, "plausible hover throttle %.3f", thr);
    Session s;
    hoverFromRest(s, h, thr);
    CHECK(s.armed(), "armed after reset");
    float y0 = h.up(s.f.position), maxDev = 0;
    s.run(3.0f, [&](float) { maxDev = std::max(maxDev, std::fabs(h.up(s.f.position) - y0)); });
    std::printf("    hover altitude deviation over 3 s: %.2f m\n", maxDev);
    CHECK(maxDev < 1.0f, "hover holds altitude within 1 m, deviation %.2f m", maxDev);
    CHECK(s.armed(), "still armed");
    s.destroy();
}

// Keyboard hover throttle (opt-in setting): no key hovers at a throttle the
// input learns in flight, W climbs, S cuts. Flown with keys only, on quads
// whose hover throttle differs a lot.
void testKeyboardHover() {
    auto setHover = [](bool on) {
        fpv_global_terminate();
        std::ofstream(std::filesystem::path(g_dataDir) / "settings.json") << "{\"keyboardHover\": " << (on ? "true" : "false") << "}";
        init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    };
    setHover(true);
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 200, 0));
    fpv_input_clear_channels();
    for (const char *name : {"freestyler-5", "betafpv-air65-ii", "270Supra"}) {
        Session s;
        float p[3];
        h.pos(0, 0, 0, p);
        s.spawn(p, name);
        auto run = [&](float seconds, const std::function<void()> &each = nullptr) {
            for (int i = 0, n = (int)std::lround(seconds / 0.01f); i < n; i++) {
                fpv_step(0.01f);
                fpv_drone_tick(s.drone, 0.01f, &s.f);
                if (each) each();
            }
        };
        auto press = [](int key, bool down) { fpv_input_key(key, down ? 1 : 0); };
        run(0.5f);
        press(FPV_KEY_ENTER, true);
        run(0.1f);
        press(FPV_KEY_ENTER, false);
        run(1.0f);
        CHECK(s.armed(), "%s: arms from the keyboard (%s)", name, s.f.arming_disable_text);
        CHECK(s.f.flags & FPV_DRONE_ON_GROUND, "%s: no key on the ground does not lift off", name);
        press('W', true);
        run(0.7f);
        press('W', false);
        float top = h.up(s.f.position);
        CHECK(top > 1.0f, "%s: W climbs, %.2f m after 0.7 s", name, top);
        run(5.0f);  // settle and learn
        float y0 = h.up(s.f.position), maxDev = 0;
        run(3.0f, [&] { maxDev = std::max(maxDev, std::fabs(h.up(s.f.position) - y0)); });
        std::printf("    %s: climbed to %.1f m, hovering at %.1f m, deviation over 3 s %.2f m\n", name, top, y0, maxDev);
        CHECK(y0 > 1.0f, "%s: still in the air after releasing W, %.2f m", name, y0);
        CHECK(maxDev < 0.3f, "%s: no key holds altitude within 0.3 m, deviation %.2f m", name, maxDev);
        press('W', true);
        press('S', true);
        y0 = h.up(s.f.position);
        run(1.0f);
        CHECK(std::fabs(h.up(s.f.position) - y0) < 0.3f, "%s: W and S together hover, moved %.2f m", name, h.up(s.f.position) - y0);
        press('W', false);
        run(0.4f);
        press('S', false);
        CHECK(h.up(s.f.position) < y0 - 0.3f, "%s: S cuts the throttle, %.2f m below", name, y0 - h.up(s.f.position));
        // A/D yaw starts slow and grows: the yaw rate after 0.1 s is well
        // below the rate after 0.8 s
        float yaw01 = 0, yaw08 = 0;
        int steps = 0;
        press('D', true);
        run(0.8f, [&] {
            float r = std::fabs(s.f.angular_velocity[1]);
            if (++steps == 10) yaw01 = r;
            yaw08 = r;
        });
        press('D', false);
        std::printf("    %s: yaw rate %.0f deg/s after 0.1 s, %.0f deg/s after 0.8 s\n", name, yaw01 * 57.3f, yaw08 * 57.3f);
        CHECK(yaw08 > 3 * yaw01 && yaw08 > 3.0f, "%s: D yaw grows while held (%.2f -> %.2f rad/s)", name, yaw01, yaw08);
        press(FPV_KEY_ENTER, true);
        run(0.1f);
        press(FPV_KEY_ENTER, false);
        run(0.1f);
        s.destroy();
    }
    setHover(false);
}

void testPunchOut() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 0));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.arm();
    s.rc.ch[2] = 1.0f;
    float maxV = 0;
    s.run(2.0f, [&](float) { maxV = std::max(maxV, s.f.speed_kmh); });
    float height = h.up(s.f.position);
    std::printf("    punch-out: %.0f km/h, %.1f m after 2 s\n", maxV, height);
    CHECK(maxV > 60.0f && maxV < 320.0f, "plausible punch-out speed %.0f km/h", maxV);
    CHECK(height > 20.0f, "climbed %.1f m", height);
    s.destroy();
}

// Flight benchmarks against what real quads of each class do. Printed for
// tuning presets; the checks are wide plausibility bands (docs/DECISIONS.md
// D25 lists the reference figures).
struct RealismBand {
    const char *preset;
    float hoverMin, hoverMax;  // throttle stick %
    float chopDropMin;         // m fallen 1 s after cutting throttle from hover (free fall 4.9 m)
    float punchMin, punchMax;  // km/h climbing 1 s into a full-throttle punch from hover
    float punch025Min;         // km/h climbing 0.25 s into it: how quickly the punch builds
    float topMin, topMax;      // km/h level, full throttle, 80 deg tilt
    float coastMax;            // s to slow from 40 to 20 km/h after levelling out at hover throttle
};

void testFlightRealism() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 0));
    const RealismBand bands[] = {
        {"freestyler-5", 15, 30, 3.5f, 80, 170, 55, 110, 180, 2.5f},
        {"betafpv-air65-ii", 20, 45, 3.0f, 40, 110, 31, 45, 85, 1.5f},
    };
    for (const RealismBand &b : bands) {
        float thr = findHoverThrottle(h, b.preset);
        float hoverPct = (thr + 1) * 50;

        Session s;
        hoverFromRest(s, h, thr, b.preset);
        float y0 = h.up(s.f.position);
        s.rc.ch[2] = -1.0f;
        s.run(1.0f);
        float drop = y0 - h.up(s.f.position);
        s.destroy();

        hoverFromRest(s, h, thr, b.preset);
        s.run(1.0f);  // settle from the teleport: the punch starts from a steady hover
        y0 = h.up(s.f.position);
        s.rc.ch[2] = 1.0f;
        s.run(0.25f);
        float punch025 = h.up(s.f.velocity) * 3.6f;
        s.run(0.75f);
        float punch1 = h.up(s.f.velocity) * 3.6f;
        float climbed = h.up(s.f.position) - y0;
        s.destroy();

        // top speed: ANGLE mode with an 80 degree limit, full forward, full throttle
        char *o = nullptr;
        fpv_cli_exec("set angle_limit = 80\nsave", &o);
        fpv_free(o);
        hoverFromRest(s, h, thr, b.preset);
        s.rc.ch[1] = 1.0f;
        s.rc.ch[2] = 1.0f;
        float top = 0;
        s.run(8.0f, [&](float) {
            float hs = std::sqrt(h.fwd(s.f.velocity) * h.fwd(s.f.velocity) + h.right(s.f.velocity) * h.right(s.f.velocity));
            top = std::max(top, hs * 3.6f);
        });
        // coast: level out at hover throttle from top speed, time to lose half the speed
        s.rc.ch[1] = 0.0f;
        s.rc.ch[2] = thr;
        // and, the "floaty" part, from 40 to 20 km/h
        float v0 = std::fabs(h.fwd(s.f.velocity)), half = -1, at40 = -1, at20 = -1;
        s.run(10.0f, [&](float t) {
            float v = std::fabs(h.fwd(s.f.velocity));
            if (half < 0 && v < 0.5f * v0) half = t;
            if (at40 < 0 && v * 3.6f < 40) at40 = t;
            if (at20 < 0 && v * 3.6f < 20) at20 = t;
        });
        float coast = (at40 >= 0 && at20 >= 0) ? at20 - at40 : -1;
        s.destroy();
        fpv_cli_exec("set angle_limit = 60\nsave", &o);
        fpv_free(o);

        std::printf("    %s: hover %.0f%% stick | chop: %.2f m in 1 s | punch: %.0f km/h at 0.25 s, %.0f km/h and "
                    "%.0f m at 1 s | top %.0f km/h | level-out from %.0f km/h halves in %.2f s, 40 -> 20 km/h in %.2f s\n",
                    b.preset, hoverPct, drop, punch025, punch1, climbed, top, v0 * 3.6f, half, coast);
        CHECK(hoverPct > b.hoverMin && hoverPct < b.hoverMax, "%s hover %.0f%%", b.preset, hoverPct);
        CHECK(drop > b.chopDropMin, "%s chop drop %.2f m", b.preset, drop);
        CHECK(punch1 > b.punchMin && punch1 < b.punchMax, "%s punch %.0f km/h", b.preset, punch1);
        CHECK(punch025 > b.punch025Min, "%s punch builds to %.0f km/h in 0.25 s", b.preset, punch025);
        CHECK(top > b.topMin && top < b.topMax, "%s top speed %.0f km/h", b.preset, top);
        CHECK(half > 0, "%s slows down after levelling out", b.preset);
        CHECK(coast > 0 && coast < b.coastMax, "%s coasts from 40 to 20 km/h in %.2f s", b.preset, coast);
    }
}


// Battery draw and flight time against real quads of each class. A hard
// freestyle pack is flown as a repeating 4 s cycle: 1 s full throttle, 1 s at
// 60 %, 1 s at zero throttle, 1 s hovering, teleported back each cycle. The
// pack is done when the cell voltage at the end of the hover second (what the
// pilot reads on the OSD, averaged over its last half) drops below 3.5 V.
struct BatteryBand {
    const char *preset;
    float hoverAmpMin, hoverAmpMax;  // A at hover, fresh pack
    float fullAmpMin, fullAmpMax;    // A at full throttle, fresh pack
    float packMin, packMax;          // s of hard flying
};

void testBatteryEndurance() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 0));
    const BatteryBand bands[] = {
        {"freestyler-5", 3, 8, 60, 130, 100, 240},
        {"betafpv-air65-ii", 2.5f, 4, 12, 22, 110, 165},
    };
    for (const BatteryBand &b : bands) {
        float thr = findHoverThrottle(h, b.preset);
        Session s;
        hoverFromRest(s, h, thr, b.preset);
        float start[3];
        std::memcpy(start, s.f.position, sizeof start);
        s.run(1.0f);
        float hoverAmp = s.f.battery.current_amp, fresh = s.f.battery.cell_voltage;
        s.rc.ch[2] = 1.0f;
        s.run(1.0f);
        float fullAmp = s.f.battery.current_amp;
        float pack = -1, mahAtLand = 0, cellAt150 = 0;
        for (float t = 0; t < 600 && (pack < 0 || t < 150); t += 4) {
            float rot[4] = {0, 0, 0, 1};
            fpv_drone_reset(s.drone, start, rot);
            s.rc.ch[2] = 1.0f;
            s.run(1.0f);
            s.rc.ch[2] = 0.2f;
            s.run(1.0f);
            s.rc.ch[2] = -1.0f;
            s.run(1.0f);
            s.rc.ch[2] = thr;
            s.run(0.5f);
            float cell = 0;
            s.run(0.5f, [&](float) { cell += s.f.battery.cell_voltage / 50; });
            if (t + 4 <= 150) cellAt150 = cell;
            if (pack < 0 && cell < 3.5f) {
                pack = t + 4;
                mahAtLand = s.f.battery.mah_drawn;
            }
        }
        CHECK(s.armed(), "%s still armed after the pack", b.preset);
        s.destroy();
        std::printf("    %s: fresh %.2f V/cell at hover | hover %.1f A, full throttle %.1f A | hard flying: %.2f V/cell "
                    "at hover after 150 s, lands (3.5 V) after %.0f s with %.0f mAh used\n",
                    b.preset, fresh, hoverAmp, fullAmp, cellAt150, pack, mahAtLand);
        CHECK(hoverAmp > b.hoverAmpMin && hoverAmp < b.hoverAmpMax, "%s hover draw %.1f A", b.preset, hoverAmp);
        CHECK(fullAmp > b.fullAmpMin && fullAmp < b.fullAmpMax, "%s full-throttle draw %.1f A", b.preset, fullAmp);
        CHECK(pack > b.packMin && pack < b.packMax, "%s pack lasts %.0f s of hard flying", b.preset, pack);
    }
}

// Zero throttle with air mode: the I-term holds the attitude while the quad
// falls, so a tilted drop from rest drifts only a few degrees. The Freestyler
// runs without air mode first: switching quads must reboot Betaflight so the
// Air65 II gets its own features (air mode on), not the previous quad's.
void testIdleAttitude() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 0));
    float p[3];
    h.pos(0, 250, 0, p);
    Session s;
    s.spawn(p, "freestyler-5");
    char *o = nullptr;
    fpv_import_betaflight_diff("feature -AIRMODE\n", &o);  // reboots: air mode off now
    fpv_free(o);
    s.destroy();
    for (const char *name : {"betafpv-air65-ii", "freestyler-5"}) {
        if (std::string(name) == "freestyler-5") {
            s.spawn(p, name);
            fpv_import_betaflight_diff("feature AIRMODE\n", &o);
            fpv_free(o);
            s.destroy();
        }
        s.spawn(p, name);
        s.arm();
        s.rc.ch[2] = 0.0f;  // raise the throttle once: air mode activates
        s.run(0.3f);
        std::printf("    %s drift 3 s after a zero-throttle drop:", name);
        for (float deg : {45.0f, 90.0f, 135.0f}) {
            float half = deg / 57.3f / 2;
            float q[4] = {std::sin(half), 0, 0, std::cos(half)};
            fpv_drone_reset(s.drone, p, q);
            s.rc.ch[2] = -1.0f;
            s.run(3.0f);
            const float *r = s.f.rotation;
            float upY = 1 - 2 * (r[0] * r[0] + r[2] * r[2]);
            float drift = std::fabs(std::acos(std::max(-1.0f, std::min(1.0f, upY))) * 57.3f - deg);
            std::printf(" %.0f deg -> %.1f", deg, drift);
            CHECK(drift < 10.0f, "%s holds %.0f deg at zero throttle, drifted %.1f deg", name, deg, drift);
        }
        std::printf("\n");
        CHECK(s.armed(), "%s still armed", name);
        s.destroy();
    }
}

// Stick directions in every host convention: pitch forward flies forward,
// roll right flies right, yaw right turns clockwise seen from above.
void testAxes(fpv_coords_preset_id preset, float upm, const char *name) {
    Host h(fpv_coords_preset(preset, upm));
    fpv_global_terminate();
    init(h.cs);
    loadWorld(makeLevel(h, 300, 0));

    auto fly = [&](int channel, float value, float seconds) {
        Session s;
        float p[3];
        h.pos(0, 30, 0, p);
        s.spawn(p);
        s.rc.ch[5] = 1;  // ANGLE
        s.arm();
        s.rc.ch[2] = -0.2f;
        s.rc.ch[channel] = value;
        s.run(seconds);
        fpv_frame f = s.f;
        s.destroy();
        return f;
    };
    fpv_frame f = fly(1, 0.5f, 1.5f);  // pitch forward
    CHECK(h.fwd(f.velocity) > 3.0f, "%s: pitch forward -> forward velocity %.2f m/s", name, h.fwd(f.velocity));
    CHECK(h.dir(f.camera.forward, h.F) > 0.3f, "%s: FPV camera looks forward", name);
    f = fly(0, 0.5f, 1.5f);  // roll right
    CHECK(h.right(f.velocity) > 3.0f, "%s: roll right -> rightward velocity %.2f m/s", name, h.right(f.velocity));
    f = fly(3, 0.5f, 0.6f);  // yaw right
    // drone forward axis expressed via the FPV camera forward (uptilt only pitches)
    float fr = h.dir(f.camera.forward, h.R), ff = h.dir(f.camera.forward, h.F);
    CHECK(fr > 0.2f, "%s: yaw right turns the nose right (r=%.2f f=%.2f)", name, fr, ff);
    // the camera basis must be right-handed in the host's own sense
    float u[3], r[3], fw[3];
    std::memcpy(u, f.camera.up, sizeof u);
    std::memcpy(r, f.camera.right, sizeof r);
    std::memcpy(fw, f.camera.forward, sizeof fw);
    CHECK(std::fabs(r[0] * u[0] + r[1] * u[1] + r[2] * u[2]) < 1e-3f, "%s: camera axes orthogonal", name);
}

void testWallCrash() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 40));
    Session s;
    float p[3];
    h.pos(0, 3, 0, p);
    s.spawn(p);
    s.arm();
    s.rc.ch[2] = -0.3f;
    s.run(0.3f);
    // Teleport the drone just in front of the wall at 30 m/s using the
    // physics: point it at the wall and let it fly level at speed.
    s.rc.ch[1] = 0.0f;
    // Set velocity via reset + fast forward flight: pitch hard forward with high throttle.
    s.rc.ch[5] = 1;      // ANGLE so it stays level
    s.rc.ch[1] = 1.0f;   // full forward
    s.rc.ch[2] = 0.6f;
    int impacts = 0;
    float maxSpeed = 0, maxFwd = -1e9f, impactSpeed = 0;
    s.run(6.0f, [&](float) {
        for (uint32_t i = 0; i < s.f.impact_count; i++) {
            impacts++;
            impactSpeed = std::max(impactSpeed, s.f.impacts[i].speed);
        }
        maxSpeed = std::max(maxSpeed, s.f.speed_kmh / 3.6f);
        // a drone that survives the hit may fly around the wall (100 x 60 m)
        if (std::fabs(h.right(s.f.position)) < 49 && h.up(s.f.position) < 59)
            maxFwd = std::max(maxFwd, h.fwd(s.f.position));
    });
    std::printf("    wall: max speed %.1f m/s, impact speed %.1f m/s, closest %.3f m before the wall\n", maxSpeed,
                impactSpeed, 40.0f - maxFwd);
    CHECK(impacts > 0, "impact reported");
    CHECK(maxFwd < 40.0f, "never passed through the wall (max %.3f)", maxFwd);
    s.destroy();

    // Direct 30 m/s shot: spawn in front of the wall and fire the drone at it
    // with an explicit velocity through a moving platform-free setup.
    World w = makeLevel(h, 300, 40);
    loadWorld(w);
    Session t;
    h.pos(0, 5, 39.0f, p);
    t.spawn(p);
    // Use reset to align, then let the sim run with injected forward speed by
    // repeatedly flying full forward; check substepping at high speed.
    t.rc.ch[5] = 1;
    t.arm();
    t.rc.ch[1] = 1.0f;
    t.rc.ch[2] = 1.0f;
    float maxF = -1e9f;
    t.run(2.0f, [&](float) { maxF = std::max(maxF, h.fwd(t.f.position)); });
    CHECK(maxF < 40.0f, "thin wall holds at full power from 1 m (max %.3f)", maxF);
    t.destroy();
}

// 30 m/s through a thin wall using a moving surface object: the wall flies
// at the resting drone at 30 m/s (relative motion is what matters).
void testThirtyMetersPerSecond() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    loadWorld(makeLevel(h, 300, 0));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(0.3f);
    // thin wall, 4 m wide, 3 m tall, zero thickness, in object space at fwd=0
    World wall;
    float a[3], b[3], c[3], d[3];
    h.pos(-2, -0.5f, 0, a);
    h.pos(2, -0.5f, 0, b);
    h.pos(2, 3, 0, c);
    h.pos(-2, 3, 0, d);
    wall.quad(a, b, c, d);
    fpv_mesh m = wall.mesh();
    fpv_transform t{};
    t.rotation[3] = 1;
    h.pos(0, 0, 10, t.position);
    uint32_t id = 0;
    CHECK(fpv_surface_object_create(&m, &t, &id) == FPV_OK, "object create: %s", fpv_last_error());
    // move the wall towards the drone at 30 m/s with 100 Hz updates
    int impacts = 0;
    float minRel = 1e9f;
    for (int i = 0; i < 100; i++) {
        float fwd = 10.0f - 30.0f * 0.01f * (i + 1);
        h.pos(0, 0, fwd, t.position);
        fpv_surface_object_move(id, &t);
        s.rc.send();
        fpv_step(0.01f);
        fpv_drone_tick(s.drone, 0.01f, &s.f);
        impacts += (int)s.f.impact_count;
        minRel = std::min(minRel, h.fwd(s.f.position) - fwd);  // drone must stay in front (< 0 = behind wall)
    }
    std::printf("    30 m/s wall: %d impacts, drone ended %.2f m along forward\n", impacts, h.fwd(s.f.position));
    CHECK(impacts > 0, "impact at 30 m/s reported");
    CHECK(h.fwd(s.f.position) < -5.0f, "the wall carried the drone (pos %.2f)", h.fwd(s.f.position));
    fpv_surface_object_delete(id);
    s.destroy();
}

void testMotorBurnout() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    // A ceiling made of a prop-killing material: the drone takes off, loses
    // its propellers, drops, and the "pilot" keeps the throttle pinned. With
    // broken props (extra friction current, no prop-wash cooling) the
    // windings overheat until a motor burns out.
    fpv_material kill{};
    kill.restitution = 0.1f;
    kill.friction = 0.5f;
    kill.flags = FPV_MATERIAL_KILLS_PROPS;
    CHECK(fpv_material_set(7, &kill) == FPV_OK, "material");
    World w = makeLevel(h, 100, 0);
    float a[3], b[3], c[3], d[3];
    h.pos(-5, 0.6f, -5, a);
    h.pos(5, 0.6f, -5, b);
    h.pos(5, 0.6f, 5, c);
    h.pos(-5, 0.6f, 5, d);
    w.quad(a, b, c, d);
    std::vector<uint8_t> mats(w.idx.size() / 3, 0);
    mats[2] = mats[3] = 7;  // the ceiling quad
    fpv_mesh m = w.mesh();
    m.materials = mats.data();
    CHECK(fpv_static_surfaces_load(&m, 1) == FPV_OK, "load");
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p, "propeater-5");
    s.arm();
    s.rc.ch[2] = 1.0f;
    float maxT = 0;
    bool burned = false, crashed = false, propsBroken = false;
    s.run(240.0f, [&](float) {
        for (auto &mo : s.f.motors) {
            maxT = std::max(maxT, mo.temperature_c);
            if (mo.flags & FPV_MOTOR_BURNED_OUT) burned = true;
            if (mo.flags & FPV_MOTOR_PROP_BROKEN) propsBroken = true;
        }
        if (s.f.flags & FPV_DRONE_CRASHED) crashed = true;
    });
    std::printf("    burnout: props broken=%d, max motor temperature %.0f C, burned=%d\n", propsBroken, maxT, burned);
    CHECK(propsBroken, "prop-killing material breaks the propellers");
    CHECK(crashed, "broken props mark the drone crashed");
    CHECK(burned, "a motor burns out at full throttle on broken props (max %.0f C)", maxT);
    s.rc.ch[2] = -1.0f;
    s.run(0.1f);
    // like a player respawning: repair, then put it back on the ground
    // (it may still be touching the prop-killing ceiling)
    fpv_drone_repair(s.drone);
    fpv_drone_reset(s.drone, p, nullptr);
    s.run(0.1f);
    bool still = false;
    for (auto &mo : s.f.motors)
        if (mo.flags & (FPV_MOTOR_BURNED_OUT | FPV_MOTOR_PROP_BROKEN)) still = true;
    CHECK(!still, "repair restores motors and propellers (flags %u %u %u %u, %.0f C, damage %.2f)", s.f.motors[0].flags,
          s.f.motors[1].flags, s.f.motors[2].flags, s.f.motors[3].flags, s.f.motors[0].temperature_c,
          s.f.motors[0].prop_damage);
    s.destroy();
    fpv_material def{0.2f, 0.6f, 0};
    fpv_material_set(7, &def);
}

void testPresets() {
    uint32_t n = fpv_preset_count();
    CHECK(n >= 5, "built-in presets found: %u", n);
    for (uint32_t i = 0; i < n; i++) {
        const char *name = fpv_preset_name(i);
        CHECK(name && name[0], "preset %u has a name", i);
        Session s;
        float p[3] = {0, 0, 0};
        s.spawn(p, name);
        s.run(0.3f);
        s.arm();
        s.rc.ch[2] = 0.0f;
        s.run(1.0f);
        CHECK(s.armed(), "preset \"%s\" arms (%s)", name, s.f.arming_disable_text);
        CHECK(s.f.position[1] > 1.0f, "preset \"%s\" flies (%.2f m)", name, s.f.position[1]);
        CHECK(s.f.battery.cell_count >= 1, "preset \"%s\" battery", name);
        s.destroy();
    }
    Session s;
    float p[3] = {0, 0, 0};
    fpv_drone_desc d;
    fpv_drone_desc_defaults(&d);
    d.preset = "no such quad";
    CHECK(fpv_drone_create(&d, &s.drone) == FPV_ERROR_NOT_FOUND, "unknown preset rejected");
    (void)p;
}

// Betaflight sets its low-voltage thresholds when it first sees a battery;
// a full 4S pack after a 6S quad must not read as critical.
// Switching quads on the ground: a bigger quad must not start inside the
// ground (the penetration push slid it away, then it fell through).
void testPresetSwitchRest() {
    float p[3] = {0, 0, 0};
    for (const char *to : {"freestyler-5", "propeater-5"}) {
        Session s;
        s.spawn(p, "betafpv-air65-ii");
        s.run(1.0f);
        for (const char *name : {to, "betafpv-air65-ii"}) {
            float before[3];
            std::memcpy(before, s.f.position, sizeof before);
            CHECK(fpv_drone_set_preset(s.drone, name) == FPV_OK, "set preset %s: %s", name, fpv_last_error());
            s.run(2.0f);
            float dx = s.f.position[0] - before[0], dz = s.f.position[2] - before[2];
            float slide = std::sqrt(dx * dx + dz * dz);
            CHECK(slide < 0.05f, "%s stays put after the switch, slid %.2f m", name, slide);
            CHECK(s.f.position[1] > -0.01f, "%s stays on the ground, y %.3f", name, s.f.position[1]);
        }
        s.destroy();
    }
}

void testPresetSwitchBattery() {
    auto batteryStatus = []() {
        char *out = nullptr;
        fpv_cli_exec("status", &out);
        std::string s = out ? out : "";
        fpv_free(out);
        size_t at = s.find("Voltage:");
        return at == std::string::npos ? s : s.substr(at, s.find('\n', at) - at);
    };
    float p[3] = {0, 0, 0};
    const char *order[] = {"freestyler-5", "vtx-slayer-one-4", "freestyler-5"};
    for (const char *name : order) {
        Session s;
        s.spawn(p, name);
        s.run(1.0f);
        std::string st = batteryStatus();
        char cells[16];
        std::snprintf(cells, sizeof cells, "(%uS battery - OK)", s.f.battery.cell_count);
        CHECK(st.find(cells) != std::string::npos, "%s: full battery is OK: %s", name, st.c_str());
        s.destroy();
    }
    // switched in place, also while armed (Betaflight's own detection only
    // runs disarmed, and could take the previous pack's voltage)
    for (bool armed : {false, true}) {
        Session s;
        s.spawn(p, "freestyler-5");
        s.run(1.0f);
        for (const char *name : {"betafpv-air65-ii", "freestyler-5", "betafpv-air65-ii"}) {
            if (armed) s.arm();
            CHECK(fpv_drone_set_preset(s.drone, name) == FPV_OK, "set preset %s: %s", name, fpv_last_error());
            s.run(1.0f);
            std::string st = batteryStatus();
            char cells[16];
            std::snprintf(cells, sizeof cells, "(%uS battery - OK)", s.f.battery.cell_count);
            CHECK(st.find(cells) != std::string::npos, "%s switched in place%s: full battery is OK: %s", name,
                  armed ? " armed" : "", st.c_str());
        }
        s.destroy();
    }
}

void testCli() {
    char *out = nullptr;
    CHECK(fpv_cli_exec("get motor_pwm_protocol", &out) == FPV_OK, "cli: %s", fpv_last_error());
    CHECK(out && std::strstr(out, "PWM"), "PWM protocol: %s", out ? out : "(null)");
    fpv_free(out);
    CHECK(fpv_cli_exec("bl", &out) == FPV_OK, "refused commands do not fail the call");
    CHECK(out && std::strstr(out, "refused"), "bootloader refused: %s", out ? out : "");
    fpv_free(out);
    CHECK(fpv_cli_exec("set roll_rc_rate = 123\nsave", &out) == FPV_OK, "set+save");
    fpv_free(out);
    CHECK(fpv_cli_exec("get roll_rc_rate", &out) == FPV_OK, "get");
    CHECK(out && std::strstr(out, "123"), "value stored: %s", out ? out : "");
    fpv_free(out);    CHECK(fpv_cli_exec("aux", &out) == FPV_OK, "aux");
    CHECK(out && std::strstr(out, "aux 3 35 3 1700 2100"), "turtle on AUX4 by default: %s", out ? out : "");
    CHECK(out && std::strstr(out, "aux 4 28 4 1700 2100"), "air mode on AUX5 by default: %s", out ? out : "");
    CHECK(out && std::strstr(out, "aux 5 19 5 1700 2100"), "OSD disable on AUX6 by default: %s", out ? out : "");
    fpv_free(out);
    CHECK(fpv_cli_exec("get crashflip_rate", &out) == FPV_OK, "get crashflip_rate");
    CHECK(out && std::strstr(out, "crashflip_rate = 30"), "turtle cut-off on by default: %s", out ? out : "");
    fpv_free(out);
}

// Corpus of real-world style `diff all` outputs in tests/diffs/.
void testDiffImport(const std::string &corpusDir) {
    namespace fs = std::filesystem;
    loadWorld(makeLevel(Host(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), 100, 0));
    int files = 0;
    std::error_code ec;
    for (auto &e : fs::directory_iterator(corpusDir, ec)) {
        if (e.path().extension() != ".txt") continue;
        files++;
        std::string name = e.path().filename().string();
        FILE *fp = std::fopen(e.path().string().c_str(), "rb");
        std::string text;
        char buf[4096];
        size_t n;
        while ((n = std::fread(buf, 1, sizeof buf, fp)) > 0) text.append(buf, n);
        std::fclose(fp);
        char *report = nullptr;
        fpv_result r = fpv_import_betaflight_diff(text.c_str(), &report);
        CHECK(r == FPV_OK, "%s: import %s", name.c_str(), fpv_last_error());
        CHECK(report && std::strstr(report, "Applied"), "%s: report", name.c_str());
        if (std::getenv("LIBFPV_TEST_VERBOSE") && report) std::printf("%s\n", report);
        // Renamed settings from older firmware are reported, not fatal.
        const char *rej = report ? std::strstr(report, "Rejected by Betaflight") : nullptr;
        if (rej) std::printf("    %s: %.*s\n", name.c_str(), (int)(std::strchr(rej, '\n') ? std::strchr(rej, '\n') - rej : 40), rej);
        fpv_free(report);

        char *out = nullptr;
        fpv_cli_exec("get motor_pwm_protocol\naux", &out);
        CHECK(out && std::strstr(out, "= PWM"), "%s: PWM forced", name.c_str());
        CHECK(out && std::strstr(out, "aux 0 0 0 1700 2100"), "%s: ARM on AUX1", name.c_str());
        fpv_free(out);

        // and it flies
        Session s;
        float p[3] = {0, 0, 0};
        s.spawn(p);
        s.run(0.3f);
        s.arm();
        CHECK(s.armed(), "%s: arms after import (%s)", name.c_str(), s.f.arming_disable_text);
        s.rc.ch[2] = 0.0f;
        s.run(1.0f);
        CHECK(s.f.position[1] > 1.0f, "%s: flies after import (%.2f m)", name.c_str(), s.f.position[1]);
        s.destroy();
    }
    CHECK(files >= 3, "diff corpus has %d files", files);
    char *report = nullptr;
    CHECK(fpv_import_betaflight_diff("hello world", &report) != FPV_OK || (report && std::strstr(report, "Rejected")),
          "garbage is not silently accepted");
    fpv_free(report);
}

void testPinned() {
#if defined(__EMSCRIPTEN__)
    // linked statically into the WebAssembly module: nothing can unload it
#elif defined(_WIN32)
    HMODULE h = nullptr;
    CHECK(GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCWSTR)(void *)&fpv_global_init, &h), "module");
    // FreeLibrary on a pinned module is a no-op; it must stay loaded.
    FreeLibrary(h);
    FreeLibrary(h);
    HMODULE again = nullptr;
    CHECK(GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                             (LPCWSTR)(void *)&fpv_global_init, &again),
          "libfpv still mapped after FreeLibrary");
#else
    Dl_info info;
    CHECK(dladdr((void *)&fpv_global_init, &info) != 0, "dladdr");
    void *h = dlopen(info.dli_fname, RTLD_NOW | RTLD_NOLOAD);
    CHECK(h != nullptr, "loaded");
    dlclose(h);
    dlclose(h);
    dlclose(h);
    void *again = dlopen(info.dli_fname, RTLD_NOW | RTLD_NOLOAD);
    CHECK(again != nullptr, "libfpv still mapped after dlclose");
    if (again) dlclose(again);
#endif
    CHECK(fpv_abi_version() == LIBFPV_ABI_VERSION, "still callable");
}

void testTerminateReinit() {
    fpv_global_terminate();
    fpv_global_terminate();  // twice is fine
    CHECK(!fpv_is_initialized(), "terminated");
    CHECK(init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), "re-init after terminate");
    Session s;
    float p[3] = {0, 0, 0};
    loadWorld(makeLevel(Host(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), 100, 0));
    s.spawn(p);
    s.arm();
    CHECK(s.armed(), "arms after re-init (%s)", s.f.arming_disable_text);
    s.destroy();
}

void testThreadedMode() {
    fpv_global_terminate();
    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    cfg.data_dir = g_dataDir.c_str();
    // real device input (SDL) and the Configurator server, as in a game
    cfg.flags = 0;
    cfg.configurator_port = 5799;
    cfg.log = logCb;
    CHECK(fpv_global_init(&cfg) == FPV_OK, "threaded init: %s", fpv_last_error());
    fpv_input_status ist{};
    ist.struct_size = sizeof(ist);
    CHECK(fpv_input_get_status(&ist) == FPV_OK, "input status");
    Host h(cfg.coords);
    loadWorld(makeLevel(h, 100, 0));
    fpv_drone *d = nullptr;
    CHECK(fpv_drone_create(nullptr, &d) == FPV_OK, "create");
    fpv_frame f{};
    f.struct_size = sizeof(f);
    float ch[8] = {0, 0, -1, 0, -1, -1, -1, -1};
    auto t0 = std::chrono::steady_clock::now();
    double simStart = -1;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(1500)) {
        double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        ch[4] = el > 0.3 ? 1.0f : -1.0f;
        ch[2] = el > 0.8 ? 0.0f : -1.0f;
        fpv_input_set_channels(ch, 8);
        fpv_drone_tick(d, 0.016f, &f);
        if (simStart < 0) simStart = f.sim_time;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    double simElapsed = f.sim_time - simStart;
    std::printf("    threaded: %.3f s simulated in 1.5 s wall\n", simElapsed);
    CHECK(simElapsed > 1.3 && simElapsed < 1.6, "real-time pacing (%.3f s)", simElapsed);
    CHECK(f.flags & FPV_DRONE_ARMED, "armed in threaded mode (%s)", f.arming_disable_text);
    CHECK(f.position[1] > 0.5f, "flying in threaded mode (%.2f)", f.position[1]);
    // auto-pause: stop ticking, sim time must stop advancing
    double before = f.sim_time;
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    fpv_drone_tick(d, 0.016f, &f);
    std::printf("    auto-pause: %.3f s advanced while the host did not tick for 1.2 s\n", f.sim_time - before);
    CHECK(f.sim_time - before < 0.7, "auto-paused when the host stopped ticking");
    fpv_drone_delete(d);
    fpv_global_terminate();
}

// --- wind ------------------------------------------------------------------------

void setWind(const Host &h, float right, float fwd, uint32_t flags = 0, float turbulence = -1) {
    fpv_wind w;
    fpv_wind_defaults(&w);
    w.turbulence = turbulence;
    for (int i = 0; i < 3; i++) w.velocity[i] = (h.R[i] * right + h.F[i] * fwd) * h.upm;
    w.flags = flags;
    CHECK(fpv_wind_set(&w) == FPV_OK, "wind_set: %s", fpv_last_error());
}

// Mean and peak of the air's speed at the drone over `seconds`, and its mean
// along a host axis (up by default).
void windAtDrone(Session &s, const Host &h, float seconds, float &mean, float &peak, float *meanAlong = nullptr,
                 const float *axis = nullptr) {
    double sum = 0, sumUp = 0;
    int n = 0;
    peak = 0;
    s.run(seconds, [&](float) {
        float l = std::sqrt(s.f.wind[0] * s.f.wind[0] + s.f.wind[1] * s.f.wind[1] + s.f.wind[2] * s.f.wind[2]) / h.upm;
        sum += l;
        sumUp += h.dir(s.f.wind, axis ? axis : h.U) / h.upm;
        peak = std::max(peak, l);
        n++;
    });
    mean = n ? (float)(sum / n) : 0;
    if (meanAlong) *meanAlong = n ? (float)(sumUp / n) : 0;
}

// A hovering quad in angle mode with centred sticks drifts with the wind:
// still air keeps it in place, 8 m/s carries it downwind, gusting.
void testWindDrift() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    fpv_global_terminate();  // earlier tests may have left another convention
    init(h.cs);
    loadWorld(makeLevel(h, 300, 0));
    float thr = findHoverThrottle(h);
    Session s;
    hoverFromRest(s, h, thr);
    float x0 = h.right(s.f.position);
    s.run(3.0f);
    float still = std::fabs(h.right(s.f.position) - x0);
    float wx = h.dir(s.f.wind, h.R) / h.upm;
    CHECK(std::fabs(wx) < 1e-4f, "no wind by default, %.4f m/s", wx);

    setWind(h, 8, 0);
    float p[3];
    h.pos(0, 50, 0, p);
    float rot[4] = {0, 0, 0, 1};
    fpv_drone_reset(s.drone, p, rot);
    s.run(0.01f);
    x0 = h.right(s.f.position);
    double gust = 0, gust2 = 0;
    int n = 0;
    s.run(4.0f, [&](float) {
        float v = h.dir(s.f.wind, h.R) / h.upm;
        gust += v;
        gust2 += v * v;
        n++;
    });
    float mean = (float)(gust / n), sd = (float)std::sqrt(std::max(0.0, gust2 / n - (gust / n) * (gust / n)));
    float drift = h.right(s.f.position) - x0;
    std::printf("    still air drift %.2f m; 8 m/s at 10 m: %.1f m/s at 50 m (sd %.2f), drifted %.1f m in 4 s\n", still,
                mean, sd, drift);
    CHECK(still < 1.0f, "still air: no drift, %.2f m", still);
    CHECK(mean > 8.5f && mean < 17.0f, "more wind at 50 m than at 10 m (about 11.7 m/s, gusting), %.1f m/s", mean);
    CHECK(sd > 0.4f && sd < 4.0f, "gusts, sd %.2f m/s", sd);
    CHECK(drift > 4.0f, "carried downwind, %.1f m", drift);
    CHECK(s.armed(), "still flying");
    fpv_wind_set(nullptr);
    s.destroy();
}

// Behind a 60 m wall (upwind of the drone) the wind is mostly held off;
// far from it, in the open, it is not. Without gusts, so the mean wind is
// what is measured. Indoors (the host's flag) there is none. In Z-up
// left-handed centimetres, for the conversions.
void testWindShelter() {
    Host h(fpv_coords_preset(FPV_COORDS_Z_UP_LEFT_HANDED, 100));
    fpv_global_terminate();
    init(h.cs);
    loadWorld(makeLevel(h, 300, 40));  // the wall across forward = 40
    setWind(h, 0, -8, 0, 0);           // blowing from beyond the wall towards the drone, no gusts
    float p[3], open, lee, peak, openAlong, leeAlong, back[3];
    for (int i = 0; i < 3; i++) back[i] = -h.F[i];  // the way the wind blows
    Session s;
    h.pos(0, 0, -150, p);
    s.spawn(p);
    s.run(1.0f);
    windAtDrone(s, h, 1.0f, open, peak, &openAlong, back);
    s.destroy();
    h.pos(0, 0, 30, p);  // 10 m behind the wall
    s.spawn(p);
    s.run(1.0f);
    windAtDrone(s, h, 1.0f, lee, peak, &leeAlong, back);
    setWind(h, 0, -8, FPV_WIND_ENCLOSED, 0);
    s.run(0.5f);
    float inside;
    windAtDrone(s, h, 1.0f, inside, peak);
    std::printf("    on the ground: open %.2f m/s (%.2f mean along), 10 m behind the wall %.2f m/s (%.2f), "
                "enclosed %.2f m/s\n",
                open, openAlong, lee, leeAlong, inside);
    CHECK(openAlong > 1.5f, "wind in the open, %.2f m/s", openAlong);
    CHECK(std::fabs(leeAlong) < 0.2f * openAlong, "the wall holds the wind off, %.2f vs %.2f m/s", leeAlong, openAlong);
    CHECK(inside < 0.05f, "no weather indoors, %.2f m/s", inside);
    fpv_wind_set(nullptr);
    s.destroy();
}

// A rotor hovering above the drone blows it down; a blast pushes it away
// for a moment; neither lasts after the source is gone.
void testWindSources() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    fpv_global_terminate();
    init(h.cs);
    loadWorld(makeLevel(h, 300, 0));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(0.5f);
    fpv_wind_source src{};
    h.pos(0, 8, 0, src.position);
    for (int i = 0; i < 3; i++) src.direction[i] = -h.U[i];
    src.speed = 10 * h.upm;
    src.radius = 5 * h.upm;
    src.type = FPV_WIND_SOURCE_ROTOR;
    CHECK(fpv_wind_set_sources(&src, 1) == FPV_OK, "set_sources: %s", fpv_last_error());
    // on the ground under it: the wake lands here and turns outwards
    float ground, peak, groundUp;
    windAtDrone(s, h, 0.5f, ground, peak, &groundUp);
    // in the column, 4 m below the disc
    float rot[4] = {0, 0, 0, 1};
    h.pos(0, 4, 0, p);
    fpv_drone_reset(s.drone, p, rot);
    float wash, up;
    windAtDrone(s, h, 0.3f, wash, peak, &up);
    // beside the rotor, on the ground: the wake spreads outwards
    h.pos(9, 0, 0, p);
    fpv_drone_reset(s.drone, p, rot);
    float out, outPeak;
    s.run(0.2f);
    windAtDrone(s, h, 0.5f, out, outPeak);
    float outward = h.dir(s.f.wind, h.R) / h.upm;
    fpv_wind_set_sources(nullptr, 0);
    s.run(0.05f);  // the simulation takes the change at its next step
    float after;
    windAtDrone(s, h, 0.3f, after, peak);
    h.pos(15, 0, 0, p);
    fpv_wind_add_blast(p, 3 * h.upm);  // 15 m from the drone (at right 0)
    h.pos(0, 0, 0, p);
    fpv_drone_reset(s.drone, p, rot);
    float blast, blastPeak;
    windAtDrone(s, h, 1.5f, blast, blastPeak);
    float calm;
    windAtDrone(s, h, 0.5f, calm, peak);
    std::printf("    downwash 4 m under the rotor %.1f m/s (%.1f down), on the ground under it %.1f down, beside it "
                "%.1f m/s (%.1f outwards); blast peak %.1f m/s, then %.2f\n",
                wash, -up, -groundUp, out, outward, blastPeak, calm);
    CHECK(-up > 6.0f, "downwash 4 m under a 10 m/s rotor (turning towards the ground), %.1f m/s down", -up);
    CHECK(std::fabs(groundUp) < 0.5f * -up, "turns at the ground, %.1f m/s down", -groundUp);
    CHECK(outward > 2.0f, "the wake spreads out along the ground, %.1f m/s", outward);
    CHECK(after < 0.05f, "gone with the source, %.2f m/s", after);
    CHECK(blastPeak > 3.0f && blastPeak < 40.0f, "a blast 15 m away gusts, %.1f m/s", blastPeak);
    CHECK(calm < 0.05f, "and passes, %.2f m/s", calm);
    s.destroy();
}

// GPS OSD elements, read off the OSD grid: home distance and arrow, heading,
// speed, distance flown, altitude and vario, with Betaflight's home (where it armed)
// and with the host's (fpv_gps_set_home). North is the host's forward axis.
void testGps() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    fpv_global_terminate();  // earlier tests may have left another convention
    init(h.cs);
    loadWorld(makeLevel(h, 300, 0));
    Session s;
    // the element at row `row` (column 1): its first character and its
    // number ("12.5" for altitude and vario), -1 when there is none
    auto cell = [&](int row) { return s.f.osd[row * FPV_OSD_COLS + 1]; };
    auto number = [&](int row) {
        std::string digits;
        for (int x = 1; x < 12; x++) {
            uint8_t c = s.f.osd[row * FPV_OSD_COLS + x];
            if ((c >= '0' && c <= '9') || (c == '.' && !digits.empty())) digits += (char)c;
            else if (!digits.empty()) break;
        }
        return digits.empty() ? -1.0f : (float)std::atof(digits.c_str());
    };
    // Betaflight's 16 arrows (SYM_ARROW_SOUTH = 0x60 onwards) as a bearing
    // relative to the nose: 0 = ahead, 4 = right, 8 = behind
    auto arrow = [&](int row) { return (24 - (cell(row) - 0x60)) % 16; };
    auto arrowFor = [](float relDeg) { return (int)std::lround(std::fmod(relDeg + 720.0f, 360.0f) / 22.5f) % 16; };
    auto arrowNear = [](int a, int b) { int d = std::abs(a - b) % 16; return std::min(d, 16 - d) <= 1; };
    // arms and waits out Betaflight's ARMED splash, which hides the elements.
    // The second before arming lets Betaflight's attitude settle after a
    // reset, so the angle check does not block the arming.
    auto armAndShow = [&] {
        s.rc.ch[4] = -1;
        s.run(1.0f);
        s.arm();
        s.run(1.8f);
    };

    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);  // first: each preset has its own Betaflight configuration
    auto osdPos = [](int x, int y) { return (x & 31) | ((y & 31) << 5) | (1 << 11); };
    std::string cmd;
    const char *elements[] = {"home_dist", "home_dir", "nheading", "altitude", "gps_speed", "flight_dist", "nvario"};
    for (int i = 0; i < 7; i++) cmd += "set osd_" + std::string(elements[i]) + "_pos = " + std::to_string(osdPos(1, 3 + i)) + "\n";
    char *out = nullptr;
    CHECK(fpv_cli_exec((cmd + "save").c_str(), &out) == FPV_OK, "osd setup: %s", fpv_last_error());
    fpv_free(out);

    armAndShow();
    CHECK(s.armed(), "armed");
    CHECK(number(3) == 0, "home distance %g m at the arming point", number(3));

    // 30 m east, 40 m north, 10 m up of the arming point, facing north. A
    // full OSD redraw takes about 0.45 s, so wait for two of them: after only
    // 0.5 s the reading depended on where in its cycle the earlier tests left
    // the OSD.
    h.pos(30, 10, 40, p);
    fpv_drone_reset(s.drone, p, nullptr);
    s.run(1.0f);
    std::printf("    Betaflight home: %g m, arrow %d, heading %g\n", number(3), arrow(4), number(5));
    CHECK(std::fabs(number(3) - 50) <= 1, "home distance %g m, expected 50", number(3));
    CHECK(arrowNear(arrow(4), arrowFor(216.9f)), "home arrow %d, expected south-west (%d)", arrow(4), arrowFor(216.9f));
    CHECK(number(5) == 0 || number(5) == 359, "heading %g facing north (host forward)", number(5));

    // host home 100 m north of the origin; turn to face east
    h.pos(0, 0, 100, p);
    CHECK(fpv_gps_set_home(p) == FPV_OK, "set home: %s", fpv_last_error());
    float east[4] = {0, -0.70710678f, 0, 0.70710678f};  // -90 deg about +Y: -Z (forward) turns to +X (right)
    h.pos(30, 10, 40, p);
    fpv_drone_reset(s.drone, p, east);
    s.run(1.0f);
    std::printf("    host home: %g m, arrow %d, heading %g\n", number(3), arrow(4), number(5));
    CHECK(std::fabs(number(3) - 67) <= 1, "home distance %g m, expected 67", number(3));
    CHECK(std::fabs(number(5) - 90) <= 1, "heading %g facing east (host right)", number(5));
    // home is at 333 deg, 243 deg from the nose
    CHECK(arrowNear(arrow(4), arrowFor(333.4f - 90.0f)), "home arrow %d, expected %d", arrow(4), arrowFor(243.4f));

    // re-arming keeps the host's home. Land level first: Betaflight will not
    // arm a quad still tumbling down from 10 m (ANGLE, RUNAWAY).
    h.pos(30, 0, 40, p);
    fpv_drone_reset(s.drone, p, east);
    armAndShow();
    CHECK(s.armed() && std::fabs(number(3) - 67) <= 1, "home distance %g m after re-arming", number(3));

    // speed and distance flown, in ANGLE mode with the nose down
    h.pos(0, 30, 0, p);
    fpv_drone_reset(s.drone, p, nullptr);
    s.rc.ch[5] = 1;
    s.rc.ch[2] = -0.2f;
    s.rc.ch[1] = 0.5f;
    // like altitude below, a reading must match the speed of the last second
    float worst = 0, kmh = 0;
    std::vector<float> speeds;
    s.run(3.0f, [&](float t) {
        kmh = std::sqrt(s.f.velocity[0] * s.f.velocity[0] + s.f.velocity[2] * s.f.velocity[2]) * 3.6f;
        speeds.push_back(kmh);
        if (t <= 2.0f) return;  // after the speed-up
        float gap = 1e9f;
        for (size_t i = speeds.size() > 100 ? speeds.size() - 100 : 0; i < speeds.size(); i++)
            gap = std::min(gap, std::fabs(number(7) - speeds[i]));
        worst = std::max(worst, gap);
    });
    std::printf("    speed: OSD %g km/h, drone %.0f km/h (worst gap %.1f); flown %g m\n", number(7), kmh, worst, number(8));
    CHECK(kmh > 20 && worst < 0.1f * kmh + 2, "OSD speed within %.1f km/h of %.0f", worst, kmh);
    CHECK(number(8) > 10, "distance flown %g m", number(8));

    // altitude and vario in a climb. The OSD redraws each element only a few
    // times a second, so a reading must match the truth of the last second.
    h.pos(0, 5, 0, p);
    fpv_drone_reset(s.drone, p, nullptr);
    s.rc.ch[2] = 0.2f;
    std::vector<std::pair<float, float>> truth;  // altitude, climb rate
    int altBad = 0, varioBad = 0, checked = 0;
    s.run(3.0f, [&](float t) {
        truth.push_back({h.up(s.f.position), h.dir(s.f.velocity, h.U) / h.upm});
        if (t < 1.0f) return;  // the jump from the previous position settles
        float alt = number(6), vario = number(9);
        bool altOk = false, varioOk = false;
        for (size_t i = truth.size() > 100 ? truth.size() - 100 : 0; i < truth.size(); i++) {
            altOk |= std::fabs(alt - truth[i].first) < 1.5f;
            varioOk |= std::fabs(vario - truth[i].second) < 0.1f * truth[i].second + 1.5f;
        }
        altBad += !altOk;
        varioBad += !varioOk && t >= 2.0f;  // the vario's own 1 Hz filter lags the speed-up
        checked++;
    });
    float vz = truth.back().second;
    std::printf("    climb: %.1f m/s, OSD altitude %g m (true %.1f), vario %g m/s; %d/%d altitude and %d/%d vario "
                "readings off\n", vz, number(6), truth.back().first, number(9), altBad, checked, varioBad, checked);
    CHECK(vz > 5 && cell(9) == 0x75, "climbing %.1f m/s, vario arrow up (%02x)", vz, cell(9));
    CHECK(altBad == 0, "%d altitude readings off", altBad);
    CHECK(varioBad == 0, "%d vario readings off", varioBad);

    // back to Betaflight's home: the next arming point
    CHECK(fpv_gps_set_home(nullptr) == FPV_OK, "clear home");
    s.rc.ch[1] = 0;  // pitch from the speed run: with air mode it lifts the quad off at idle
    s.rc.ch[2] = -1;
    s.rc.ch[5] = -1;
    h.pos(-20, 0, 0, p);
    fpv_drone_reset(s.drone, p, nullptr);
    armAndShow();
    CHECK(number(3) >= 0 && number(3) <= 1, "home distance %g m at the new arming point", number(3));
    s.destroy();
}

void testOsdAudioOverlay() {
    Session s;
    float p[3] = {0, 0, 0};
    s.spawn(p);
    s.run(0.5f);
    int chars = 0;
    for (uint8_t c : s.f.osd) chars += (c != 0 && c != ' ');
    CHECK(chars > 10, "Betaflight OSD has content (%d characters)", chars);
    CHECK(!(s.f.flags & FPV_DRONE_OSD_DISABLED), "OSD enabled by default");
    // the OSD disable switch (AUX6) blanks it and tells the host
    auto osdChars = [&] {
        int n = 0;
        for (uint8_t c : s.f.osd) n += (c != 0 && c != ' ');
        return n;
    };
    s.rc.ch[9] = 1;
    s.run(0.5f);
    CHECK(s.f.flags & FPV_DRONE_OSD_DISABLED, "OSD disabled flag on AUX6");
    CHECK(osdChars() == 0, "OSD blank while disabled (%d characters)", osdChars());
    s.rc.ch[9] = -1;
    s.run(0.5f);
    CHECK(!(s.f.flags & FPV_DRONE_OSD_DISABLED), "OSD disabled flag cleared");
    CHECK(osdChars() == chars, "OSD back after the switch (%d of %d characters)", osdChars(), chars);
    fpv_texture tex{};
    CHECK(fpv_texture_get(FPV_TEXTURE_OSD_FONT, &tex) == FPV_OK && tex.width > 0 && tex.rgba, "OSD font texture");
    for (int style = 0; style < 2; style++) {
        const fpv_draw_data *dd = fpv_osd_draw_data(&s.f, 1280, 720, (fpv_osd_style)style);
        CHECK(dd && dd->list_count == 1 && dd->total_index_count == (uint32_t)chars * 6, "OSD quads style %d", style);
        if (dd && dd->list_count) CHECK(dd->lists[0].commands[0].texture_id == FPV_TEXTURE_OSD_FONT, "OSD texture id");
    }
    std::vector<uint8_t> img(640 * 480 * 4);
    CHECK(fpv_osd_render_rgba(&s.f, img.data(), 640, 480, 0, FPV_OSD_STYLE_ANALOG) == FPV_OK, "OSD raster");
    size_t opaque = 0;
    for (size_t i = 3; i < img.size(); i += 4) opaque += img[i] != 0;
    CHECK(opaque > 500, "OSD raster has pixels (%zu)", opaque);

    // audio: silent when disarmed, motor sound when spinning
    CHECK(fpv_audio_configure(48000, 2) == FPV_OK, "audio configure");
    fpv_audio_set_volume(1.0f);  // full scale, so the check below does not follow the default volume
    std::vector<float> buf(4800 * 2);
    auto energy = [&] {
        fpv_audio_read(buf.data(), 4800);
        double e = 0;
        for (float v : buf) e += v * v;
        return e / buf.size();
    };
    energy();
    double quiet = energy();
    s.arm();
    s.rc.ch[2] = 0.0f;
    s.run(0.5f);
    energy();
    double loud = energy();
    std::printf("    audio energy disarmed %.2e, flying %.2e\n", quiet, loud);
    CHECK(quiet < 1e-6, "silent when the motors are stopped");
    CHECK(loud > 1e-3, "motor sound when flying");
    for (float v : buf) {
        if (!(std::fabs(v) <= 1.0f)) {
            CHECK(false, "audio sample out of range %f", v);
            break;
        }
    }

    // overlay
    CHECK(fpv_overlay_render(0.016f) == nullptr, "closed overlay draws nothing");
    fpv_overlay_set_display(1280, 720, 1.0f);
    fpv_overlay_set_open(1);
    CHECK(fpv_overlay_wants_input(), "open overlay wants input");
    const fpv_draw_data *od = nullptr;
    for (int i = 0; i < 3; i++) od = fpv_overlay_render(0.016f);
    CHECK(od && od->total_vertex_count > 100, "overlay draw data (%u vertices)", od ? od->total_vertex_count : 0);
    bool fontCmd = false;
    for (uint32_t l = 0; od && l < od->list_count; l++)
        for (uint32_t c = 0; c < od->lists[l].command_count; c++) {
            const fpv_draw_cmd &cmd = od->lists[l].commands[c];
            fontCmd |= cmd.texture_id == FPV_TEXTURE_OVERLAY_FONT;
            CHECK(cmd.index_offset + cmd.index_count <= od->lists[l].index_count, "command within index buffer");
        }
    CHECK(fontCmd, "overlay uses its font texture");
    CHECK(fpv_texture_get(FPV_TEXTURE_OVERLAY_FONT, &tex) == FPV_OK && tex.width >= 256, "overlay font %ux%u", tex.width,
          tex.height);
    // host buttons render in the Fly tab; nothing is clicked yet
    fpv_overlay_set_action(7, "Move player here");
    fpv_overlay_set_action(7, "Move the player here");
    CHECK(fpv_overlay_render(0.016f) != nullptr, "renders with a host button");
    CHECK(fpv_overlay_poll_action() == 0, "no host button clicked");
    fpv_overlay_set_action(7, nullptr);
    CHECK(fpv_overlay_render(0.016f) != nullptr, "renders after removing the host button");
    // click through every tab (the tab bar is at the top of the centred window)
    for (int tab = 0; tab < 10; tab++) {
        fpv_overlay_mouse_move(0, 0);
        od = fpv_overlay_render(0.016f);
        CHECK(od != nullptr, "renders with tab %d", tab);
    }
    // the toggle key closes it; flight keys do not leak while it is open
    fpv_input_key(FPV_KEY_F8, 1);
    fpv_input_key(FPV_KEY_F8, 0);
    CHECK(!fpv_overlay_is_open(), "F8 closes the overlay");
    fpv_overlay_set_open(1);
    fpv_overlay_render(0.016f);
    fpv_input_key(FPV_KEY_ESCAPE, 1);
    fpv_input_key(FPV_KEY_ESCAPE, 0);
    CHECK(!fpv_overlay_is_open(), "Esc closes the overlay");
    s.destroy();
}

#if defined(LIBFPV_KAMIKAZE)
// --- kamikaze builds -----------------------------------------------------------------

constexpr int kBomb = 10, kExplode = 11;  // AUX7, AUX8

bool osdHas(const fpv_frame &f, const char *text) {
    for (int r = 0; r < FPV_OSD_ROWS; r++) {
        std::string row((const char *)&f.osd[r * FPV_OSD_COLS], FPV_OSD_COLS);
        if (row.find(text) != std::string::npos) return true;
    }
    return false;
}

bool osdBlank(const fpv_frame &f) {
    for (int i = 0; i < FPV_OSD_ROWS * FPV_OSD_COLS; i++)
        if (f.osd[i] != ' ') return false;
    return true;
}

fpv_kamikaze_state warhead(fpv_drone *d) {
    fpv_kamikaze_state k{};
    k.struct_size = sizeof k;
    fpv_kamikaze_get_state(d, &k);
    return k;
}

// Takes off in angle mode with a short climb, then throttles back to
// about a slow climb or hover, arm switch on.
void takeOff(Session &s, float seconds) {
    s.rc.ch[5] = 1;  // angle
    s.arm();
    s.rc.ch[2] = -0.2f;
    s.run(seconds);
    s.rc.ch[2] = -0.45f;
}

// Betaflight's post-flight stats cover the OSD (bomb line too) for 60 s
// after a disarm; pitch up dismisses them, as on a real quad.
void skipStats(Session &s) {
    s.rc.ch[1] = 1;
    s.run(0.2f);
    s.rc.ch[1] = 0;
    s.run(0.5f);
}

int drainDetonations(fpv_detonation *last = nullptr) {
    int n = 0;
    fpv_detonation d{};
    d.struct_size = sizeof d;
    while (fpv_kamikaze_poll_detonation(&d)) {
        n++;
        if (last) *last = d;
        d.struct_size = sizeof d;
    }
    return n;
}

// The motors do not arm with the bomb arm switch on; after it is turned
// off, the arm switch must be cycled.
void testKamikazeArmingBlock() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(1.0f);  // past Betaflight's boot logo
    skipStats(s);  // an earlier test may have flown
    CHECK(warhead(s.drone).warhead == FPV_WARHEAD_SAFE, "new drone: warhead safe");
    CHECK(osdHas(s.f, "BOMB_SAFE"), "OSD shows BOMB_SAFE");
    s.rc.ch[kBomb] = 1;
    s.run(0.3f);
    s.arm();
    CHECK(!s.armed(), "does not arm with the bomb arm switch on");
    fpv_kamikaze_state k = warhead(s.drone);
    CHECK(k.flags & FPV_KAMIKAZE_BLOCKS_ARMING, "BLOCKS_ARMING set (flags %x)", k.flags);
    CHECK(k.warhead == FPV_WARHEAD_SAFE, "a disarmed drone does not arm its warhead (%d)", k.warhead);
    CHECK(std::strstr(s.f.arming_disable_text, "BOMB") != nullptr, "reports BOMB: \"%s\"", s.f.arming_disable_text);
    bool shown = false;
    s.run(1.0f, [&](float) { shown |= osdHas(s.f, "BOMB SW ON"); });
    CHECK(shown, "OSD warns BOMB SW ON");
    s.rc.ch[kBomb] = -1;  // bomb safe, arm switch still on
    s.run(0.5f);
    CHECK(!s.armed(), "still not armed until the arm switch is cycled");
    s.arm();
    CHECK(s.armed(), "arms after cycling the arm switch (%s)", s.f.arming_disable_text);
    CHECK(drainDetonations() == 0, "nothing exploded");
    s.destroy();
}

// In flight: bomb arm switch -> ARMING countdown -> live ("BOMB_ARMED"); the
// warhead stays live when the motors disarm, and the fall detonates it.
void testKamikazeImpact() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(0.3f);
    drainDetonations();
    fpv_kamikaze_config cfg;
    fpv_kamikaze_config_defaults(&cfg);
    CHECK(cfg.arming_delay == 0.0f, "no arming delay by default (%.1f)", cfg.arming_delay);
    cfg.arming_delay = 2.0f;  // a short flight: this quad climbs fast
    fpv_kamikaze_set_config(&cfg);
    takeOff(s, 0.4f);
    CHECK(s.armed() && h.up(s.f.position) > 0.5f, "flying, %.1f m up", h.up(s.f.position));
    s.rc.ch[kBomb] = 1;
    s.run(0.45f);  // Betaflight redraws the OSD every ~0.3 s
    fpv_kamikaze_state k = warhead(s.drone);
    CHECK(k.warhead == FPV_WARHEAD_ARMING && k.arming_remaining > 1.5f && k.arming_remaining < 1.6f,
          "arming, %.2f s left (state %d)", k.arming_remaining, k.warhead);
    CHECK(osdHas(s.f, "ARMING 2"), "OSD counts down ARMING 2");
    // switching off during the countdown safes it
    s.rc.ch[kBomb] = -1;
    s.run(0.1f);
    CHECK(warhead(s.drone).warhead == FPV_WARHEAD_SAFE, "safed during the countdown");
    s.rc.ch[kBomb] = 1;
    s.run(2.5f);
    CHECK(warhead(s.drone).warhead == FPV_WARHEAD_ARMED, "live after the delay (%d)", warhead(s.drone).warhead);
    CHECK(osdHas(s.f, "BOMB_ARMED"), "OSD shows BOMB_ARMED");
    CHECK(drainDetonations() == 0, "nothing exploded in the air");
    float height = h.up(s.f.position);
    CHECK(height > 2, "high enough for the fall, %.1f m", height);
    s.rc.ch[4] = -1;  // motors off: it falls
    bool sawDisarmedLive = false;
    s.run(6.0f, [&](float) {
        if (!s.armed() && warhead(s.drone).warhead == FPV_WARHEAD_ARMED) sawDisarmedLive = true;
    });
    CHECK(sawDisarmedLive, "warhead stays live with the motors disarmed");
    fpv_detonation d{};
    CHECK(drainDetonations(&d) == 1, "one detonation");
    CHECK(d.cause == FPV_DETONATION_IMPACT, "cause impact (%d)", d.cause);
    CHECK(std::fabs(h.up(d.position)) < 0.3f, "exploded on the ground, %.2f m", h.up(d.position));
    CHECK(h.dir(d.normal, h.U) > 0.9f, "ground normal");
    CHECK(s.f.flags & FPV_DRONE_DETONATED, "DETONATED flag");
    CHECK(s.f.flags & FPV_DRONE_CRASHED, "CRASHED flag");
    CHECK(s.f.video.signal_strength == 0 && s.f.video.noise == 1, "no video signal");
    CHECK(s.f.motors[0].rpm < 1, "motors stopped (%.0f rpm)", s.f.motors[0].rpm);
    CHECK(warhead(s.drone).detonation_count >= 1, "counted");
    CHECK(osdBlank(s.f), "OSD blank after the explosion");
    float frozen[3];
    std::memcpy(frozen, s.f.position, sizeof frozen);
    s.rc.ch[4] = 1;
    s.rc.ch[kBomb] = -1;
    s.run(0.5f);
    CHECK(!s.armed(), "a destroyed drone does not arm");
    CHECK(std::fabs(s.f.position[1] - frozen[1]) < 1e-3f, "the wreck stays put");
    // repair = new drone with a new, safe warhead
    fpv_drone_repair(s.drone);
    fpv_drone_reset(s.drone, p, nullptr);
    s.rc.ch[kBomb] = 1;
    s.run(0.3f);
    k = warhead(s.drone);
    CHECK(k.warhead == FPV_WARHEAD_SAFE && !(s.f.flags & FPV_DRONE_DETONATED), "repaired: safe, not destroyed");
    s.rc.ch[kBomb] = -1;
    s.rc.ch[4] = -1;
    s.run(0.3f);
    s.arm();
    CHECK(s.armed(), "repaired drone arms (%s)", s.f.arming_disable_text);
    fpv_kamikaze_config_defaults(&cfg);
    fpv_kamikaze_set_config(&cfg);
    s.destroy();
}

// The explode switch works only on a live warhead; host detonation too,
// unless forced. Without the bomb switch, crashes do not explode.
void testKamikazeExplodeSwitch() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(0.3f);
    drainDetonations();
    fpv_kamikaze_config cfg;
    fpv_kamikaze_config_defaults(&cfg);
    cfg.arming_delay = 1.0f;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_OK, "set config: %s", fpv_last_error());
    takeOff(s, 0.4f);
    s.rc.ch[kExplode] = 1;  // bomb safe: refused
    bool refused = false;
    s.run(1.2f, [&](float) { refused |= osdHas(s.f, "NOT ARMED"); });
    CHECK(refused, "OSD shows NOT ARMED");
    s.rc.ch[kExplode] = -1;
    CHECK(fpv_kamikaze_detonate(s.drone, 0) == FPV_ERROR_NOT_FOUND, "host detonation refused while safe");
    CHECK(drainDetonations() == 0 && !(s.f.flags & FPV_DRONE_DETONATED), "nothing exploded");
    s.rc.ch[kBomb] = 1;
    s.run(1.3f);
    CHECK(warhead(s.drone).warhead == FPV_WARHEAD_ARMED, "live after the 1 s delay");
    s.rc.ch[kExplode] = 1;
    bool boom = false;
    s.run(0.2f, [&](float) { boom |= osdBlank(s.f); });
    fpv_detonation d{};
    CHECK(drainDetonations(&d) == 1 && d.cause == FPV_DETONATION_SWITCH, "explode switch detonates (cause %d)", d.cause);
    CHECK(h.up(d.position) > 1, "in the air, %.1f m", h.up(d.position));
    CHECK(boom, "OSD blank after the explosion");

    // forced host detonation of a safe warhead
    fpv_drone_repair(s.drone);
    fpv_drone_reset(s.drone, p, nullptr);
    s.rc.ch[kBomb] = s.rc.ch[kExplode] = -1;
    s.run(0.3f);
    CHECK(fpv_kamikaze_detonate(s.drone, 1) == FPV_OK, "forced detonation");
    s.run(0.05f);
    CHECK(drainDetonations(&d) == 1 && d.cause == FPV_DETONATION_HOST, "host cause (%d)", d.cause);
    CHECK(s.f.flags & FPV_DRONE_DETONATED, "destroyed");

    // a crash with the bomb safe does not explode
    fpv_drone_repair(s.drone);
    fpv_drone_reset(s.drone, p, nullptr);
    s.rc.ch[4] = -1;
    s.run(0.3f);
    takeOff(s, 0.4f);
    s.rc.ch[4] = -1;
    s.run(5.0f);
    CHECK(drainDetonations() == 0 && !(s.f.flags & FPV_DRONE_DETONATED), "safe crash: no explosion");
    fpv_kamikaze_config_defaults(&cfg);
    fpv_kamikaze_set_config(&cfg);
    s.destroy();
}

// A contact fuse: a light tap on the ground sets it off, resting does not.
void testKamikazeGroundTap() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    Session s;
    float p[3];
    h.pos(0, 0, 0, p);
    s.spawn(p);
    s.run(0.3f);
    drainDetonations();
    s.arm();  // motors idle on the ground
    s.rc.ch[kBomb] = 1;
    s.run(3.5f);
    CHECK(warhead(s.drone).warhead == FPV_WARHEAD_ARMED, "live on the ground (%d)", warhead(s.drone).warhead);
    s.run(1.0f);
    CHECK(drainDetonations() == 0 && !(s.f.flags & FPV_DRONE_DETONATED), "resting on the ground does not fire it");
    float up[3];
    h.pos(0, 0.03f, 0, up);  // a 3 cm drop: ~0.75 m/s, below the reported-impact speed
    // a less sensitive fuse (the overlay's "Impact fuse") ignores the tap
    fpv_kamikaze_config cfg;
    fpv_kamikaze_config_defaults(&cfg);
    CHECK(std::fabs(cfg.min_impact_speed - 0.25f) < 1e-4f, "default fuse 0.25 m/s (%.2f)", cfg.min_impact_speed);
    cfg.min_impact_speed = 2.0f;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_OK, "set fuse: %s", fpv_last_error());
    fpv_drone_reset(s.drone, up, nullptr);
    s.run(0.5f);
    CHECK(drainDetonations() == 0, "a 2 m/s fuse ignores the tap");
    cfg.min_impact_speed = 0.01f;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_ERROR_INVALID_ARGUMENT, "fuse below 0.1 m/s refused");
    // the ending duration (the overlay's "Ending"): stored for the game, kept by older hosts
    fpv_kamikaze_config_defaults(&cfg);
    CHECK(cfg.ending_duration == 3.0f, "default ending 3 s (%.1f)", cfg.ending_duration);
    cfg.ending_duration = 5.0f;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_OK, "set ending: %s", fpv_last_error());
    fpv_kamikaze_config_defaults(&cfg);
    cfg.struct_size = offsetof(fpv_kamikaze_config, ending_duration);  // a host built before it
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_OK, "set an old config: %s", fpv_last_error());
    fpv_kamikaze_config got{};
    got.struct_size = sizeof got;
    CHECK(fpv_kamikaze_get_config(&got) == FPV_OK && got.ending_duration == 5.0f,
          "an old host keeps the player's ending (%.1f)", got.ending_duration);
    cfg.struct_size = sizeof cfg;
    cfg.ending_duration = 31.0f;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_ERROR_INVALID_ARGUMENT, "ending over 30 s refused");
    fpv_kamikaze_config_defaults(&cfg);
    fpv_kamikaze_set_config(&cfg);
    fpv_drone_reset(s.drone, up, nullptr);
    uint32_t reported = 0;
    s.run(0.5f, [&](float) { reported += s.f.impact_count; });
    fpv_detonation d{};
    CHECK(drainDetonations(&d) == 1 && d.cause == FPV_DETONATION_IMPACT, "a tap on the ground detonates");
    CHECK(reported == 0, "too soft for fpv_frame.impacts (%u reported)", reported);
    s.destroy();
}

// Without the bomb installed the drone is a plain quad; installed, it is
// live at once (no arming delay by default), its OSD line is a Betaflight
// element, and it stays installed for the next drone.
void testKamikazeInstall() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    float p[3];
    h.pos(0, 0, 0, p);
    auto cli = [](const char *cmd) {
        char *o = nullptr;
        fpv_cli_exec(cmd, &o);
        fpv_free(o);
    };
    CHECK(fpv_kamikaze_set_installed(0) == FPV_OK && !fpv_kamikaze_is_installed(), "uninstalled");
    Session s;
    s.spawn(p);
    s.run(0.5f);
    drainDetonations();
    fpv_kamikaze_state k = warhead(s.drone);
    CHECK(!(k.flags & FPV_KAMIKAZE_INSTALLED) && k.payload_mass == 0, "no bomb, no payload (%.3f kg)", k.payload_mass);
    CHECK(!osdHas(s.f, "BOMB_SAFE") && !osdHas(s.f, "CUSTOM"), "no bomb OSD");
    s.rc.ch[kBomb] = 1;
    s.run(0.2f);
    takeOff(s, 0.4f);
    CHECK(s.armed(), "arms with the bomb arm switch on (%s)", s.f.arming_disable_text);
    s.rc.ch[kExplode] = 1;
    s.run(0.5f);
    k = warhead(s.drone);
    CHECK(k.warhead == FPV_WARHEAD_SAFE && !(k.flags & FPV_KAMIKAZE_BOMB_SWITCH), "switches ignored (%d, %x)", k.warhead,
          k.flags);
    CHECK(!osdHas(s.f, "NOT ARMED") && !osdHas(s.f, "BOMB_ARMED"), "nothing on the OSD");
    CHECK(fpv_kamikaze_detonate(s.drone, 1) == FPV_ERROR_NOT_FOUND, "nothing to detonate");
    CHECK(drainDetonations() == 0, "nothing exploded");
    s.rc.ch[kExplode] = -1;

    // installed in flight, with the bomb switch left on: it must be cycled
    CHECK(fpv_kamikaze_set_installed(1) == FPV_OK && fpv_kamikaze_is_installed(), "installed");
    s.run(0.3f);
    k = warhead(s.drone);
    CHECK((k.flags & FPV_KAMIKAZE_INSTALLED) && k.payload_mass > 0, "carries the payload (%.3f kg)", k.payload_mass);
    CHECK(k.warhead == FPV_WARHEAD_SAFE && (k.flags & FPV_KAMIKAZE_NEEDS_SWITCH_CYCLE), "switch left on: safe");
    s.rc.ch[kBomb] = -1;
    s.run(0.1f);
    s.rc.ch[kBomb] = 1;
    s.run(0.5f);
    CHECK(warhead(s.drone).warhead == FPV_WARHEAD_ARMED, "live at once (%d)", warhead(s.drone).warhead);
    CHECK(osdHas(s.f, "BOMB_ARMED"), "OSD shows BOMB_ARMED");
    // the OSD line is the custom message 1 element: it moves, and the OSD
    // disable switch hides it
    cli("set osd_custom_msg0_pos = 2434\nsave");  // column 2, row 12
    s.run(0.5f);
    std::string row12((const char *)&s.f.osd[12 * FPV_OSD_COLS], FPV_OSD_COLS);
    CHECK(row12.find("BOMB_ARMED") != std::string::npos, "moved to row 12: \"%s\"", row12.c_str());
    s.rc.ch[9] = 1;
    s.run(0.5f);
    CHECK(!osdHas(s.f, "BOMB_ARMED"), "hidden by the OSD disable switch");
    s.rc.ch[9] = -1;
    // removed: safe and light again
    fpv_kamikaze_set_installed(0);
    s.run(0.2f);
    k = warhead(s.drone);
    CHECK(k.warhead == FPV_WARHEAD_SAFE && k.payload_mass == 0, "removed: safe, no payload (%d)", k.warhead);
    CHECK(drainDetonations() == 0, "nothing exploded");
    s.destroy();

    // kept for the next drone
    fpv_kamikaze_set_installed(1);
    Session s2;
    s2.spawn(p);
    s2.run(1.0f);
    skipStats(s2);  // the first drone flew: Betaflight shows its stats
    k = warhead(s2.drone);
    CHECK((k.flags & FPV_KAMIKAZE_INSTALLED) && k.payload_mass > 0, "a new drone carries it too");
    CHECK(osdHas(s2.f, "BOMB_SAFE"), "OSD shows BOMB_SAFE");
    s2.destroy();
}

// The payload is real weight: reported, configurable, and it slows the climb.
void testKamikazePayload() {
    Host h(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    float p[3];
    h.pos(0, 0, 0, p);
    fpv_kamikaze_config cfg;
    auto climb = [&](float payloadKg, float &total) {
        fpv_kamikaze_config_defaults(&cfg);
        cfg.payload_mass = payloadKg;
        CHECK(fpv_kamikaze_set_config(&cfg) == FPV_OK, "set payload: %s", fpv_last_error());
        Session s;
        s.spawn(p);
        s.run(0.5f);
        fpv_kamikaze_state k = warhead(s.drone);
        total = k.total_mass;
        if (payloadKg >= 0)
            CHECK(std::fabs(k.payload_mass - payloadKg) < 1e-4f, "payload %.3f kg reported (%.3f)", payloadKg, k.payload_mass);
        CHECK(std::fabs(h.up(s.f.position)) < 0.15f, "rests on its payload, %.3f m", h.up(s.f.position));
        s.rc.ch[5] = 1;
        s.arm();
        s.rc.ch[2] = 0.0f;
        s.run(0.8f);
        float up = h.up(s.f.position);
        s.destroy();
        return up;
    };
    float autoTotal = 0, bareTotal = 0, heavyTotal = 0;
    climb(-1, autoTotal);
    fpv_kamikaze_config got{};
    got.struct_size = sizeof got;
    CHECK(fpv_kamikaze_get_config(&got) == FPV_OK && got.payload_mass < 0, "automatic payload by default");
    float bare = climb(0, bareTotal);
    float heavy = climb(0.4f, heavyTotal);
    CHECK(autoTotal > bareTotal * 1.5f, "automatic payload is 60%% of the quad: %.3f vs %.3f kg", autoTotal, bareTotal);
    CHECK(std::fabs(heavyTotal - bareTotal - 0.4f) < 1e-3f, "all-up mass %.3f = %.3f + 0.4", heavyTotal, bareTotal);
    CHECK(heavy < bare * 0.8f, "the loaded drone climbs less: %.1f m vs %.1f m", heavy, bare);
    fpv_kamikaze_config_defaults(&cfg);
    fpv_kamikaze_set_config(&cfg);
}

// The ending options (view, duration, then): stored for the game, saved in
// settings.json, kept by older hosts, and limited to what the host can do.
void testKamikazeEnding() {
    auto get = [] {
        fpv_kamikaze_config c{};
        c.struct_size = sizeof c;
        CHECK(fpv_kamikaze_get_config(&c) == FPV_OK, "get config: %s", fpv_last_error());
        return c;
    };
    auto settingsText = [] {
        std::ifstream in(std::filesystem::path(g_dataDir) / "settings.json");
        return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    };
    auto reinit = [] {
        fpv_global_terminate();
        init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    };
    const uint32_t allViews = 0x3, allThens = 0x7;
    fpv_kamikaze_config cfg;
    fpv_kamikaze_config_defaults(&cfg);
    CHECK(cfg.ending_view == FPV_KAMIKAZE_VIEW_SIGNAL_LOST && cfg.ending_then == FPV_KAMIKAZE_THEN_RESPAWN &&
              cfg.ending_duration == 3.0f,
          "defaults: signal lost, 3 s, respawn (%u, %.1f, %u)", cfg.ending_view, cfg.ending_duration, cfg.ending_then);
    cfg.ending_view = FPV_KAMIKAZE_VIEW_CUTSCENE;
    cfg.ending_then = FPV_KAMIKAZE_THEN_RECALL;
    cfg.ending_duration = 0;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_OK, "set ending: %s", fpv_last_error());
    fpv_kamikaze_config c = get();
    CHECK(c.ending_view == FPV_KAMIKAZE_VIEW_CUTSCENE && c.ending_then == FPV_KAMIKAZE_THEN_RECALL &&
              c.ending_duration == 0,
          "cutscene, 0 s, recall (%u, %.1f, %u)", c.ending_view, c.ending_duration, c.ending_then);
    // a host built before the view and then fields keeps the player's
    fpv_kamikaze_config_defaults(&cfg);
    cfg.struct_size = offsetof(fpv_kamikaze_config, ending_view);
    cfg.ending_duration = 4.0f;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_OK, "set an old config: %s", fpv_last_error());
    c = get();
    CHECK(c.ending_view == FPV_KAMIKAZE_VIEW_CUTSCENE && c.ending_then == FPV_KAMIKAZE_THEN_RECALL &&
              c.ending_duration == 4.0f,
          "an old host keeps the player's view and then (%u, %.1f, %u)", c.ending_view, c.ending_duration,
          c.ending_then);
    fpv_kamikaze_config small{};
    small.struct_size = offsetof(fpv_kamikaze_config, ending_view);
    CHECK(fpv_kamikaze_get_config(&small) == FPV_OK && small.struct_size == offsetof(fpv_kamikaze_config, ending_view),
          "an old host reads the old size (%u)", small.struct_size);
    cfg = get();
    cfg.ending_view = 2;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_ERROR_INVALID_ARGUMENT, "unknown view refused");
    cfg.ending_view = FPV_KAMIKAZE_VIEW_CUTSCENE;
    cfg.ending_then = 3;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_ERROR_INVALID_ARGUMENT, "unknown then refused");
    cfg.ending_then = FPV_KAMIKAZE_THEN_RECALL;
    cfg.ending_duration = 30.5f;
    CHECK(fpv_kamikaze_set_config(&cfg) == FPV_ERROR_INVALID_ARGUMENT, "duration over 30 s refused");

    // saved in settings.json, by name
    std::string text = settingsText();
    CHECK(text.find("\"cutscene\"") != std::string::npos && text.find("\"recall\"") != std::string::npos,
          "settings.json has the view and then by name");
    reinit();
    c = get();
    CHECK(c.ending_view == FPV_KAMIKAZE_VIEW_CUTSCENE && c.ending_then == FPV_KAMIKAZE_THEN_RECALL &&
              c.ending_duration == 4.0f,
          "kept across a restart (%u, %.1f, %u)", c.ending_view, c.ending_duration, c.ending_then);

    // a host without the cutscene nor recall (the standalone example)
    CHECK(fpv_kamikaze_set_ending_support(1u << FPV_KAMIKAZE_VIEW_SIGNAL_LOST,
                                          (1u << FPV_KAMIKAZE_THEN_RESPAWN) | (1u << FPV_KAMIKAZE_THEN_NOTHING),
                                          FPV_KAMIKAZE_VIEW_SIGNAL_LOST, FPV_KAMIKAZE_THEN_NOTHING) == FPV_OK,
          "set support: %s", fpv_last_error());
    c = get();
    CHECK(c.ending_view == FPV_KAMIKAZE_VIEW_SIGNAL_LOST && c.ending_then == FPV_KAMIKAZE_THEN_NOTHING,
          "unsupported choices read back as the host's defaults (%u, %u)", c.ending_view, c.ending_then);
    fpv_kamikaze_config_defaults(&cfg);
    CHECK(cfg.ending_then == FPV_KAMIKAZE_THEN_NOTHING, "defaults use the host's then (%u)", cfg.ending_then);
    // ... without forgetting them: a host that supports them sees them again
    CHECK(fpv_kamikaze_set_ending_support(allViews, allThens, FPV_KAMIKAZE_VIEW_SIGNAL_LOST,
                                          FPV_KAMIKAZE_THEN_NOTHING) == FPV_OK,
          "set support: %s", fpv_last_error());
    c = get();
    CHECK(c.ending_view == FPV_KAMIKAZE_VIEW_CUTSCENE && c.ending_then == FPV_KAMIKAZE_THEN_RECALL,
          "a supported saved choice wins over the host default (%u, %u)", c.ending_view, c.ending_then);
    // a player who never chose gets the host's default, even if it supports others
    fpv_global_terminate();
    std::ofstream(std::filesystem::path(g_dataDir) / "settings.json") << "{}";
    init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
    CHECK(fpv_kamikaze_set_ending_support(allViews, allThens, FPV_KAMIKAZE_VIEW_CUTSCENE,
                                          FPV_KAMIKAZE_THEN_NOTHING) == FPV_OK,
          "set support: %s", fpv_last_error());
    c = get();
    CHECK(c.ending_view == FPV_KAMIKAZE_VIEW_CUTSCENE && c.ending_then == FPV_KAMIKAZE_THEN_NOTHING,
          "no saved choice: the host's defaults (%u, %u)", c.ending_view, c.ending_then);
    CHECK(settingsText().find("kamikazeEndingThen") == std::string::npos, "nothing saved for a default");
    // signal lost and nothing are always possible; defaults must be supported
    CHECK(fpv_kamikaze_set_ending_support(0, 0, FPV_KAMIKAZE_VIEW_SIGNAL_LOST, FPV_KAMIKAZE_THEN_NOTHING) == FPV_OK,
          "empty masks still allow signal lost and nothing: %s", fpv_last_error());
    CHECK(fpv_kamikaze_set_ending_support(0, 0, FPV_KAMIKAZE_VIEW_CUTSCENE, FPV_KAMIKAZE_THEN_NOTHING) ==
              FPV_ERROR_INVALID_ARGUMENT,
          "unsupported default view refused");
    CHECK(fpv_kamikaze_set_ending_support(0, 0, FPV_KAMIKAZE_VIEW_SIGNAL_LOST, FPV_KAMIKAZE_THEN_RESPAWN) ==
              FPV_ERROR_INVALID_ARGUMENT,
          "unsupported default then refused");
    CHECK(fpv_kamikaze_set_ending_support(allViews, allThens, 5, 0) == FPV_ERROR_INVALID_ARGUMENT,
          "unknown default view refused");

    // back to the defaults for the other tests
    fpv_kamikaze_set_ending_support(allViews, allThens, FPV_KAMIKAZE_VIEW_SIGNAL_LOST, FPV_KAMIKAZE_THEN_RESPAWN);
    fpv_kamikaze_config_defaults(&cfg);
    fpv_kamikaze_set_config(&cfg);
    loadWorld(makeLevel(Host(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), 100, 0));
}
#endif

struct Test {
    const char *name;
    std::function<void()> fn;
};

}  // namespace

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // keep output if a test crashes
    std::string filter = argc > 1 ? argv[1] : "";
    namespace fs = std::filesystem;
    g_dataDir = (fs::temp_directory_path() / ("libfpv-test-" + std::to_string((long long)std::time(nullptr)))).string();
    std::string corpus = LIBFPV_TEST_DIFF_DIR;

    testVersionAndErrors();
    testSharedDataDir();
    if (!init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1))) return 1;

    std::vector<Test> tests = {
        {"arm_disarm", testArmDisarm},
        {"turtle", testTurtle},
        {"eeprom_upgrade", testEepromUpgrade},
        {"default_diff", testDefaultDiff},
        {"second_drone_rejected", testSecondDroneRejected},
        {"hover", testHover},
        {"keyboard_hover", testKeyboardHover},
        {"punch_out", testPunchOut},
        {"flight_realism", testFlightRealism},
        {"idle_attitude", testIdleAttitude},
        {"battery_endurance", testBatteryEndurance},
        {"wall_crash", testWallCrash},
        {"thirty_mps_thin_wall", testThirtyMetersPerSecond},
        {"motor_burnout", testMotorBurnout},
        {"presets", testPresets},
        {"preset_switch_battery", testPresetSwitchBattery},
        {"preset_switch_rest", testPresetSwitchRest},
        {"cli", testCli},
        {"diff_import", [&] { testDiffImport(corpus); }},
        {"axes_y_up_right", [] { testAxes(FPV_COORDS_Y_UP_RIGHT_HANDED, 1, "Y-up right-handed"); }},
        {"axes_y_up_left", [] { testAxes(FPV_COORDS_Y_UP_LEFT_HANDED, 1, "Y-up left-handed"); }},
        {"axes_z_up_left_cm", [] { testAxes(FPV_COORDS_Z_UP_LEFT_HANDED, 100, "Z-up left-handed (cm)"); }},
        {"axes_z_up_right_in", [] { testAxes(FPV_COORDS_Z_UP_RIGHT_HANDED, 39.37f, "Z-up right-handed (in)"); }},
        {"wind_drift", testWindDrift},
        {"wind_shelter", testWindShelter},
        {"wind_sources", testWindSources},
        {"gps", testGps},
        {"osd_audio_overlay", testOsdAudioOverlay},
        {"pinned", testPinned},
        {"terminate_reinit", testTerminateReinit},
        {"threaded", testThreadedMode},
#if defined(LIBFPV_KAMIKAZE)
        {"kamikaze_arming_block", testKamikazeArmingBlock},
        {"kamikaze_impact", testKamikazeImpact},
        {"kamikaze_explode_switch", testKamikazeExplodeSwitch},
        {"kamikaze_ground_tap", testKamikazeGroundTap},
        {"kamikaze_payload", testKamikazePayload},
        {"kamikaze_install", testKamikazeInstall},
        {"kamikaze_ending", testKamikazeEnding},
#endif
    };
    int ran = 0;
    for (auto &t : tests) {
        if (!filter.empty() && ("," + filter + ",").find("," + std::string(t.name) + ",") == std::string::npos) continue;
        if (!fpv_is_initialized() && std::string(t.name) != "threaded")
            init(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1));
        // every test starts from libfpv's default Betaflight configuration
        if (fpv_is_initialized()) {
            char *o = nullptr;
            fpv_cli_exec("defaults\nsave", &o);
            fpv_free(o);
            // and from a plain 200 m ground in the default convention
            loadWorld(makeLevel(Host(fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1)), 100, 0));
#if defined(LIBFPV_KAMIKAZE)
            // the other tests fly a bare quad and read Betaflight's own OSD
            // (the bomb is not installed by default)
            fpv_kamikaze_config kc;
            fpv_kamikaze_config_defaults(&kc);
            fpv_kamikaze_set_config(&kc);
            fpv_kamikaze_set_installed(std::string(t.name).rfind("kamikaze", 0) == 0);
#endif
        }
        int before = g_failures;
        auto t0 = std::chrono::steady_clock::now();
        std::printf("[ RUN  ] %s\n", t.name);
        std::fflush(stdout);
        t.fn();
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("[ %s ] %s (%.0f ms)\n", g_failures == before ? " OK " : "FAIL", t.name, ms);
        ran++;
    }
    fpv_global_terminate();
    std::error_code ec;
    fs::remove_all(g_dataDir, ec);
    std::printf("\n%d tests, %d checks, %d failures\n", ran, g_checks, g_failures);
    return g_failures ? 1 : 0;
}
