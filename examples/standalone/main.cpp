// examples/standalone - a minimal "game" that integrates libfpv exactly the
// way a mod would: SDL3 window, OpenGL 3.3 renderer, a test level (ground,
// gates, buildings, a thin wall, a moving platform), and every libfpv hook:
//
//   1. fpv_global_init                       once, after the game starts
//   2. fpv_static_surfaces_load              level collision
//      fpv_surface_object_create/move        the moving platform
//   3. fpv_drone_create                      spawn
//      fpv_gps_set_home                      the launch pad is GPS home
//      fpv_wind_set                          a breeze from the west (not in the
//                                            scripted flights)
//   4. every frame: fpv_drone_tick           pose + camera + OSD + impacts
//      place the game camera at frame.camera (or chase/LOS)
//      draw the drone at frame.position/rotation, props by frame.motors[].rpm
//      lens distortion + video noise post-process from frame.camera/video
//      fpv_osd_draw_data + fpv_overlay_render drawn by fpv_render_gl3.h
//   5. forward keyboard/mouse to fpv_input_key / fpv_overlay_*,
//      and stop game input while fpv_overlay_wants_input()
//   6. audio: fpv_audio_read from the audio callback
//   7. fpv_global_terminate on exit
//
// Keys: F8 overlay, V camera (FPV/chase/LOS), R respawn, F11 or Alt+Enter
// fullscreen, Esc quit.
// Verification flags: --autopilot (scripted flight), --screenshot out.ppm
// --frames N (save the N-th frame and exit), --overlay (open it), --camera.
//
// Web builds (Emscripten) run the same game in a browser page: WebGL 2,
// the page's audio, the browser's Gamepad API for radios (docs/WEB.md).
//
// Kamikaze builds (-DLIBFPV_KAMIKAZE=ON) add the warhead hooks:
//      fpv_kamikaze_poll_detonation          every frame: explosion, blast damage
//      fpv_drone_repair + fpv_drone_reset    the next drone, when the game decides
// and targets to hit (surface objects, so a direct hit names its object_id).
//      fpv_kamikaze_set_ending_support       the endings this game can do
// The bomb is not installed by default: F8 > Kamikaze > "Bomb installed" (or
// the game's own fpv_kamikaze_set_installed) adds it, for every drone.
// Keys: K bomb arm switch, X explode, R respawn (also during the ending).
// After the explosion the game shows the player's ending (F8 > Kamikaze):
// the view (signal lost or a cutscene) for its duration, then respawn, or
// nothing: the view stays until R (this game has no pilot to recall to). --view signal|cutscene,
// --duration S and --then respawn|nothing set it like the overlay would.
// --kamikaze-demo flies a scripted strike, then checks the ending options,
// and exits 0 (pass) or 1 (fail).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <SDL3/SDL.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#include <GLES3/gl3.h>
#define FPV_GL3_GLES  // WebGL 2
#else
#define GL_MIN_IMPLEMENTATION
#include "gl_min.h"
#endif
#define FPV_RENDER_GL3_IMPLEMENTATION
#include "fpv_render_gl3.h"
#include "libfpv.h"

// --- tiny math ---------------------------------------------------------------------

struct V3 {
    float x, y, z;
};
static V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static V3 norm(V3 a) {
    float l = std::sqrt(dot(a, a));
    return l > 0 ? a * (1 / l) : a;
}

struct M4 {
    float m[16];  // column major
};
static M4 mul(const M4 &a, const M4 &b) {
    M4 r{};
    for (int c = 0; c < 4; c++)
        for (int rr = 0; rr < 4; rr++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += a.m[k * 4 + rr] * b.m[c * 4 + k];
            r.m[c * 4 + rr] = s;
        }
    return r;
}
static M4 identity() {
    M4 r{};
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1;
    return r;
}
static M4 translate(V3 t) {
    M4 r = identity();
    r.m[12] = t.x, r.m[13] = t.y, r.m[14] = t.z;
    return r;
}
// rotation matrix from a quaternion {x, y, z, w}
static M4 fromQuat(const float q[4]) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    M4 r = identity();
    r.m[0] = 1 - 2 * (y * y + z * z), r.m[1] = 2 * (x * y + z * w), r.m[2] = 2 * (x * z - y * w);
    r.m[4] = 2 * (x * y - z * w), r.m[5] = 1 - 2 * (x * x + z * z), r.m[6] = 2 * (y * z + x * w);
    r.m[8] = 2 * (x * z + y * w), r.m[9] = 2 * (y * z - x * w), r.m[10] = 1 - 2 * (x * x + y * y);
    return r;
}
static M4 rotY(float a) {
    M4 r = identity();
    r.m[0] = std::cos(a), r.m[2] = -std::sin(a), r.m[8] = std::sin(a), r.m[10] = std::cos(a);
    return r;
}
static M4 perspective(float fovYDeg, float aspect, float n, float f) {
    float t = 1.0f / std::tan(fovYDeg * 3.14159265f / 360.0f);
    M4 r{};
    r.m[0] = t / aspect, r.m[5] = t, r.m[10] = (f + n) / (n - f), r.m[11] = -1, r.m[14] = 2 * f * n / (n - f);
    return r;
}
// view matrix from libfpv's camera basis (OpenGL style: camera looks down -Z)
static M4 viewFromCamera(const fpv_camera &c) {
    V3 r{c.right[0], c.right[1], c.right[2]}, u{c.up[0], c.up[1], c.up[2]}, f{c.forward[0], c.forward[1], c.forward[2]};
    V3 p{c.position[0], c.position[1], c.position[2]};
    M4 v = identity();
    v.m[0] = r.x, v.m[4] = r.y, v.m[8] = r.z;
    v.m[1] = u.x, v.m[5] = u.y, v.m[9] = u.z;
    v.m[2] = -f.x, v.m[6] = -f.y, v.m[10] = -f.z;
    v.m[12] = -dot(r, p), v.m[13] = -dot(u, p), v.m[14] = dot(f, p);
    return v;
}

// --- meshes ------------------------------------------------------------------------

struct Mesh {
    std::vector<float> v;  // pos3 normal3 color3
    std::vector<uint32_t> idx;
    GLuint vao = 0, vbo = 0, ebo = 0;

    void tri(V3 a, V3 b, V3 c, V3 col) {
        V3 n = norm(cross(b - a, c - a));
        uint32_t base = (uint32_t)(v.size() / 9);
        for (V3 p : {a, b, c}) {
            float d[9] = {p.x, p.y, p.z, n.x, n.y, n.z, col.x, col.y, col.z};
            v.insert(v.end(), d, d + 9);
        }
        idx.insert(idx.end(), {base, base + 1, base + 2});
    }
    void quad(V3 a, V3 b, V3 c, V3 d, V3 col) {
        tri(a, b, c, col);
        tri(a, c, d, col);
    }
    // box with centre, half extents and yaw
    void box(V3 c, V3 h, float yaw, V3 col) {
        float cs = std::cos(yaw), sn = std::sin(yaw);
        auto P = [&](float x, float y, float z) {
            float rx = x * cs + z * sn, rz = -x * sn + z * cs;
            return V3{c.x + rx, c.y + y, c.z + rz};
        };
        V3 p[8] = {P(-h.x, -h.y, -h.z), P(h.x, -h.y, -h.z), P(h.x, h.y, -h.z), P(-h.x, h.y, -h.z),
                   P(-h.x, -h.y, h.z),  P(h.x, -h.y, h.z),  P(h.x, h.y, h.z),  P(-h.x, h.y, h.z)};
        quad(p[4], p[5], p[6], p[7], col);  // +z
        quad(p[1], p[0], p[3], p[2], col);  // -z
        quad(p[5], p[1], p[2], p[6], col);  // +x
        quad(p[0], p[4], p[7], p[3], col);  // -x
        quad(p[7], p[6], p[2], p[3], col);  // +y
        quad(p[0], p[1], p[5], p[4], col);  // -y
    }
    // UV sphere (explosion fireball)
    void sphere(float r, int seg, V3 col) {
        auto P = [&](int i, int j) {
            float th = 3.14159265f * i / seg, ph = 6.2831853f * j / seg;
            return V3{r * std::sin(th) * std::cos(ph), r * std::cos(th), r * std::sin(th) * std::sin(ph)};
        };
        for (int i = 0; i < seg; i++)
            for (int j = 0; j < seg; j++) quad(P(i, j), P(i, j + 1), P(i + 1, j + 1), P(i + 1, j), col);
    }
    void upload() {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(v.size() * 4), v.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(idx.size() * 4), idx.data(), GL_STATIC_DRAW);
        for (int i = 0; i < 3; i++) {
            glEnableVertexAttribArray(i);
            glVertexAttribPointer(i, 3, GL_FLOAT, GL_FALSE, 36, (void *)(size_t)(i * 12));
        }
        glBindVertexArray(0);
    }
    void draw() const {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, (GLsizei)idx.size(), GL_UNSIGNED_INT, nullptr);
    }
    // positions only, for libfpv collision
    fpv_mesh collision() const {
        fpv_mesh m{};
        m.positions = v.data();
        m.vertex_count = (uint32_t)(v.size() / 9);
        m.vertex_stride = 36;
        m.indices = idx.data();
        m.triangle_count = (uint32_t)(idx.size() / 3);
        return m;
    }
};

static void buildLevel(Mesh &m) {
    // ground (also drawn with a checker in the shader)
    m.quad({-300, 0, 300}, {300, 0, 300}, {300, 0, -300}, {-300, 0, -300}, {0.35f, 0.5f, 0.3f});
    // a loop of race gates (1.8 m opening) ahead of the spawn; the loop
    // passes through the spawn, where no gate stands (the drone would take
    // off under its bottom bar)
    for (int i = 0; i < 6; i++) {
        if (i == 3) continue;
        float a = i * 6.2831853f / 6, r = 35;
        V3 c{std::sin(a) * r, 0, -std::cos(a) * r - 35};
        float yaw = -a;
        V3 col{1.0f, 0.45f, 0.1f};
        auto P = [&](float x, float y) { return V3{c.x + x * std::cos(yaw), y, c.z - x * std::sin(yaw)}; };
        m.box(P(-1.0f, 1.2f), {0.1f, 1.2f, 0.1f}, yaw, col);
        m.box(P(1.0f, 1.2f), {0.1f, 1.2f, 0.1f}, yaw, col);
        m.box(P(0, 2.3f), {1.1f, 0.1f, 0.1f}, yaw, col);
        m.box(P(0, 0.4f), {1.1f, 0.05f, 0.1f}, yaw, col);
    }
    // buildings
    struct B {
        float x, z, w, h, d;
    } const bs[] = {{-30, -10, 8, 14, 8}, {25, -5, 10, 9, 6}, {-15, -80, 12, 22, 12}, {40, -70, 6, 30, 6},
                    {0, 40, 20, 6, 4}};
    for (auto &b : bs) m.box({b.x, b.h / 2, b.z}, {b.w / 2, b.h / 2, b.d / 2}, 0.2f, {0.55f, 0.55f, 0.6f});
    // a thin (zero thickness) wall: collision must still hold at speed
    V3 wc{0.75f, 0.85f, 0.95f};
    m.quad({-6, 0, -15}, {6, 0, -15}, {6, 5, -15}, {-6, 5, -15}, wc);
    m.quad({6, 0, -15}, {-6, 0, -15}, {-6, 5, -15}, {6, 5, -15}, wc);
}

static void buildDrone(Mesh &body, Mesh &prop, const fpv_frame &f) {
    V3 dark{0.12f, 0.12f, 0.14f}, red{0.85f, 0.1f, 0.1f};
    body.box({0, 0.01f, 0}, {0.035f, 0.018f, 0.06f}, 0, dark);
    body.box({0, 0.035f, -0.045f}, {0.015f, 0.012f, 0.012f}, 0, red);  // camera, front (-Z)
    for (int i = 0; i < 4; i++) {
        V3 m{f.motors[i].position[0], f.motors[i].position[1], f.motors[i].position[2]};
        V3 mid = m * 0.5f;
        float yaw = std::atan2(m.x, m.z);
        float len = std::sqrt(m.x * m.x + m.z * m.z);
        body.box({mid.x, 0.0f, mid.z}, {0.008f, 0.004f, len / 2}, yaw, dark);
        body.box(m + V3{0, 0.006f, 0}, {0.013f, 0.012f, 0.013f}, 0, {0.6f, 0.6f, 0.65f});
    }
    // one propeller (3 blades) in its local frame
    for (int b = 0; b < 3; b++) {
        float a = b * 2.0943951f;
        V3 tip{std::sin(a) * 0.064f, 0.02f, std::cos(a) * 0.064f}, side{std::cos(a) * 0.009f, 0, -std::sin(a) * 0.009f};
        V3 hub{0, 0.02f, 0};
        prop.tri(hub, tip + side, tip - side, {0.2f, 0.7f, 1.0f});
        prop.tri(hub, tip - side, tip + side, {0.2f, 0.7f, 1.0f});
    }
}

// --- shaders -------------------------------------------------------------------------

#if defined(__EMSCRIPTEN__)
static const char *kGlslVersion = "#version 300 es\nprecision highp float;\n";
#else
static const char *kGlslVersion = "#version 330 core\n";
#endif

static GLuint program(const char *vs, const char *fs) {
    auto sh = [](GLenum t, const char *src) {
        GLuint s = glCreateShader(t);
        const char *srcs[2] = {kGlslVersion, src};
        glShaderSource(s, 2, srcs, nullptr);
        glCompileShader(s);
        GLint ok;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[2048];
            glGetShaderInfoLog(s, sizeof log, nullptr, log);
            std::fprintf(stderr, "shader: %s\n", log);
        }
        return s;
    };
    GLuint p = glCreateProgram();
    glAttachShader(p, sh(GL_VERTEX_SHADER, vs));
    glAttachShader(p, sh(GL_FRAGMENT_SHADER, fs));
    glLinkProgram(p);
    return p;
}

static const char *kSceneVs = R"(layout(location=0) in vec3 pos; layout(location=1) in vec3 nrm; layout(location=2) in vec3 col;
uniform mat4 viewProj; uniform mat4 model;
out vec3 v_nrm; out vec3 v_col; out vec3 v_world;
void main() {
  vec4 w = model * vec4(pos, 1.0);
  v_world = w.xyz; v_nrm = mat3(model) * nrm; v_col = col;
  gl_Position = viewProj * w;
})";
static const char *kSceneFs = R"(in vec3 v_nrm; in vec3 v_col; in vec3 v_world; uniform vec3 eye; uniform vec3 tint; uniform float emissive; out vec4 o;
void main() {
  vec3 n = normalize(v_nrm);
  float l = 0.35 + 0.65 * abs(dot(n, normalize(vec3(0.4, 1.0, 0.3))));
  vec3 c = mix(v_col * l, v_col, emissive) * tint;
  if (n.y > 0.99 && v_world.y < 0.01) { // ground checker, 2 m tiles
    float k = mod(floor(v_world.x * 0.5) + floor(v_world.z * 0.5), 2.0);
    c *= 0.85 + 0.15 * k;
  }
  float fog = clamp(length(v_world - eye) / 400.0, 0.0, 1.0);
  o = vec4(mix(c, vec3(0.62, 0.74, 0.86), fog * fog), 1.0);
})";
// Post-process: Brown-Conrady lens distortion and analog video noise, as in
// docs/INTEGRATION.md#lens-distortion.
static const char *kPostVs = R"(out vec2 uv;
void main() { vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2); uv = p; gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0); })";
static const char *kPostFs = R"(in vec2 uv; uniform sampler2D scene; uniform vec2 k; uniform float zoom; uniform float aspect;
uniform float noise; uniform float breakup; uniform float time; uniform float analog; uniform float flash;
uniform float lost; out vec4 o;
float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }
void main() {
  vec2 scale = vec2(aspect, 1.0) / (0.5 * sqrt(aspect * aspect + 1.0)); // r = 1 at the corners
  vec2 c = (uv - 0.5) * scale;
  float r2 = dot(c, c);
  vec2 d = c * (1.0 + k.x * r2 + k.y * r2 * r2) / zoom;
  vec2 suv = d / scale + 0.5;
  if (breakup > 0.0) { // rolling tear lines
    float line = step(1.0 - breakup * 0.25, hash(vec2(floor(uv.y * 120.0), floor(time * 20.0))));
    suv.x += line * (hash(vec2(time, uv.y)) - 0.5) * 0.15;
  }
  vec3 col = (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) ? vec3(0.0) : texture(scene, suv).rgb;
  float n = hash(uv * 513.0 + time) - 0.5;
  col += n * noise * 0.9 * analog;
  col = mix(col, vec3(dot(col, vec3(0.3, 0.59, 0.11))), noise * 0.6 * analog);
  col = mix(col, vec3(1.0, 0.85, 0.6), flash);
  col = mix(col, vec3(hash(floor(uv * vec2(320.0, 240.0)) + fract(time) * 91.0) * 0.85), lost); // no signal: static
  o = vec4(col, 1.0);
})";

// --- input mapping ----------------------------------------------------------------------

// Scancodes, not keycodes: libfpv's letter keys are physical positions (W/A/S/D
// throttle and yaw sit under the left hand on AZERTY too, labelled Z/Q/S/D).
static int toFpvKey(SDL_Scancode k) {
    if (k >= SDL_SCANCODE_A && k <= SDL_SCANCODE_Z) return 'A' + (int)(k - SDL_SCANCODE_A);
    if (k >= SDL_SCANCODE_1 && k <= SDL_SCANCODE_9) return '1' + (int)(k - SDL_SCANCODE_1);
    if (k >= SDL_SCANCODE_F1 && k <= SDL_SCANCODE_F12) return FPV_KEY_F1 + (int)(k - SDL_SCANCODE_F1);
    switch (k) {
    case SDL_SCANCODE_0: return '0';
    case SDL_SCANCODE_TAB: return FPV_KEY_TAB;
    case SDL_SCANCODE_RETURN: return FPV_KEY_ENTER;
    case SDL_SCANCODE_ESCAPE: return FPV_KEY_ESCAPE;
    case SDL_SCANCODE_SPACE: return FPV_KEY_SPACE;
    case SDL_SCANCODE_LEFT: return FPV_KEY_LEFT;
    case SDL_SCANCODE_RIGHT: return FPV_KEY_RIGHT;
    case SDL_SCANCODE_UP: return FPV_KEY_UP;
    case SDL_SCANCODE_DOWN: return FPV_KEY_DOWN;
    case SDL_SCANCODE_PAGEUP: return FPV_KEY_PAGE_UP;
    case SDL_SCANCODE_PAGEDOWN: return FPV_KEY_PAGE_DOWN;
    case SDL_SCANCODE_HOME: return FPV_KEY_HOME;
    case SDL_SCANCODE_END: return FPV_KEY_END;
    case SDL_SCANCODE_INSERT: return FPV_KEY_INSERT;
    case SDL_SCANCODE_DELETE: return FPV_KEY_DELETE;
    case SDL_SCANCODE_BACKSPACE: return FPV_KEY_BACKSPACE;
    case SDL_SCANCODE_LSHIFT: return FPV_KEY_LSHIFT;
    case SDL_SCANCODE_RSHIFT: return FPV_KEY_RSHIFT;
    case SDL_SCANCODE_LCTRL: return FPV_KEY_LCTRL;
    case SDL_SCANCODE_RCTRL: return FPV_KEY_RCTRL;
    case SDL_SCANCODE_LALT: return FPV_KEY_LALT;
    case SDL_SCANCODE_RALT: return FPV_KEY_RALT;
    default: return FPV_KEY_NONE;
    }
}

static void logCb(void *, fpv_log_level level, const char *msg) {
    if (level >= FPV_LOG_INFO) SDL_Log("[libfpv] %s", msg);
}

static void SDLCALL audioCb(void *, SDL_AudioStream *stream, int additional, int) {
    static std::vector<float> buf;
    int frames = additional / (int)(2 * sizeof(float));
    if (frames <= 0) return;
    buf.resize((size_t)frames * 2);
    fpv_audio_read(buf.data(), (uint32_t)frames);
    SDL_PutAudioStreamData(stream, buf.data(), frames * 2 * (int)sizeof(float));
}

#if !defined(__EMSCRIPTEN__)
static void savePpm(const char *path, int w, int h) {
    std::vector<unsigned char> px((size_t)w * h * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    FILE *f = std::fopen(path, "wb");
    if (!f) return;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; y--) std::fwrite(&px[(size_t)y * w * 3], 1, (size_t)w * 3, f);
    std::fclose(f);
}
#endif

#if defined(LIBFPV_KAMIKAZE)
// --- kamikaze -------------------------------------------------------------------------

// The ending in progress after a detonation, timed with the game's clock.
struct Ending {
    bool showing = false;    // the view is on screen
    double until = -1;       // when the view ends, < 0 = until the player acts
    uint32_t view = FPV_KAMIKAZE_VIEW_SIGNAL_LOST, then = FPV_KAMIKAZE_THEN_NOTHING;
    double blastTime = -1;
    int respawns = 0;        // for the demo's checks
    int cutsceneFrames = 0;  // frames drawn from the cutscene camera
};

struct Target {
    V3 centre, half;
    uint32_t objectId = 0;
    bool destroyed = false;
};

// A camera looking from eye at target (a cutscene camera of the game).
static fpv_camera lookCamera(V3 eye, V3 target, float fov) {
    fpv_camera c{};
    V3 f = norm(target - eye), r = norm(cross(f, {0, 1, 0})), u = cross(r, f);
    float *dst[] = {c.position, c.forward, c.up, c.right};
    V3 src[] = {eye, f, u, r};
    for (int i = 0; i < 4; i++) dst[i][0] = src[i].x, dst[i][1] = src[i].y, dst[i][2] = src[i].z;
    c.fov_y_degrees = fov;
    c.near_plane = 0.1f;
    c.distortion_zoom = 1;
    return c;
}

// The demo's target, straight ahead of the spawn (the first of `targets`).
static const V3 kDemoTarget{0, 1.0f, -11};

// Scripted strike for --kamikaze-demo. Checks each warhead rule against
// what libfpv reports and returns the channels to send. Then checks the
// ending options on host detonations: no view, a view that stays, an early
// respawn that cancels the timer.
struct KamikazeDemo {
    int failures = 0;
    int stage = 0;
    double stageTime = 0, respawnSeen = -1;
    bool detonated = false, respawned = false, targetHit = false, directHit = false;
    bool wantRespawn = false;    // the "player" presses R
    bool wantDetonate = false;   // the game blows the warhead up
    float armingDelay = 0;       // the player's setting (F8 > Kamikaze), none by default
    fpv_kamikaze_config ending{};  // the strike's ending (the player's choice)
    const Ending *game = nullptr;
    // the ending scenarios after the strike
    struct Scenario {
        const char *name;
        uint32_t view;
        float duration;
        uint32_t then;
        double earlyRespawn;  // the player presses R this long after the blast, < 0 = never
    };
    int scenario = 0, phase = 0, respawnsBefore = 0;
    double phaseTime = 0;
    void check(bool ok, const char *what) {
        SDL_Log("[demo] %s %s", ok ? "PASS" : "FAIL", what);
        if (!ok) failures++;
    }
    // returns false when finished
    bool update(double t, const fpv_frame &f, const fpv_kamikaze_state &k, float ch[FPV_CHANNEL_COUNT]) {
        for (int i = 0; i < FPV_CHANNEL_COUNT; i++) ch[i] = (i == 2 || i >= 4) ? -1.0f : 0.0f;
        ch[5] = 1;  // angle mode
        bool armed = f.flags & FPV_DRONE_ARMED;
        bool dead = f.flags & FPV_DRONE_DETONATED;
        double st = t - stageTime;
        // the scripted pilot's throttle: hold a height (PD, any payload)
        auto holdHeight = [&](float h) {
            float thr = -0.35f + 0.2f * (h - f.position[1]) - 0.15f * f.velocity[1];
            return std::fmax(-0.9f, std::fmin(0.4f, thr));
        };
        auto next = [&] {
            SDL_Log("[demo] stage %d done at %.1f s, %.1f m, %.0f km/h", stage, t, f.position[1], f.speed_kmh);
            stage++, stageTime = t;
        };
        switch (stage) {
        case 0:  // bomb switch on, then try to arm the motors
            ch[10] = 1;
            ch[4] = st > 0.3 ? 1 : -1;
            if (st > 1.0) {
                check(!armed && (k.flags & FPV_KAMIKAZE_BLOCKS_ARMING), "motors refuse to arm with the bomb switch on");
                next();
            }
            break;
        case 1:  // bomb safe, arm switch cycled
            ch[4] = st > 0.5 ? 1 : -1;
            if (st > 1.0) {
                check(armed, "motors arm with the bomb switch off");
                check(k.warhead == FPV_WARHEAD_SAFE, "warhead SAFE");
                next();
            }
            break;
        case 2:  // take off, explode switch on a safe bomb
            ch[4] = 1;
            ch[2] = holdHeight(5);
            ch[11] = st > 0.3 && st < 0.6 ? 1 : -1;
            if (st > 1.0) {
                check(k.warhead == FPV_WARHEAD_SAFE && !dead, "explode switch does nothing while SAFE");
                next();
            }
            break;
        case 3:  // bomb arm switch in flight: countdown
            ch[4] = 1;
            ch[2] = holdHeight(5);  // hold height while it arms
            ch[10] = 1;
            if (armingDelay > 1 && st > 0.5 && st < 0.52)
                check(k.warhead == FPV_WARHEAD_ARMING, "ARMING countdown after the switch");
            if (k.warhead == FPV_WARHEAD_ARMED) {
                check(st > armingDelay - 0.5, "live (STEADY) after the arming delay, if any");
                next();
            }
            if (st > armingDelay + 3) check(false, "warhead went live"), next();
            break;
        case 4: {  // fly at the target (straight ahead) on a glide slope
            ch[4] = 1;
            ch[10] = 1;
            float dist = std::fabs(f.position[2] - kDemoTarget.z);
            ch[1] = 0.35f;
            ch[2] = holdHeight(kDemoTarget.y * 0.5f + 0.1f * dist);  // aim low at the truck's side
            ch[0] = std::fmax(-0.3f, std::fmin(0.3f, -0.1f * f.position[0]));  // stay on the line
            if (dead) {
                check(detonated, "detonation event received (fpv_kamikaze_poll_detonation)");
                check(targetHit && directHit, "direct hit: the target ahead is destroyed (its object_id)");
                check(f.video.signal_strength == 0, "video signal lost after the explosion");
                next();
            }
            if (st > 15) check(false, "hit something within 15 s"), next();
            break;
        }
        case 5: {  // the strike's ending, as the player set it
            double d = ending.ending_duration;
            bool respawnAfter = ending.ending_then == FPV_KAMIKAZE_THEN_RESPAWN;
            if (respawned && respawnSeen < 0) {
                respawnSeen = t;
                if (respawnAfter)
                    check(std::fabs(t - game->blastTime - d) < 0.25,
                          "the game respawns the drone when the ending's time is up");
            }
            if (!respawnAfter && !respawned && st > d + 1) {
                check(dead && game->showing, "\"nothing\": the view stays until the player acts");
                wantRespawn = true;
            }
            if ((respawnSeen >= 0 && t - respawnSeen > 0.3) || st > d + 6) {
                check(respawned && k.warhead == FPV_WARHEAD_SAFE && !dead && !game->showing,
                      "respawned with a new, safe warhead");
                next();
            }
            break;
        }
        case 6: {  // the ending options on host detonations, on the ground
            static const Scenario scenarios[] = {
                {"0 s view, then respawn", FPV_KAMIKAZE_VIEW_SIGNAL_LOST, 0, FPV_KAMIKAZE_THEN_RESPAWN, -1},
                {"1 s cutscene, then nothing", FPV_KAMIKAZE_VIEW_CUTSCENE, 1, FPV_KAMIKAZE_THEN_NOTHING, 2.0},
                {"3 s cutscene, early respawn", FPV_KAMIKAZE_VIEW_CUTSCENE, 3, FPV_KAMIKAZE_THEN_RESPAWN, 0.5},
                {"0 s cutscene, then nothing", FPV_KAMIKAZE_VIEW_CUTSCENE, 0, FPV_KAMIKAZE_THEN_NOTHING, 1.0},
            };
            if (scenario >= (int)(sizeof scenarios / sizeof scenarios[0])) return false;
            const Scenario &sc = scenarios[scenario];
            double pt = t - phaseTime;
            char what[160];
            auto say = [&](const char *msg) {
                std::snprintf(what, sizeof what, "%s: %s", sc.name, msg);
                return what;
            };
            if (phase == 0 && pt > 0.5) {  // settled: the player's choice, then the blast
                fpv_kamikaze_config c{};
                c.struct_size = sizeof c;
                fpv_kamikaze_get_config(&c);
                c.ending_view = sc.view;
                c.ending_duration = sc.duration;
                c.ending_then = sc.then;
                fpv_kamikaze_set_config(&c);
                respawnsBefore = game->respawns;
                wantDetonate = true;
                phase = 1, phaseTime = t;
            } else if (phase == 1 && pt > 0.1) {  // the blast went off a moment ago
                bool respawnNow = sc.duration <= 0 && sc.then == FPV_KAMIKAZE_THEN_RESPAWN;
                if (respawnNow) {
                    check(game->respawns == respawnsBefore + 1 && !dead && game->cutsceneFrames == 0,
                          say("respawned at once, no view"));
                    phase = 3, phaseTime = t;
                } else {
                    // "nothing" shows the view even for 0 s, and holds it
                    check(dead && game->showing, say("the view shows"));
                    if (sc.view == FPV_KAMIKAZE_VIEW_CUTSCENE)
                        check(game->cutsceneFrames > 0, say("drawn from the cutscene camera"));
                    phase = 2, phaseTime = t - pt;
                }
            } else if (phase == 2 && sc.earlyRespawn >= 0 && pt >= sc.earlyRespawn) {
                // the player respawns during the view (or with nothing on screen)
                if (sc.then == FPV_KAMIKAZE_THEN_NOTHING)
                    check(dead && game->showing && game->respawns == respawnsBefore,
                          say("the view stays past its duration until the player acts"));
                wantRespawn = true;
                phase = 3, phaseTime = t;
            } else if (phase == 3 && pt > 0.2 && pt < 0.3) {
                check(!dead && !game->showing && game->respawns == respawnsBefore + 1, say("respawned, the view is gone"));
                phase = 4;
            } else if (phase == 4 && pt > std::fmax(0.5, sc.duration + 0.5)) {
                // the cancelled timer must not respawn it again
                check(game->respawns == respawnsBefore + 1, say("no second respawn from the cancelled timer"));
                scenario++, phase = 0, phaseTime = t;
            }
            break;
        }
        }
        return true;
    }
};
#endif

// --- main -------------------------------------------------------------------------------

int main(int argc, char **argv) {
#if defined(__EMSCRIPTEN__)
    std::string dataDir = FPV_WEB_DATA_DIR, screenshot;  // kept by the browser
#else
    // the folder every libfpv game shares (--data picks another)
    char shared[1024];
    std::string dataDir = fpv_shared_data_dir(shared, sizeof shared) == FPV_OK ? shared : "data", screenshot;
#endif
    int frames = 0, width = 1280, height = 720, camMode = 0;
    bool autopilot = false, openOverlay = false;
#if defined(LIBFPV_KAMIKAZE)
    bool kamikazeDemo = false;
    // the ending chosen on the command line (-1 = the player's, F8 > Kamikaze)
    int argView = -1, argThen = -1;
    float argDuration = -1;
#endif
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto nextArg = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--data") dataDir = nextArg();
        else if (a == "--screenshot") screenshot = nextArg();
        else if (a == "--frames") frames = std::atoi(nextArg().c_str());
        else if (a == "--autopilot") autopilot = true;
        else if (a == "--overlay") openOverlay = true;
        else if (a == "--width") width = std::atoi(nextArg().c_str());
        else if (a == "--height") height = std::atoi(nextArg().c_str());
#if defined(LIBFPV_KAMIKAZE)
        else if (a == "--kamikaze-demo") kamikazeDemo = true;
        else if (a == "--view") argView = nextArg() == "cutscene" ? FPV_KAMIKAZE_VIEW_CUTSCENE : FPV_KAMIKAZE_VIEW_SIGNAL_LOST;
        else if (a == "--then") argThen = nextArg() == "respawn" ? FPV_KAMIKAZE_THEN_RESPAWN : FPV_KAMIKAZE_THEN_NOTHING;
        else if (a == "--duration") argDuration = (float)std::atof(nextArg().c_str());
#endif
        else if (a == "--camera") {
            std::string c = nextArg();
            camMode = c == "chase" ? 1 : c == "los" ? 2 : 0;
        }
    }

    // The game's own SDL (video + audio). libfpv uses a separate, hidden SDL
    // instance for radios; the game does not initialise joysticks for it.
    // Text input is on for the overlay's text fields; without this, holding
    // W/S on macOS opens the accent picker instead of repeating the key.
    SDL_SetHint(SDL_HINT_MAC_PRESS_AND_HOLD, "0");
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO)) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
#if defined(__EMSCRIPTEN__)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_WindowFlags winFlags = SDL_WINDOW_FILL_DOCUMENT;  // the canvas fills the page
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_WindowFlags winFlags = SDL_WINDOW_RESIZABLE;
#endif
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_Window *win = SDL_CreateWindow("libfpv standalone", width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_HIGH_PIXEL_DENSITY | winFlags);
    if (!win) {
        std::fprintf(stderr, "window: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GLContext gl = SDL_GL_CreateContext(win);
#if defined(__EMSCRIPTEN__)
    if (!gl) {
        std::fprintf(stderr, "WebGL 2: %s\n", SDL_GetError());
        return 1;
    }
#else
    if (!gl || !gl_min_load((void *(*)(const char *))SDL_GL_GetProcAddress)) {
        std::fprintf(stderr, "OpenGL 3.3: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetSwapInterval(screenshot.empty() ? 1 : 0);
#endif
    SDL_StartTextInput(win);

    // 1. libfpv
    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    cfg.data_dir = dataDir.c_str();
    cfg.coords = fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, 1.0f);  // OpenGL: X right, Y up, -Z forward
    cfg.log = logCb;
#if defined(FPV_STANDALONE_BF_CONFIG)
    // the build's Betaflight config: every new drone configuration (and a
    // reset one) starts from it; existing ones and later edits are kept
    if (FILE *f = std::fopen(FPV_STANDALONE_BF_CONFIG, "rb")) {
        std::string diff;
        char buf[4096];
        for (size_t n; (n = std::fread(buf, 1, sizeof buf, f)) > 0;) diff.append(buf, n);
        std::fclose(f);
        fpv_set_default_betaflight_diff(diff.c_str());
    } else {
        std::fprintf(stderr, "Betaflight config: cannot read %s\n", FPV_STANDALONE_BF_CONFIG);
    }
#endif
    if (fpv_global_init(&cfg) != FPV_OK) {
        std::fprintf(stderr, "libfpv: %s\n", fpv_last_error());
        return 1;
    }

    // 2. world
    Mesh level;
    buildLevel(level);
    level.upload();
    fpv_mesh lm = level.collision();
    fpv_static_surfaces_load(&lm, 1);
    Mesh platform;
    platform.box({0, 0, 0}, {2.0f, 0.15f, 2.0f}, 0, {0.2f, 0.45f, 0.9f});
    platform.upload();
    fpv_transform pt{};
    pt.rotation[3] = 1;
    pt.position[0] = 12, pt.position[1] = 2, pt.position[2] = -25;
    uint32_t platformId = 0;
    fpv_mesh pm = platform.collision();
    fpv_surface_object_create(&pm, &pt, &platformId);
#if defined(LIBFPV_KAMIKAZE)
    // targets: surface objects, so a direct hit reports their object_id
    Target targets[] = {{kDemoTarget, {1.2f, 1.0f, 2.2f}}, {{-12, 1.0f, -45}, {1.3f, 1.0f, 2.5f}},
                        {{14, 1.5f, -40}, {2.0f, 1.5f, 2.0f}}};
    Mesh targetMesh, fireball;
    targetMesh.box({0, 0, 0}, {1, 1, 1}, 0, {0.3f, 0.38f, 0.22f});  // unit box, scaled per target
    targetMesh.box({0, 1.1f, 0.3f}, {0.5f, 0.12f, 0.4f}, 0, {0.25f, 0.3f, 0.18f});
    targetMesh.upload();
    fireball.sphere(1.0f, 16, {1.0f, 1.0f, 1.0f});  // coloured by the tint
    fireball.upload();
    for (Target &t : targets) {
        fpv_transform tt{};
        tt.position[0] = t.centre.x, tt.position[1] = t.centre.y, tt.position[2] = t.centre.z;
        tt.rotation[3] = 1;
        tt.scale[0] = t.half.x, tt.scale[1] = t.half.y, tt.scale[2] = t.half.z;
        fpv_mesh tm = targetMesh.collision();
        fpv_surface_object_create(&tm, &tt, &t.objectId);
    }
    struct Blast {
        V3 pos;
        double start = -1;
    } blast;
    // This game can show both views; after them it respawns the drone or
    // does nothing (there is no pilot to recall to), nothing by default.
    fpv_kamikaze_set_ending_support((1u << FPV_KAMIKAZE_VIEW_SIGNAL_LOST) | (1u << FPV_KAMIKAZE_VIEW_CUTSCENE),
                                    (1u << FPV_KAMIKAZE_THEN_RESPAWN) | (1u << FPV_KAMIKAZE_THEN_NOTHING),
                                    FPV_KAMIKAZE_VIEW_SIGNAL_LOST, FPV_KAMIKAZE_THEN_NOTHING);
    fpv_kamikaze_config playerConfig{};  // restored after the demo
    playerConfig.struct_size = sizeof playerConfig;
    fpv_kamikaze_get_config(&playerConfig);
    if (argView >= 0 || argThen >= 0 || argDuration >= 0) {
        fpv_kamikaze_config kc = playerConfig;
        if (argView >= 0) kc.ending_view = (uint32_t)argView;
        if (argThen >= 0) kc.ending_then = (uint32_t)argThen;
        if (argDuration >= 0) kc.ending_duration = std::fmin(argDuration, 30.0f);
        if (fpv_kamikaze_set_config(&kc) != FPV_OK) std::fprintf(stderr, "ending: %s\n", fpv_last_error());
    }
    Ending ending;
    KamikazeDemo demo;
    demo.game = &ending;
    if (kamikazeDemo) {
        fpv_kamikaze_set_installed(1);  // the player would tick it in F8 > Kamikaze
        demo.ending.struct_size = sizeof demo.ending;
        fpv_kamikaze_get_config(&demo.ending);
        demo.armingDelay = demo.ending.arming_delay;
    }
#endif

    // 3. drone
    fpv_drone_desc dd;
    fpv_drone_desc_defaults(&dd);
    dd.position[0] = 0, dd.position[1] = 0, dd.position[2] = 0;
    fpv_drone *drone = nullptr;
    if (fpv_drone_create(&dd, &drone) != FPV_OK) {
        std::fprintf(stderr, "drone: %s\n", fpv_last_error());
        return 1;
    }
    // the home arrow and distance point at the launch pad, wherever the drone
    // is armed (without this, home is where it last armed)
    fpv_gps_set_home(dd.position);
    // a 4 m/s breeze from the west (towards +X, east): the buildings shelter
    // the air behind them. The scripted flights keep still air.
    bool scripted = autopilot;
#if defined(LIBFPV_KAMIKAZE)
    scripted |= kamikazeDemo;
#endif
    if (!scripted) {
        fpv_wind wind;
        fpv_wind_defaults(&wind);
        wind.velocity[0] = 4.0f;
        fpv_wind_set(&wind);
    }
#if defined(LIBFPV_KAMIKAZE)
    // the next drone: a new, safe warhead at the spawn; ends any ending
    auto respawn = [&] {
        fpv_drone_repair(drone);
        fpv_drone_reset(drone, dd.position, dd.rotation);
        ending.showing = false;
        ending.respawns++;
        demo.respawned = true;
    };
#endif
    fpv_frame frame;
    std::memset(&frame, 0, sizeof frame);
    frame.struct_size = sizeof frame;
    fpv_drone_tick(drone, 0, &frame);
    Mesh body, prop;
    buildDrone(body, prop, frame);
    body.upload();
    prop.upload();

    // renderers
    GLuint sceneProg = program(kSceneVs, kSceneFs), postProg = program(kPostVs, kPostFs);
    GLuint emptyVao;
    glGenVertexArrays(1, &emptyVao);
    fpv_gl3 ui;
    if (!fpv_gl3_init(&ui)) std::fprintf(stderr, "overlay renderer failed\n");
    GLuint fbo = 0, fboTex = 0, fboDepth = 0;
    int fboW = 0, fboH = 0;

    // 6. audio
    SDL_AudioSpec spec{SDL_AUDIO_F32, 2, 48000};
    SDL_AudioStream *audio = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, audioCb, nullptr);
    if (audio) {
        fpv_audio_configure(48000, 2);
        SDL_ResumeAudioStreamDevice(audio);
    }
    if (openOverlay) fpv_overlay_set_open(1);

    double propAngle[4] = {0, 0, 0, 0};
    Uint64 last = SDL_GetPerformanceCounter();
    double time = 0;
    bool running = true;
    int frameNo = 0;
    // one frame of the game (a browser calls it once per display refresh)
    auto gameFrame = [&] {
        // 5. input
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_EVENT_QUIT: running = false; break;
            case SDL_EVENT_KEY_DOWN:
            case SDL_EVENT_KEY_UP: {
                int k = toFpvKey(e.key.scancode);
                bool down = e.type == SDL_EVENT_KEY_DOWN;
                // fullscreen toggle; Alt+Enter must not reach libfpv (Enter arms)
                bool altEnter = (e.key.key == SDLK_RETURN || e.key.key == SDLK_KP_ENTER) && (e.key.mod & SDL_KMOD_ALT);
                if (altEnter || e.key.key == SDLK_F11) {
                    if (down && !e.key.repeat)
                        SDL_SetWindowFullscreen(win, !(SDL_GetWindowFlags(win) & SDL_WINDOW_FULLSCREEN));
                    break;
                }
                if (k != FPV_KEY_NONE && !e.key.repeat) fpv_input_key(k, down);
                if (down && !fpv_overlay_wants_input()) {  // the game's own keys
#if !defined(__EMSCRIPTEN__)
                    if (e.key.key == SDLK_ESCAPE) running = false;  // a web page has nothing to quit
#endif
                    if (e.key.key == SDLK_V) camMode = (camMode + 1) % 3;
                    if (e.key.key == SDLK_R) {
#if defined(LIBFPV_KAMIKAZE)
                        respawn();  // also during the ending view: it cancels the timer
#else
                        fpv_drone_repair(drone);
                        fpv_drone_reset(drone, dd.position, dd.rotation);
#endif
                    }
                }
                break;
            }
            case SDL_EVENT_TEXT_INPUT: fpv_overlay_text(e.text.text); break;
            case SDL_EVENT_MOUSE_MOTION: fpv_overlay_mouse_move(e.motion.x, e.motion.y); break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
            case SDL_EVENT_MOUSE_BUTTON_UP: {
                int b = e.button.button == SDL_BUTTON_LEFT ? 0 : e.button.button == SDL_BUTTON_RIGHT ? 1 : 2;
                fpv_overlay_mouse_button(b, e.type == SDL_EVENT_MOUSE_BUTTON_DOWN);
                break;
            }
            case SDL_EVENT_MOUSE_WHEEL: fpv_overlay_mouse_wheel(e.wheel.x, e.wheel.y); break;
            default: break;
            }
        }
        Uint64 now = SDL_GetPerformanceCounter();
        float dt = (float)((double)(now - last) / (double)SDL_GetPerformanceFrequency());
        last = now;
        if (!screenshot.empty()) dt = 1.0f / 60.0f;
        time += dt;

        if (autopilot) {
            // scripted pilot: arm, climb, fly forward through the gate loop
            float ch[8] = {0, 0, -1, 0, -1, 1, -1, -1};
            if (time > 0.5) ch[4] = 1;
            if (time > 1.0) ch[2] = 0.1f;
            if (time > 2.0) {
                ch[2] = -0.25f;
                ch[1] = 0.35f;
                ch[3] = 0.12f;
            }
            fpv_input_set_channels(ch, 8);
            if (!screenshot.empty()) SDL_Delay(16);  // real-time pacing for the sim thread
        }

#if defined(LIBFPV_KAMIKAZE)
        fpv_kamikaze_state ks{};
        ks.struct_size = sizeof ks;
        fpv_kamikaze_get_state(drone, &ks);
        if (kamikazeDemo) {
            float ch[FPV_CHANNEL_COUNT];
            if (!demo.update(time, frame, ks, ch)) running = false;
            fpv_input_set_channels(ch, FPV_CHANNEL_COUNT);
            SDL_Delay(screenshot.empty() ? 0 : 16);
        }
#endif

        // moving platform: up and down 2..6 m
        pt.position[1] = 4 + 2 * (float)std::sin(time * 0.6);
        fpv_surface_object_move(platformId, &pt);

        // 4. tick
        fpv_drone_tick(drone, dt, &frame);
        for (int i = 0; i < 4; i++)
            propAngle[i] += frame.motors[i].rpm / 60.0 * 6.2831853 * dt * frame.motors[i].spin_direction;
        for (uint32_t i = 0; i < frame.impact_count; i++)
            if (frame.impacts[i].speed > 4) SDL_Log("impact %.1f m/s", frame.impacts[i].speed);

#if defined(LIBFPV_KAMIKAZE)
        if (demo.wantDetonate) {
            demo.wantDetonate = false;
            fpv_kamikaze_detonate(drone, 1);  // as if shot: goes off even while safe
        }
        // the warhead went off: explosion, blast damage, then the player's
        // ending: the view for its duration, then respawn or nothing
        fpv_detonation det{};
        det.struct_size = sizeof det;
        while (fpv_kamikaze_poll_detonation(&det)) {
            static const char *causes[] = {"impact", "explode switch", "game"};
            V3 at{det.position[0], det.position[1], det.position[2]};
            SDL_Log("BOOM (%s) at %.1f %.1f %.1f, %.0f km/h", causes[det.cause], at.x, at.y, at.z,
                    std::sqrt(dot(V3{det.velocity[0], det.velocity[1], det.velocity[2]},
                                  V3{det.velocity[0], det.velocity[1], det.velocity[2]})) * 3.6f);
            blast.pos = at;
            blast.start = time;
            demo.detonated = true;
            for (Target &t : targets) {
                V3 d = at - t.centre;
                // distance from the blast to the target's box
                V3 out{std::fmax(std::fabs(d.x) - t.half.x, 0.0f), std::fmax(std::fabs(d.y) - t.half.y, 0.0f),
                       std::fmax(std::fabs(d.z) - t.half.z, 0.0f)};
                bool direct = det.object_id == t.objectId;
                if (!t.destroyed && (direct || dot(out, out) < 4.0f * 4.0f)) {
                    t.destroyed = true;
                    demo.targetHit = true;
                    demo.directHit |= direct;
                    SDL_Log("target destroyed (%s)", direct ? "direct hit" : "blast");
                }
            }
            fpv_kamikaze_config kc{};
            kc.struct_size = sizeof kc;
            fpv_kamikaze_get_config(&kc);  // the player's ending (F8 > Kamikaze)
            bool respawnAfter = kc.ending_then == FPV_KAMIKAZE_THEN_RESPAWN;
            ending.blastTime = time;
            ending.view = kc.ending_view;
            ending.then = kc.ending_then;
            ending.cutsceneFrames = 0;
            // "nothing" holds the view until the player acts (whatever the
            // duration); otherwise 0 s means no view at all
            ending.showing = kc.ending_duration > 0 || !respawnAfter;
            ending.until = ending.showing && respawnAfter ? time + kc.ending_duration : -1;
            if (!ending.showing && respawnAfter) respawn();
            else if (!respawnAfter) SDL_Log("SIGNAL LOST - press R for the next drone");
        }
        if (ending.showing && ending.until >= 0 && time >= ending.until) {
            ending.showing = false;
            if (ending.then == FPV_KAMIKAZE_THEN_RESPAWN) respawn();
        }
        if (demo.wantRespawn) {  // the player's R
            demo.wantRespawn = false;
            respawn();
        }
#endif

        int fbw, fbh;
        SDL_GetWindowSizeInPixels(win, &fbw, &fbh);
        int ww, wh;
        SDL_GetWindowSize(win, &ww, &wh);
        if (fbw != fboW || fbh != fboH) {
            if (fbo) {
                glDeleteFramebuffers(1, &fbo);
                glDeleteTextures(1, &fboTex);
                glDeleteRenderbuffers(1, &fboDepth);
            }
            glGenFramebuffers(1, &fbo);
            glBindFramebuffer(GL_FRAMEBUFFER, fbo);
            glGenTextures(1, &fboTex);
            glBindTexture(GL_TEXTURE_2D, fboTex);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, fbw, fbh, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTex, 0);
            glGenRenderbuffers(1, &fboDepth);
            glBindRenderbuffer(GL_RENDERBUFFER, fboDepth);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, fbw, fbh);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, fboDepth);
            fboW = fbw, fboH = fbh;
        }

        // scene into the offscreen target
        const fpv_camera *camPtr = camMode == 0 ? &frame.camera : camMode == 1 ? &frame.chase_camera : &frame.los_camera;
        bool cutscene = false;
#if defined(LIBFPV_KAMIKAZE)
        fpv_camera orbit;
        if (ending.showing && ending.view == FPV_KAMIKAZE_VIEW_CUTSCENE) {
            // explosion cutscene: orbit the blast until the view ends
            float a = (float)(time - blast.start) * 0.5f;
            orbit = lookCamera(blast.pos + V3{std::sin(a) * 14, 6, std::cos(a) * 14}, blast.pos, 60);
            camPtr = &orbit;
            cutscene = true;
            ending.cutsceneFrames++;
        } else if (ending.showing) {
            camPtr = &frame.camera;  // signal lost: the dead FPV feed, whatever the camera mode
        }
#endif
        const fpv_camera &cam = *camPtr;
        float aspect = (float)fbw / (float)std::max(1, fbh);
        M4 viewProj = mul(perspective(cam.fov_y_degrees, aspect, std::max(0.01f, cam.near_plane), 2000), viewFromCamera(cam));
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, fbw, fbh);
        glClearColor(0.62f, 0.74f, 0.86f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glUseProgram(sceneProg);
        glUniformMatrix4fv(glGetUniformLocation(sceneProg, "viewProj"), 1, GL_FALSE, viewProj.m);
        glUniform3f(glGetUniformLocation(sceneProg, "eye"), cam.position[0], cam.position[1], cam.position[2]);
        GLint locModel = glGetUniformLocation(sceneProg, "model");
        GLint locTint = glGetUniformLocation(sceneProg, "tint"), locEmissive = glGetUniformLocation(sceneProg, "emissive");
        glUniform3f(locTint, 1, 1, 1);
        glUniform1f(locEmissive, 0);
        M4 I = identity();
        glUniformMatrix4fv(locModel, 1, GL_FALSE, I.m);
        level.draw();
        M4 pmod = translate({pt.position[0], pt.position[1], pt.position[2]});
        glUniformMatrix4fv(locModel, 1, GL_FALSE, pmod.m);
        platform.draw();
        bool drawDrone = camMode != 0 || cutscene;
#if defined(LIBFPV_KAMIKAZE)
        for (const Target &t : targets) {
            M4 sc = identity();
            sc.m[0] = t.half.x, sc.m[5] = t.half.y, sc.m[10] = t.half.z;
            M4 tm = mul(translate(t.centre), sc);
            glUniformMatrix4fv(locModel, 1, GL_FALSE, tm.m);
            if (t.destroyed) glUniform3f(locTint, 0.15f, 0.13f, 0.12f);  // burnt
            targetMesh.draw();
            glUniform3f(locTint, 1, 1, 1);
        }
        if (blast.start >= 0 && time - blast.start < 2.5) {
            // fireball: grows fast, then cools to smoke
            float age = (float)(time - blast.start);
            float r = 4.5f * (1 - std::exp(-age * 8));
            float heat = std::fmax(0.0f, 1 - age / 0.9f);
            M4 sc = identity();
            sc.m[0] = sc.m[5] = sc.m[10] = r;
            M4 fm = mul(translate(blast.pos + V3{0, age * 1.5f, 0}), sc);
            glUniformMatrix4fv(locModel, 1, GL_FALSE, fm.m);
            glUniform1f(locEmissive, heat);
            // white-yellow fire cooling to dark smoke
            V3 fire{1.0f, 0.75f - 0.25f * (1 - heat), 0.3f * heat}, smoke{0.22f, 0.21f, 0.2f};
            V3 fc = smoke + (fire - smoke) * std::sqrt(heat);
            glUniform3f(locTint, fc.x, fc.y, fc.z);
            fireball.draw();
            glUniform1f(locEmissive, 0);
            glUniform3f(locTint, 1, 1, 1);
        }
        drawDrone = drawDrone && !(frame.flags & FPV_DRONE_DETONATED);  // nothing left of it
#endif
        if (drawDrone) {  // the FPV camera sits inside the frame
            M4 dm = mul(translate({frame.position[0], frame.position[1], frame.position[2]}), fromQuat(frame.rotation));
            glUniformMatrix4fv(locModel, 1, GL_FALSE, dm.m);
            body.draw();
            for (int i = 0; i < 4; i++) {
                if (frame.motors[i].flags & FPV_MOTOR_PROP_BROKEN) continue;
                const float *mp = frame.motors[i].position;
                M4 pm4 = mul(dm, mul(translate({mp[0], mp[1], mp[2]}), rotY((float)propAngle[i])));
                glUniformMatrix4fv(locModel, 1, GL_FALSE, pm4.m);
                prop.draw();
            }
        }

        // post-process to the window: lens distortion + analog noise
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, fbw, fbh);
        glDisable(GL_DEPTH_TEST);
        glUseProgram(postProg);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, fboTex);
        glUniform1i(glGetUniformLocation(postProg, "scene"), 0);
        bool fpv = camMode == 0 && !cutscene;
#if defined(LIBFPV_KAMIKAZE)
        if (ending.showing && ending.view == FPV_KAMIKAZE_VIEW_SIGNAL_LOST) fpv = true;
#endif
        float flash = 0;
#if defined(LIBFPV_KAMIKAZE)
        if (blast.start >= 0 && !fpv) flash = (float)std::fmax(0.0, 0.8 - (time - blast.start) * 3);
#endif
        glUniform1f(glGetUniformLocation(postProg, "flash"), flash);
        float lost = 0;
#if defined(LIBFPV_KAMIKAZE)
        if (fpv && (frame.flags & FPV_DRONE_DETONATED)) lost = 1;  // the camera went with the drone
#endif
        glUniform1f(glGetUniformLocation(postProg, "lost"), lost);
        glUniform2f(glGetUniformLocation(postProg, "k"), fpv ? frame.camera.distortion_k1 : 0, fpv ? frame.camera.distortion_k2 : 0);
        glUniform1f(glGetUniformLocation(postProg, "zoom"), fpv ? frame.camera.distortion_zoom : 1.0f);
        glUniform1f(glGetUniformLocation(postProg, "aspect"), aspect);
        glUniform1f(glGetUniformLocation(postProg, "noise"), fpv ? frame.video.noise : 0);
        glUniform1f(glGetUniformLocation(postProg, "breakup"), fpv ? frame.video.breakup : 0);
        glUniform1f(glGetUniformLocation(postProg, "analog"), frame.video.is_digital ? 0.0f : 1.0f);
        glUniform1f(glGetUniformLocation(postProg, "time"), (float)time);
        glBindVertexArray(emptyVao);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        // OSD (FPV view only) and overlay, in window coordinates
        // the overlay works in window coordinates, so its scale is the content
        // scale only (Windows 150% = 1.5), not the Retina pixel density
        float uiScale = SDL_GetWindowDisplayScale(win) / std::max(1.0f, SDL_GetWindowPixelDensity(win));
        fpv_overlay_set_display((float)ww, (float)wh, uiScale > 0 ? uiScale : 1.0f);
        if (fpv) fpv_gl3_render(&ui, fpv_osd_draw_data(&frame, (float)ww, (float)wh, FPV_OSD_STYLE_ANALOG), fbw, fbh);
        fpv_gl3_render(&ui, fpv_overlay_render(dt), fbw, fbh);

        frameNo++;
#if !defined(__EMSCRIPTEN__)
        if (!screenshot.empty() && frameNo >= frames) {
            savePpm(screenshot.c_str(), fbw, fbh);
            SDL_Log("saved %s (sim %.2f s, %s, height %.1f m, %.0f km/h)", screenshot.c_str(), frame.sim_time,
                    (frame.flags & FPV_DRONE_ARMED) ? "armed" : "disarmed", frame.position[1], frame.speed_kmh);
            running = false;
        }
#endif
        SDL_GL_SwapWindow(win);
    };
#if defined(__EMSCRIPTEN__)
    // The browser drives the frames; main() never returns, so its locals stay.
    static std::function<void()> s_frame;
    s_frame = [&] {
        gameFrame();
        if (!running) emscripten_cancel_main_loop();
    };
    emscripten_set_main_loop([] { s_frame(); }, 0, true);
#else
    while (running) gameFrame();
#endif

    // 7. shutdown
#if defined(LIBFPV_KAMIKAZE)
    if (kamikazeDemo) fpv_kamikaze_set_config(&playerConfig);  // the demo's endings were not the player's
#endif
    if (audio) SDL_DestroyAudioStream(audio);
    fpv_drone_delete(drone);
    fpv_global_terminate();
    fpv_gl3_shutdown(&ui);
    SDL_GL_DestroyContext(gl);
    SDL_DestroyWindow(win);
    SDL_Quit();
#if defined(LIBFPV_KAMIKAZE)
    if (kamikazeDemo) {
        SDL_Log("[demo] %s (%d failed)", demo.failures ? "FAILED" : "ALL PASSED", demo.failures);
        return demo.failures ? 1 : 0;
    }
#endif
    return 0;
}
