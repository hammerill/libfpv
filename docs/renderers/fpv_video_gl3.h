/*
 * fpv_video_gl3.h - draws the FPV view the way goggles show it, with OpenGL
 * 3.3 core (or OpenGL ES 3.0 and WebGL 2 with FPV_GL3_GLES): the fisheye
 * lens, and when the player turns them on (F8 > Camera) the video feed's
 * effects from frame.video_feed: the analog link's noise (snow, sparkles,
 * torn and rolling lines, colour loss, motor noise), digital breakup, and
 * the camera's resolution and aspect with black bars. Single header, next
 * to fpv_render_gl3.h (which it uses for the OSD):
 *
 *   #include <your GL loader>
 *   #define FPV_RENDER_GL3_IMPLEMENTATION     // once, if not done elsewhere
 *   #include "fpv_render_gl3.h"
 *   #define FPV_VIDEO_GL3_IMPLEMENTATION
 *   #include "fpv_video_gl3.h"
 *
 *   fpv_video_gl3 v; fpv_video_gl3_init(&v);           // once, GL context current
 *   every frame: render the scene from frame.camera (vertical FOV
 *   frame.camera.fov_y_degrees, any aspect at least as wide as the camera's)
 *   into a texture, then
 *   fpv_video_gl3_draw(&v, &ui, &frame, scene_tex, scene_w, scene_h, fb_w, fb_h, 1);
 *   fpv_gl3_render(&ui, fpv_overlay_render(dt), fb_w, fb_h);  // overlay on top
 *   fpv_video_gl3_shutdown(&v);
 *
 * It draws into the framebuffer bound when called, over all of it, and the
 * OSD too (draw_osd = 0 skips it): an analog OSD goes into the signal, as
 * the flight controller's OSD chip does, so it gets the noise and the blur.
 * The signal is only redrawn when a new field (analog, 59.94 or 50 a
 * second) or frame (digital) arrives, so the feed runs at the link's rate
 * whatever the game's. Saves and restores the GL state it changes.
 * How it works, to port it: docs/INTEGRATION.md#the-video-feed.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FPV_VIDEO_GL3_H
#define FPV_VIDEO_GL3_H

#include "fpv_render_gl3.h"
#include "libfpv.h"

typedef struct fpv_video_gl3 {
    unsigned int signal_prog[3];  /* per mode: lens only, analog, digital */
    unsigned int present_prog, vao;
    unsigned int fbo[2], tex[2];  /* the signal, ping-pong: [cur] is the latest */
    int cur, raster_w, raster_h;
    unsigned int osd_fbo, osd_tex;
    int osd_w, osd_h;
    unsigned int lines_tex;
    uint32_t field, flags;
    int have;                     /* a signal was drawn */
    int sl[3][32], pl[8];         /* uniform locations */
} fpv_video_gl3;

int fpv_video_gl3_init(fpv_video_gl3 *v);
void fpv_video_gl3_draw(fpv_video_gl3 *v, fpv_gl3 *ui, const fpv_frame *frame, unsigned int scene_tex, int scene_w,
                        int scene_h, int fb_w, int fb_h, int draw_osd);
void fpv_video_gl3_shutdown(fpv_video_gl3 *v);

#endif

#ifdef FPV_VIDEO_GL3_IMPLEMENTATION
#include <math.h>
#include <string.h>

#ifndef GL_FRAMEBUFFER_BINDING
#define GL_FRAMEBUFFER_BINDING 0x8CA6
#endif

#ifdef FPV_GL3_GLES
#define FPV_VIDEO_GL3_VERSION "#version 300 es\nprecision highp float;\nprecision highp int;\nprecision highp sampler2D;\n"
#else
#define FPV_VIDEO_GL3_VERSION "#version 330 core\n"
#endif

static const char *fpv_video_gl3__vs =
    "out vec2 uv;\n"
    "void main() { vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));\n"
    "  uv = p; gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0); }\n";

/* The signal: one pixel of the feed's raster per fragment. Built once per
 * mode (FPV_VIDEO_MODE 0 lens only, 1 analog, 2 digital) rather than as one
 * program that branches on it: as one, it is too big for Direct3D's shader
 * compiler, which ANGLE (Chrome and Edge on Windows) runs it through, and
 * the browser's GPU process dies of a stack overflow compiling it. */
static const char *fpv_video_gl3__signal_fs =
    "in vec2 uv; out vec4 o;\n"
    "uniform sampler2D scene, osd, lines, history;\n"
    "uniform vec2 raster;        // pixels of this target\n"
    "uniform float noise, res, osdIn;\n"
    "uniform float picAspect, sceneAspect;\n"
    "uniform vec2 k; uniform float zoom; uniform vec2 sceneTexel;\n"
    "uniform float lumaCycles, chromaCycles, subcarrier, sharpen;\n"
    "uniform float parity, roll, seed, activeLines, totalLines, ntsc;\n"
    "uniform vec4 interf;\n"
    "uniform vec2 digital; uniform float blocks, softness;\n"
    "const float PI = 3.14159265;\n"
    /* Dave Hoskins' hash without sine: the same on every GPU */
    "float h13(vec3 p) { p = fract(p * 0.1031); p += dot(p, p.zyx + 31.32); return fract((p.x + p.y) * p.z); }\n"
    "float gn(vec3 p) { return (h13(p) + h13(p + vec3(17.1, 3.7, 9.3)) - 1.0) * 2.45; }\n"
    /* picture (0..1, y up) to the scene texture through the lens */
    "vec2 sceneUv(vec2 p) {\n"
    "  vec2 scale = vec2(picAspect, 1.0) / (0.5 * sqrt(picAspect * picAspect + 1.0));\n"
    "  vec2 c = (p - 0.5) * scale; float r2 = dot(c, c);\n"
    "  vec2 d = c * (1.0 + k.x * r2 + k.y * r2 * r2) / zoom / scale;\n"
    "  float s = picAspect / sceneAspect;\n"
    "  if (s <= 1.0) d.x *= s; else d.y /= s;\n"
    "  return d + 0.5;\n"
    "}\n"
    "vec3 tap(vec2 p) { vec2 s = sceneUv(p);\n"
    "  return (s.x < 0.0 || s.x > 1.0 || s.y < 0.0 || s.y > 1.0) ? vec3(0.0) : texture(scene, s).rgb; }\n"
    "// the OSD chip's 4:3 grid; over a picture of another aspect (resolution off) it stays 4:3\n"
    "vec4 osdTap(vec2 p) {\n"
    "  if (osdIn < 0.5) return vec4(0.0);\n"
    "  if (res < 0.5) p.x = (p.x - 0.5) * picAspect / (4.0 / 3.0) + 0.5;\n"
    "  return (p.x < 0.0 || p.x > 1.0) ? vec4(0.0) : texture(osd, p);\n"
    "}\n"
    "const mat3 toYiq = mat3(0.299, 0.596, 0.211, 0.587, -0.274, -0.523, 0.114, -0.322, 0.312);\n"
    "const mat3 toRgb = mat3(1.0, 1.0, 1.0, 0.956, -0.272, -1.106, 0.621, -0.647, 1.703);\n"
    "#if FPV_VIDEO_MODE == 1\n"
    "vec3 analog() {\n"
    "  vec2 px = floor(uv * raster);\n"
    "  // the field line this pixel shows (from the top) and, when the picture\n"
    "  // rolls, the transmitted line it carries\n"
    "  float lineF = res > 0.5 ? raster.y - 1.0 - px.y : (1.0 - uv.y) * activeLines;\n"
    "  float rolled = mod(lineF + roll * totalLines, totalLines);\n"
    "  float line = floor(rolled);\n"
    "  vec4 t0 = vec4(0.0, 0.0, 1.0, 1.0), t1 = vec4(0.5, 0.5, 0.5, 0.5);\n"
    "  if (noise > 0.5) { t0 = texelFetch(lines, ivec2(0, int(line)), 0); t1 = texelFetch(lines, ivec2(1, int(line)), 0); }\n"
    "  float shift = noise > 0.5 ? (floor(t1.r * 255.0 + 0.5) * 256.0 + floor(t1.g * 255.0 + 0.5)) / 65535.0 * 0.5 - 0.25 : 0.0;\n"
    "  bool blank = rolled >= activeLines;  // the vertical blanking: a black bar\n"
    "  // where the line sits in the camera's picture: interlaced, a field carries every other line\n"
    "  float yTop = res > 0.5 ? (2.0 * line + parity + 0.5) / (2.0 * activeLines) : rolled / activeLines;\n"
    "  float x = (res > 0.5 ? (px.x + 0.5) / raster.x : uv.x) + shift;\n"
    "  vec2 p = vec2(x, 1.0 - yTop);\n"
    "  vec3 yiq;\n"
    "  if (res > 0.5) {\n"
    "    // the camera and the link: luma limited to lumaCycles, colour to chromaCycles,\n"
    "    // colour a little late; a quarter line up and down against vertical aliasing\n"
    "    float sl = 0.35 / lumaCycles, sc = 0.35 / chromaCycles, dv = 0.25 / (2.0 * activeLines);\n"
    "    float Y = 0.0, Yring = 0.0, wsum = 0.0; vec3 wide = vec3(0.0); float wcs = 0.0;\n"
    "    for (int i = -3; i <= 3; i++) {\n"
    "      float fi = float(i); float w = exp(-0.5 * (fi * 0.75) * (fi * 0.75));\n"
    "      vec2 q = p + vec2(fi * 0.75 * sl, (i - (i / 2) * 2) != 0 ? dv : -dv);\n"
    "      vec3 c = tap(q); vec4 od = osdTap(q); c = mix(c, od.rgb, od.a);\n"
    "      float y = dot(c, vec3(0.299, 0.587, 0.114));\n"
    "      Y += w * y; wsum += w; if (abs(i) >= 2) Yring += y * 0.25;\n"
    "      vec3 cw = tap(p + vec2(fi * 0.75 * sc + 0.3 * sc, 0.0)); vec4 ow = osdTap(p + vec2(fi * 0.75 * sc + 0.3 * sc, 0.0));\n"
    "      wide += w * mix(cw, ow.rgb, ow.a); wcs += w;\n"
    "    }\n"
    "    Y /= wsum; wide /= wcs;\n"
    "    Y += sharpen * 1.6 * (Y - Yring);  // the camera's edge enhancement: the halos\n"
    "    yiq = vec3(Y, (toYiq * wide).yz);\n"
    "    // composite: fine detail near the subcarrier decodes as colour (rainbows)\n"
    "    float hp = 0.5 / subcarrier;\n"
    "    float yc = dot(tap(p), vec3(0.299, 0.587, 0.114));\n"
    "    float yhf = yc - 0.5 * (dot(tap(p + vec2(hp, 0.0)), vec3(0.299, 0.587, 0.114)) +\n"
    "                            dot(tap(p - vec2(hp, 0.0)), vec3(0.299, 0.587, 0.114)));\n"
    "    float ph = 2.0 * PI * (x * subcarrier + 0.5 * line + 0.5 * parity);\n"
    "    yiq.yz += 0.45 * yhf * vec2(cos(ph), sin(ph));\n"
    "  } else {\n"
    "    vec3 c = tap(p); vec4 od = osdTap(p); yiq = toYiq * mix(c, od.rgb, od.a);\n"
    "  }\n"
    "  // the black porch where a torn line starts, the blanking bar of a rolling picture:\n"
    "  // black while there is a signal, the receiver's snow when there is none\n"
    "  if (x < 0.0 || x > 1.0) yiq = vec3(0.0);\n"
    "  if (noise > 0.5) {\n"
    "    float grain = t0.r * 0.5, density = t0.g * t0.g * t0.g, chroma = t0.b, pic = t0.a;\n"
    "    float hue = (t1.a - 0.5) * 2.0 * PI;\n"
    "    vec3 sd = vec3(0.0, line, seed);\n"
    "    // grain: the link's bandwidth stretches it along the line\n"
    "    float gc = x * lumaCycles * 2.0; float gi = floor(gc), gf = gc - gi;\n"
    "    float g = mix(gn(sd + vec3(gi, 0.0, 0.0)), gn(sd + vec3(gi + 1.0, 0.0, 0.0)), gf);\n"
    "    float cc = x * 90.0; float ci = floor(cc), cf = cc - ci;\n"
    "    vec2 gcol = mix(vec2(gn(sd + vec3(ci, 51.0, 0.0)), gn(sd + vec3(ci, 77.0, 0.0))),\n"
    "                    vec2(gn(sd + vec3(ci + 1.0, 51.0, 0.0)), gn(sd + vec3(ci + 1.0, 77.0, 0.0))), cf);\n"
    "    // no picture: the receiver shows its own noise, coarse grey streaks\n"
    "    float sc2 = x * 260.0; float si = floor(sc2), sf = sc2 - si;\n"
    "    float snow = 0.42 + 0.30 * mix(gn(sd + vec3(si, 9.0, 1.0)), gn(sd + vec3(si + 1.0, 9.0, 1.0)), sf)\n"
    "                      + 0.12 * g;\n"
    "    if (blank) { yiq = vec3(0.01, 0.0, 0.0); chroma = 0.0; }\n"
    "    yiq.x = mix(snow, yiq.x, pic);\n"
    "    yiq.yz *= pic;\n"
    "    // colour: the burst's phase (hue) and the colour killer\n"
    "    float cs = cos(hue), sn = sin(hue);\n"
    "    yiq.yz = mat2(cs, sn, -sn, cs) * yiq.yz * chroma;\n"
    "    yiq.x += grain * g;\n"
    "    // FM noise is strongest high up the spectrum, where the colour lives: colour\n"
    "    // speckle while the burst holds; the colour killer leaves grey snow\n"
    "    yiq.yz += grain * 1.6 * gcol * chroma;\n"
    "    // FM clicks: short comets, white on dark, black on bright, with a colour fringe\n"
    "    if (density > 0.0) {\n"
    "      float kc = x * 360.0; float k0 = floor(kc);\n"
    "      for (int j = 0; j < 6; j++) {\n"
    "        float cell = k0 - float(j);\n"
    "        vec3 q = sd + vec3(cell, 0.0, 33.0);\n"
    "        if (h13(q) < density) {\n"
    "          float dist = kc - cell - h13(q + 1.3) * 0.8;\n"
    "          float len = 0.6 + 3.5 * h13(q + 2.9);\n"
    "          float v = dist < 0.0 ? 0.0 : exp(-dist / len) * (1.0 - exp(-dist * 6.0));\n"
    "          float pol = h13(q + 4.7) < 0.8 ? (yiq.x < 0.45 ? 1.0 : -1.0) : (h13(q + 6.1) < 0.5 ? 1.0 : -1.0);\n"
    "          yiq.x += pol * v * 1.4;\n"
    "          yiq.yz += (vec2(h13(q + 7.7), h13(q + 8.3)) - 0.5) * v * 0.9 * chroma;\n"
    "        }\n"
    "      }\n"
    "    }\n"
    "    // motor and ESC noise on the power wiring, and the line's own offset\n"
    "    yiq.x += interf.x * sin(2.0 * PI * (x * interf.y + line * interf.z + interf.w));\n"
    "    yiq.x += t1.b - 0.5;\n"
    "  }\n"
    "  vec3 rgb = toRgb * yiq;\n"
    "  return clamp(rgb, 0.0, 1.0);\n"
    "}\n"
    "#elif FPV_VIDEO_MODE == 2\n"
    "vec3 boxTap(vec2 p, vec2 size) {\n"
    "  vec2 o = 0.25 * size;\n"
    "  return 0.25 * (tap(p + vec2(-o.x, -o.y)) + tap(p + vec2(o.x, -o.y)) + tap(p + vec2(-o.x, o.y)) + tap(p + vec2(o.x, o.y)));\n"
    "}\n"
    "vec3 digitalPix() {\n"
    "  vec2 p = uv;\n"
    "  vec3 c = res > 0.5 ? boxTap(p, 1.0 / raster) : tap(p);\n"
    "  vec2 bsize = 16.0 / digital;\n"
    "  vec2 b = floor(p / bsize);\n"
    "  if (softness > 0.0) {  // the bitrate drops: detail goes first, in blocks\n"
    "    vec2 bc = (b + 0.5) * bsize;\n"
    "    vec3 dc = boxTap(bc, bsize);\n"
    "    vec3 mid = boxTap(p, bsize * 0.5);\n"
    "    c = mix(c, mix(mid, dc, 0.35 * softness), softness);\n"
    "  }\n"
    "  if (noise > 0.5 && blocks > 0.0) {\n"
    "    vec3 q = vec3(b, seed);\n"
    "    // errors come in runs along the stream: a lost block often takes its neighbours\n"
    "    // a lost packet takes a run of blocks along the row, then the decoder resyncs\n"
    "    float runStart = floor(b.x / 6.0);\n"
    "    float run = h13(vec3(runStart, b.y, seed + 3.1));\n"
    "    float runLen = 1.0 + floor(h13(vec3(runStart, b.y, seed + 9.7)) * 6.0);\n"
    "    bool inRun = run < blocks * 0.7 && b.x - runStart * 6.0 < runLen;\n"
    "    if (inRun || h13(q) < blocks * 0.3) {\n"
    "      float kind = h13(q + 5.5);\n"
    "      if (kind < 0.6) {  // stale: last frame's block, moved by a broken motion vector\n"
    "        vec2 mv = (vec2(h13(q + 1.1), h13(q + 2.2)) - 0.5) * bsize * 3.0;\n"
    "        c = texture(history, p + mv).rgb;\n"
    "      } else if (kind < 0.9) {  // only the DC: a flat block, colour off\n"
    "        vec3 dc = boxTap((b + 0.5) * bsize, bsize);\n"
    "        vec3 y = toYiq * dc; y.yz += (vec2(h13(vec3(runStart, b.y, seed + 3.3)), h13(vec3(runStart, b.y, seed + 4.4))) - 0.5) * 0.18;\n"
    "        vec2 f = fract(p / bsize) * 2.0 - 1.0;\n"
    "        y.x += 0.08 * (h13(q + 6.6) - 0.5) * (f.x + f.y);\n"
    "        c = toRgb * y;\n"
    "      } else {  // garbage coefficients: stripes\n"
    "        vec2 f = fract(p / bsize);\n"
    "        float s = cos(PI * floor(h13(q + 7.7) * 8.0) * f.x) * cos(PI * floor(h13(q + 8.8) * 8.0) * f.y);\n"
    "        c = clamp(boxTap((b + 0.5) * bsize, bsize) + 0.25 * s * vec3(0.8, 1.0, 0.7), 0.0, 1.0);\n"
    "      }\n"
    "    }\n"
    "  }\n"
    "  return clamp(c, 0.0, 1.0);\n"
    "}\n"
    "#endif\n"
    "void main() {\n"
    "#if FPV_VIDEO_MODE == 1\n"
    "  vec3 c = analog();\n"
    "#elif FPV_VIDEO_MODE == 2\n"
    "  vec3 c = digitalPix();\n"
    "#else\n"
    "  vec3 c = tap(uv);\n"
    "  if (osdIn > 0.5) { vec4 od = osdTap(uv); c = mix(c, od.rgb, od.a); }\n"
    "#endif\n"
    "  o = vec4(c, 1.0);\n"
    "}\n";

/* The goggles: the signal scaled into the picture, black around it. */
static const char *fpv_video_gl3__present_fs =
    "in vec2 uv; out vec4 o;\n"
    "uniform sampler2D signal; uniform vec4 rect; uniform vec2 rows;  // rows: field lines, parity (0 = progressive)\n"
    "void main() {\n"
    "  vec2 p = (gl_FragCoord.xy - rect.xy) / rect.zw;\n"
    "  if (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0) { o = vec4(0.0, 0.0, 0.0, 1.0); return; }\n"
    "  if (rows.x > 0.0) {\n"
    "    // bob: a field's lines sit half a line lower when it carries the odd lines\n"
    "    float fromTop = (1.0 - p.y) * rows.x - 0.5 * rows.y;\n"
    "    p.y = 1.0 - fromTop / rows.x;\n"
    "  }\n"
    "  o = vec4(texture(signal, p).rgb, 1.0);\n"
    "}\n";

#ifndef FPV_VIDEO_GL3_LOG
#include <stdio.h>
#define FPV_VIDEO_GL3_LOG(...) fprintf(stderr, __VA_ARGS__)
#endif

static int fpv_video_gl3__log(unsigned int obj, int is_program, const char *what, const char *name) {
    char log[2048];
    log[0] = 0;
    if (is_program) glGetProgramInfoLog(obj, (int)sizeof log, NULL, log);
    else glGetShaderInfoLog(obj, (int)sizeof log, NULL, log);
    FPV_VIDEO_GL3_LOG("fpv_video_gl3: the %s shader failed to %s: %s\n", name, what,
                      log[0] ? log : "(no log: the GPU driver or the browser's GPU process may have crashed)");
    return 0;
}

/* Builds a program; define is "" or "#define ...\n". 0 on failure, logged. */
static unsigned int fpv_video_gl3__program(const char *define, const char *fs, const char *name) {
    const char *vsrc[2] = {FPV_VIDEO_GL3_VERSION, fpv_video_gl3__vs};
    const char *fsrc[3] = {FPV_VIDEO_GL3_VERSION, define, fs};
    unsigned int v = glCreateShader(GL_VERTEX_SHADER), f = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(v, 2, vsrc, NULL);
    glCompileShader(v);
    glShaderSource(f, 3, fsrc, NULL);
    glCompileShader(f);
    unsigned int p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    int ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        int vok = 0, fok = 0;
        glGetShaderiv(v, GL_COMPILE_STATUS, &vok);
        glGetShaderiv(f, GL_COMPILE_STATUS, &fok);
        if (!vok) fpv_video_gl3__log(v, 0, "compile", "vertex");
        if (!fok) fpv_video_gl3__log(f, 0, "compile", name);
        if (vok && fok) fpv_video_gl3__log(p, 1, "link", name);
        glDeleteProgram(p);
        p = 0;
    }
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

static const char *fpv_video_gl3__signal_uniforms[] = {
    "scene", "osd", "lines", "history", "raster", "noise", "res", "osdIn", "picAspect", "sceneAspect",
    "k", "zoom", "sceneTexel", "lumaCycles", "chromaCycles", "subcarrier", "sharpen", "parity", "roll", "seed",
    "activeLines", "totalLines", "ntsc", "interf", "digital", "blocks", "softness", NULL};
enum {
    FPV_VU_SCENE, FPV_VU_OSD, FPV_VU_LINES, FPV_VU_HISTORY, FPV_VU_RASTER, FPV_VU_NOISE, FPV_VU_RES,
    FPV_VU_OSDIN, FPV_VU_PICASPECT, FPV_VU_SCENEASPECT, FPV_VU_K, FPV_VU_ZOOM, FPV_VU_SCENETEXEL, FPV_VU_LUMA,
    FPV_VU_CHROMA, FPV_VU_SUBC, FPV_VU_SHARPEN, FPV_VU_PARITY, FPV_VU_ROLL, FPV_VU_SEED, FPV_VU_ACTIVE,
    FPV_VU_TOTAL, FPV_VU_NTSC, FPV_VU_INTERF, FPV_VU_DIGITAL, FPV_VU_BLOCKS, FPV_VU_SOFTNESS
};

static void fpv_video_gl3__texparams(int filter) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

/* Returns 0 (and holds nothing) when a shader fails; the reason is logged. */
int fpv_video_gl3_init(fpv_video_gl3 *v) {
    static const char *defines[3] = {"#define FPV_VIDEO_MODE 0\n", "#define FPV_VIDEO_MODE 1\n",
                                     "#define FPV_VIDEO_MODE 2\n"};
    static const char *names[3] = {"lens", "analog video", "digital video"};
    memset(v, 0, sizeof(*v));
    for (int m = 0; m < 3; m++) {
        v->signal_prog[m] = fpv_video_gl3__program(defines[m], fpv_video_gl3__signal_fs, names[m]);
        if (!v->signal_prog[m]) break;
    }
    if (v->signal_prog[0] && v->signal_prog[1] && v->signal_prog[2])
        v->present_prog = fpv_video_gl3__program("", fpv_video_gl3__present_fs, "present");
    if (!v->present_prog) {
        fpv_video_gl3_shutdown(v);
        return 0;
    }
    for (int m = 0; m < 3; m++)
        for (int i = 0; fpv_video_gl3__signal_uniforms[i]; i++)
            v->sl[m][i] = glGetUniformLocation(v->signal_prog[m], fpv_video_gl3__signal_uniforms[i]);
    v->pl[0] = glGetUniformLocation(v->present_prog, "signal");
    v->pl[1] = glGetUniformLocation(v->present_prog, "rect");
    v->pl[2] = glGetUniformLocation(v->present_prog, "rows");
    glGenVertexArrays(1, &v->vao);
    glGenTextures(1, &v->lines_tex);
    glBindTexture(GL_TEXTURE_2D, v->lines_tex);
    fpv_video_gl3__texparams(GL_NEAREST);
    return 1;
}

/* (Re)creates a render target texture. */
static void fpv_video_gl3__target(unsigned int *fbo, unsigned int *tex, int w, int h, int filter) {
    if (!*tex) glGenTextures(1, tex);
    if (!*fbo) glGenFramebuffers(1, fbo);
    glBindTexture(GL_TEXTURE_2D, *tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    fpv_video_gl3__texparams(filter);
    glBindFramebuffer(GL_FRAMEBUFFER, *fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, *tex, 0);
}

void fpv_video_gl3_draw(fpv_video_gl3 *v, fpv_gl3 *ui, const fpv_frame *f, unsigned int scene_tex, int scene_w,
                        int scene_h, int fb_w, int fb_h, int draw_osd) {
    if (!v->present_prog || fb_w <= 0 || fb_h <= 0 || scene_w <= 0 || scene_h <= 0) return;
    const fpv_video_feed *vf = &f->video_feed;
    /* an older libfpv fills no video_feed: struct_size tells */
    int has_feed = f->struct_size >= offsetof(fpv_frame, video_feed) + sizeof(fpv_video_feed);
    uint32_t flags = has_feed ? vf->flags : 0u;
    int noise = (flags & FPV_VIDEO_FEED_NOISE) != 0, res = (flags & FPV_VIDEO_FEED_RESOLUTION) != 0;
    int analog = has_feed && vf->system != FPV_VIDEO_DIGITAL;
    int mode = !flags ? 0 : analog ? 1 : 2;
    int osd_in = draw_osd && ui && mode == 1 && (flags & FPV_VIDEO_FEED_OSD_IN_SIGNAL);
    float active = vf->system == FPV_VIDEO_PAL ? 288.0f : 240.0f;
    float total = vf->system == FPV_VIDEO_PAL ? 313.0f : 263.0f;

    /* the picture on the screen: the camera's aspect with black bars, or all of it */
    float rx = 0, ry = 0, rw = (float)fb_w, rh = (float)fb_h;
    float pic_aspect = (float)fb_w / (float)fb_h;
    if (res && vf->picture_aspect > 0.1f) {
        pic_aspect = vf->picture_aspect;
        if (rw / rh > pic_aspect) rw = rh * pic_aspect, rx = (fb_w - rw) * 0.5f;
        else rh = rw / pic_aspect, ry = (fb_h - rh) * 0.5f;
    }
    /* the signal's raster: an analog field, a digital frame, or the screen's pixels */
    int w = (int)rw, h = (int)rh;
    if (res && mode == 1) w = (int)vf->raster_width, h = (int)active;
    else if (res && mode == 2) w = (int)vf->raster_width, h = (int)vf->raster_height;
    if (w < 1) w = 1;
    if (h < 1) h = 1;

    /* save state */
    int last_program, last_vao, last_fbo, last_active, last_viewport[4], last_tex[4];
    glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vao);
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &last_fbo);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &last_active);
    glGetIntegerv(GL_VIEWPORT, last_viewport);
    for (int i = 0; i < 4; i++) {
        glActiveTexture(GL_TEXTURE0 + i);
        glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_tex[i]);
    }
    unsigned char was_blend = glIsEnabled(GL_BLEND), was_cull = glIsEnabled(GL_CULL_FACE);
    unsigned char was_depth = glIsEnabled(GL_DEPTH_TEST), was_scissor = glIsEnabled(GL_SCISSOR_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_SCISSOR_TEST);

    int resized = w != v->raster_w || h != v->raster_h;
    if (resized) {
        for (int i = 0; i < 2; i++) fpv_video_gl3__target(&v->fbo[i], &v->tex[i], w, h, GL_LINEAR);
        v->raster_w = w, v->raster_h = h;
        v->have = 0;
    }
    /* a new field (frame), a change of settings, or no effects at all: draw the signal */
    int fresh = !v->have || !flags || vf->field != v->field || flags != v->flags;
    int hold = mode == 2 && noise && vf->frozen && v->have;  /* the goggles hold the last frame */
    if (fresh && !hold) {
        if (osd_in) {
            /* the OSD chip's picture: the analog 4:3 OSD stretched over the raster */
            int ow = res ? w : (int)vf->raster_width, oh = res ? (int)active * 2 : (int)(active * 2);
            if (ow != v->osd_w || oh != v->osd_h) {
                fpv_video_gl3__target(&v->osd_fbo, &v->osd_tex, ow, oh, GL_LINEAR);
                v->osd_w = ow, v->osd_h = oh;
            }
            glBindFramebuffer(GL_FRAMEBUFFER, v->osd_fbo);
            glViewport(0, 0, ow, oh);
            glClearColor(0, 0, 0, 0);
            glClear(GL_COLOR_BUFFER_BIT);
            fpv_gl3_render(ui, fpv_osd_draw_data(f, oh * 4.0f / 3.0f, (float)oh, FPV_OSD_STYLE_ANALOG), ow, oh);
            glDisable(GL_BLEND);
            glDisable(GL_SCISSOR_TEST);
        }
        if (noise && mode == 1) {
            glActiveTexture(GL_TEXTURE0 + 2);
            glBindTexture(GL_TEXTURE_2D, v->lines_tex);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 2, vf->line_count ? (int)vf->line_count : 1, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, vf->lines);
        }
        int next = v->have ? 1 - v->cur : v->cur;
        glBindFramebuffer(GL_FRAMEBUFFER, v->fbo[next]);
        glViewport(0, 0, w, h);
        glUseProgram(v->signal_prog[mode]);
        const int *L = v->sl[mode];
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, scene_tex);
        glActiveTexture(GL_TEXTURE0 + 1);
        glBindTexture(GL_TEXTURE_2D, osd_in ? v->osd_tex : 0);
        /* the last frame, never the target: with no signal yet, the other
           (blank) one, as sampling the target is a feedback loop */
        glActiveTexture(GL_TEXTURE0 + 3);
        glBindTexture(GL_TEXTURE_2D, v->tex[1 - next]);
        glUniform1i(L[FPV_VU_SCENE], 0);
        glUniform1i(L[FPV_VU_OSD], 1);
        glUniform1i(L[FPV_VU_LINES], 2);
        glUniform1i(L[FPV_VU_HISTORY], 3);
        glUniform2f(L[FPV_VU_RASTER], (float)w, (float)h);
        glUniform1f(L[FPV_VU_NOISE], (float)noise);
        glUniform1f(L[FPV_VU_RES], (float)res);
        glUniform1f(L[FPV_VU_OSDIN], (float)osd_in);
        glUniform1f(L[FPV_VU_PICASPECT], pic_aspect);
        glUniform1f(L[FPV_VU_SCENEASPECT], (float)scene_w / (float)scene_h);
        glUniform2f(L[FPV_VU_K], f->camera.distortion_k1, f->camera.distortion_k2);
        glUniform1f(L[FPV_VU_ZOOM], f->camera.distortion_zoom > 0 ? f->camera.distortion_zoom : 1.0f);
        glUniform2f(L[FPV_VU_SCENETEXEL], 1.0f / scene_w, 1.0f / scene_h);
        glUniform1f(L[FPV_VU_LUMA], vf->luma_cycles > 1 ? vf->luma_cycles : 250.0f);
        glUniform1f(L[FPV_VU_CHROMA], vf->chroma_cycles > 1 ? vf->chroma_cycles : 53.0f);
        glUniform1f(L[FPV_VU_SUBC], vf->subcarrier_cycles > 1 ? vf->subcarrier_cycles : 188.5f);
        glUniform1f(L[FPV_VU_SHARPEN], vf->sharpening);
        glUniform1f(L[FPV_VU_PARITY], (float)(vf->field_parity & 1));
        glUniform1f(L[FPV_VU_ROLL], vf->roll);
        glUniform1f(L[FPV_VU_SEED], vf->seed);
        glUniform1f(L[FPV_VU_ACTIVE], active);
        glUniform1f(L[FPV_VU_TOTAL], total);
        glUniform1f(L[FPV_VU_NTSC], vf->system == FPV_VIDEO_NTSC ? 1.0f : 0.0f);
        glUniform4f(L[FPV_VU_INTERF], vf->interference[0], vf->interference[1], vf->interference[2], vf->interference[3]);
        glUniform2f(L[FPV_VU_DIGITAL], vf->raster_width ? (float)vf->raster_width : (float)w,
                    vf->raster_height ? (float)vf->raster_height : (float)h);
        glUniform1f(L[FPV_VU_BLOCKS], vf->block_errors);
        glUniform1f(L[FPV_VU_SOFTNESS], noise ? vf->softness : 0.0f);
        glBindVertexArray(v->vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        v->cur = next;
        v->have = 1;
        v->field = vf->field;
        v->flags = flags;
    }

    /* the goggles */
    glBindFramebuffer(GL_FRAMEBUFFER, (unsigned)last_fbo);
    glViewport(0, 0, fb_w, fb_h);
    glUseProgram(v->present_prog);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, v->tex[v->cur]);
    glUniform1i(v->pl[0], 0);
    glUniform4f(v->pl[1], rx, ry, rw, rh);
    glUniform2f(v->pl[2], res && mode == 1 ? active : 0.0f, (float)(vf->field_parity & 1));
    glBindVertexArray(v->vao);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    /* the OSD over the picture (digital: the goggles draw it; or effects off) */
    if (draw_osd && ui && !osd_in) {
        int style = has_feed && flags ? (int)vf->osd_style : FPV_OSD_STYLE_ANALOG;
        fpv_gl3_render(ui, fpv_osd_draw_data(f, (float)fb_w, (float)fb_h, (fpv_osd_style)style), fb_w, fb_h);
    }

    /* restore state */
    glUseProgram((unsigned)last_program);
    glBindVertexArray((unsigned)last_vao);
    for (int i = 3; i >= 0; i--) {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, (unsigned)last_tex[i]);
    }
    glActiveTexture((unsigned)last_active);
    glViewport(last_viewport[0], last_viewport[1], last_viewport[2], last_viewport[3]);
    if (was_blend) glEnable(GL_BLEND); else glDisable(GL_BLEND);
    if (was_cull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    if (was_depth) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (was_scissor) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST);
}

void fpv_video_gl3_shutdown(fpv_video_gl3 *v) {
    for (int i = 0; i < 2; i++) {
        if (v->tex[i]) glDeleteTextures(1, &v->tex[i]);
        if (v->fbo[i]) glDeleteFramebuffers(1, &v->fbo[i]);
    }
    if (v->osd_tex) glDeleteTextures(1, &v->osd_tex);
    if (v->osd_fbo) glDeleteFramebuffers(1, &v->osd_fbo);
    if (v->lines_tex) glDeleteTextures(1, &v->lines_tex);
    if (v->vao) glDeleteVertexArrays(1, &v->vao);
    for (int m = 0; m < 3; m++)
        if (v->signal_prog[m]) glDeleteProgram(v->signal_prog[m]);
    if (v->present_prog) glDeleteProgram(v->present_prog);
    memset(v, 0, sizeof(*v));
}
#endif
