/*
 * fpv_video_d3d9.h - fpv_video_gl3.h for Direct3D 9 games: the same goggles
 * (the fisheye lens, and when the player turns them on in F8 > Camera the
 * video feed's analog noise, digital breakup and the camera's resolution and
 * aspect from frame.video_feed) with ps_3_0 / vs_3_0 shaders. The HLSL is
 * fpv_video_gl3.h's GLSL, only translated: keep the three headers in step.
 * C++ (Direct3D 9's interfaces). Single header:
 *
 *   #include <d3d9.h>
 *   #define FPV_VIDEO_D3D9_IMPLEMENTATION
 *   #include "fpv_video_d3d9.h"
 *
 *   fpv_video_d3d9 v = {};
 *   fpv_video_d3d9_compile(&v);   // once, on any thread (~0.7 s): loads
 *                                 // d3dcompiler_47.dll; 0 = no video feed
 *   every frame, the scene rendered from frame.camera (vertical FOV
 *   frame.camera.fov_y_degrees) into a texture:
 *   fpv_video_d3d9_draw(&v, dev, &frame, scene_tex, scene_w, scene_h, ui_tex, fb_w, fb_h, draw_osd, user);
 *   on a device reset: fpv_video_d3d9_release(&v); at the end: fpv_video_d3d9_shutdown(&v);
 *
 * Draw only after compile returned 1 (from another thread: once it has
 * returned). It draws into the render target bound when called, over all of
 * it, and restores the device's state (a state block, the render target and
 * the depth surface). It returns 0 when it can't draw (no ps_3_0 with 1024
 * slots, a texture failed): then draw the view without it.
 *
 * `ui_tex` (or NULL) is the game's own UI to keep in the picture, the size of
 * the render target, premultiplied alpha: it goes into the camera's picture
 * after the lens (not bent by it) and under the OSD, so the feed's noise
 * covers it.
 *
 * The OSD: an analog OSD that goes into the signal
 * (FPV_VIDEO_FEED_OSD_IN_SIGNAL) is built here from frame.osd and libfpv's
 * font, pixel for pixel like the OSD chip. draw_osd(user, data, w, h) draws
 * libfpv draw data (fpv_osd_draw_data) over the picture (digital, or the
 * effects off), alpha-blended over the bound w x h target; NULL draws no OSD.
 *
 * A game that loads libfpv at runtime defines, before the implementation,
 * FPV_VIDEO_D3D9_TEXTURE_GET and FPV_VIDEO_D3D9_OSD_DRAW_DATA to its
 * pointers to fpv_texture_get and fpv_osd_draw_data, and FPV_VIDEO_D3D9_LOG
 * (printf-like) to its log.
 * How it works: docs/INTEGRATION.md#the-video-feed.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FPV_VIDEO_D3D9_H
#define FPV_VIDEO_D3D9_H

#include <d3d9.h>
#include <stdint.h>

#include "libfpv.h"

typedef void (*fpv_video_d3d9_osd_fn)(void *user, IDirect3DDevice9 *dev, const fpv_draw_data *data, float width,
                                      float height);

typedef struct fpv_video_d3d9 {
    /* compiled shaders: signal for modes 0 (lens only), 1 (analog), 2 (digital), present, the quad's vs */
    void *code[5];
    unsigned int code_size[5];
    int compiled;                     /* 1 ok, -1 failed, 0 not yet */
    /* device resources */
    int tried, ok;
    IDirect3DPixelShader9 *ps[4];
    IDirect3DVertexShader9 *vs;
    IDirect3DVertexDeclaration9 *decl;
    IDirect3DStateBlock9 *state;
    IDirect3DTexture9 *tex[2];        /* the signal, ping-pong: [cur] is the latest */
    int cur, raster_w, raster_h;
    IDirect3DTexture9 *osd;           /* the OSD chip's picture, analog */
    int osd_lines;
    unsigned char osd_grid[FPV_OSD_ROWS * FPV_OSD_COLS];
    IDirect3DTexture9 *lines;
    uint32_t field, flags;
    int have;                         /* a signal was drawn */
} fpv_video_d3d9;

/* Compiles the shaders (any thread, once). 1 when they are ready. */
int fpv_video_d3d9_compile(fpv_video_d3d9 *v);
/* 1 when it drew. */
int fpv_video_d3d9_draw(fpv_video_d3d9 *v, IDirect3DDevice9 *dev, const fpv_frame *frame, IDirect3DTexture9 *scene,
                        unsigned int scene_w, unsigned int scene_h, IDirect3DTexture9 *ui, unsigned int fb_w,
                        unsigned int fb_h, fpv_video_d3d9_osd_fn draw_osd, void *user);
/* Whether draw can work: compiled, and the device has what it needs (as far as known yet). */
int fpv_video_d3d9_usable(const fpv_video_d3d9 *v);
/* The feed's flags in a frame (0 for an older libfpv that fills none). */
uint32_t fpv_video_d3d9_flags(const fpv_frame *frame);
/* Releases the device's resources (device lost or reset); the next draw makes them again. */
void fpv_video_d3d9_release(fpv_video_d3d9 *v);
/* Releases everything, the compiled shaders too. */
void fpv_video_d3d9_shutdown(fpv_video_d3d9 *v);

#endif

#ifdef FPV_VIDEO_D3D9_IMPLEMENTATION
#include <windows.h>

#include <d3dcompiler.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifndef FPV_VIDEO_D3D9_LOG
#include <stdio.h>
#define FPV_VIDEO_D3D9_LOG(...) (fprintf(stderr, __VA_ARGS__), fputc('\n', stderr))
#endif
#ifndef FPV_VIDEO_D3D9_TEXTURE_GET
#define FPV_VIDEO_D3D9_TEXTURE_GET fpv_texture_get
#endif
#ifndef FPV_VIDEO_D3D9_OSD_DRAW_DATA
#define FPV_VIDEO_D3D9_OSD_DRAW_DATA fpv_osd_draw_data
#endif

/* The signal: one pixel of the feed's raster per fragment. Compiled once per
 * mode (MODE 0 lens only, 1 analog, 2 digital), so each fits ps_3_0's slots.
 * The GLSL's conventions are kept (picture coordinates 0..1 with y up); y is
 * flipped only where a texture is read at picture coordinates (S()). */
static const char *fpv_video_d3d9__signal_ps = R"hlsl(
sampler2D scene : register(s0);
sampler2D osd : register(s1);
sampler2D lines : register(s2);
sampler2D history : register(s3);
sampler2D ui : register(s4);        // the game UI kept in the picture, premultiplied
float2 raster : register(c0);       // pixels of this target
float noise : register(c1);
float res : register(c2);
float osdIn : register(c3);
float picAspect : register(c4);
float sceneAspect : register(c5);
float2 k : register(c6);
float zoom : register(c7);
float lumaCycles : register(c8);
float chromaCycles : register(c9);
float subcarrier : register(c10);
float sharpen : register(c11);
float parity : register(c12);
float roll : register(c13);
float seed : register(c14);
float activeLines : register(c15);
float totalLines : register(c16);
float4 interf : register(c17);
float2 digital : register(c18);
float blocks : register(c19);
float softness : register(c20);
float lineRows : register(c21);     // rows of the lines texture
float4 uiRect : register(c22);      // the picture in the UI's screen: x, y, width, height (0: no UI)
float osdLines : register(c23);     // rows of the OSD chip's picture
float4 upset : register(c24);       // chroma boost, tint I, Q, hue sweep (turns)

static const float PI = 3.14159265;
// Dave Hoskins' hash without sine: the same on every GPU
float h13(float3 p) { p = frac(p * 0.1031); p += dot(p, p.zyx + 31.32); return frac((p.x + p.y) * p.z); }
float gn(float3 p) { return (h13(p) + h13(p + float3(17.1, 3.7, 9.3)) - 1.0) * 2.45; }
float glmod(float x, float y) { return x - y * floor(x / y); }
// a texture at picture coordinates (y up), as GL's texture()
float4 S(sampler2D s, float2 p) { return tex2Dlod(s, float4(p.x, 1.0 - p.y, 0.0, 0.0)); }
// picture (0..1, y up) to the scene texture through the lens
float2 sceneUv(float2 p) {
  float2 scale = float2(picAspect, 1.0) / (0.5 * sqrt(picAspect * picAspect + 1.0));
  float2 c = (p - 0.5) * scale; float r2 = dot(c, c);
  float2 d = c * (1.0 + k.x * r2 + k.y * r2 * r2) / zoom / scale;
  float s = picAspect / sceneAspect;
  if (s <= 1.0) d.x *= s; else d.y /= s;
  return d + 0.5;
}
float3 lensTap(float2 p) {
  float2 s = sceneUv(p);
  float3 c = S(scene, s).rgb;
  return (s.x < 0.0 || s.x > 1.0 || s.y < 0.0 || s.y > 1.0) ? float3(0.0, 0.0, 0.0) : c;
}
// the kept game UI over the lens's picture, not through the lens
float3 withUi(float3 c, float2 p) {
  [branch] if (uiRect.z > 0.0) {
    float4 u = S(ui, uiRect.xy + p * uiRect.zw);
    c = c * (1.0 - u.a) + u.rgb;
  }
  return c;
}
// the camera's picture
float3 tap(float2 p) { return withUi(lensTap(p), p); }
// The OSD chip's picture (360 font pixels by osdLines, premultiplied, point sampled):
// two samples a font pixel across the line (30 cells of 24 samples, as the chip writes
// them), centred down the field, one field line per glyph row. x 0..1 across the line, line
// from the top. Over a picture of another aspect (resolution off) it stays 4:3.
float4 osdTexel(float i, float ln) {
  return (i < 0.0 || i >= 360.0) ? float4(0.0, 0.0, 0.0, 0.0)
                                 : tex2Dlod(osd, float4((i + 0.5) / 360.0, (ln + 0.5) / osdLines, 0.0, 0.0));
}
float4 osdChip(float x, float fieldLine) {
  if (res < 0.5) x = (x - 0.5) * picAspect / (4.0 / 3.0) + 0.5;
  float ln = fieldLine - floor((activeLines - osdLines) * 0.5);
  if (osdIn < 0.5 || ln < 0.0 || ln >= osdLines) return float4(0.0, 0.0, 0.0, 0.0);
  return osdTexel(floor(x * 360.0), ln);
}
float4 osdTap(float2 p) { return osdChip(p.x, floor((1.0 - p.y) * activeLines)); }
float luma(float3 c) { return dot(c, float3(0.299, 0.587, 0.114)); }
float3 toYiq(float3 c) {
  return float3(luma(c), dot(c, float3(0.596, -0.274, -0.322)), dot(c, float3(0.211, -0.523, 0.312)));
}
float3 toRgb(float3 y) {
  return float3(dot(y, float3(1.0, 0.956, 0.621)), dot(y, float3(1.0, -0.272, -0.647)), dot(y, float3(1.0, -1.106, 1.703)));
}

#if MODE == 1
float3 analog(float2 uv) {
  float2 px = floor(uv * raster);
  // the field line this pixel shows (from the top) and, when the picture
  // rolls, the transmitted line it carries
  float lineF = res > 0.5 ? raster.y - 1.0 - px.y : (1.0 - uv.y) * activeLines;
  float rolled = glmod(lineF + roll * totalLines, totalLines);
  float ln = floor(rolled);
  float4 t0 = float4(0.0, 0.0, 1.0, 1.0), t1 = float4(0.5, 0.5, 0.5, 0.5);
  if (noise > 0.5) {
    float v = (ln + 0.5) / lineRows;
    t0 = tex2Dlod(lines, float4(0.25, v, 0.0, 0.0));
    t1 = tex2Dlod(lines, float4(0.75, v, 0.0, 0.0));
  }
  float shift = noise > 0.5 ? (floor(t1.r * 255.0 + 0.5) * 256.0 + floor(t1.g * 255.0 + 0.5)) / 65535.0 * 0.5 - 0.25 : 0.0;
  bool blank = rolled >= activeLines;  // the vertical blanking: a black bar
  // where the line sits in the camera's picture: interlaced, a field carries every other line
  float yTop = res > 0.5 ? (2.0 * ln + parity + 0.5) / (2.0 * activeLines) : rolled / activeLines;
  float x = (res > 0.5 ? (px.x + 0.5) / raster.x : uv.x) + shift;
  // a line torn past its end shows the black porch, then the next line's start
  bool porch = false;
  if (x > 1.0) { x -= 1.035; porch = x < 0.0; } else if (x < 0.0) { x += 1.035; porch = x > 1.0; }
  float2 p = float2(x, 1.0 - yTop);
  float3 yiq;
  float ph = 2.0 * PI * (x * subcarrier + 0.5 * ln + 0.5 * parity);
  float2 carrier = float2(cos(ph), sin(ph));
  [branch] if (res > 0.5) {
    // the camera and the link: luma limited to lumaCycles (about -6 dB there), colour
    // to chromaCycles, colour a little late; a quarter line up and down against vertical aliasing
    float sl = 0.25 / lumaCycles, sc = 0.35 / chromaCycles, dv = 0.25 / (2.0 * activeLines);
    float Y = 0.0, Yring = 0.0, wsum = 0.0; float3 wide = float3(0.0, 0.0, 0.0); float wcs = 0.0;
    [loop] for (int i = -3; i <= 3; i++) {
      float fi = float(i); float w = exp(-0.5 * (fi * 0.75) * (fi * 0.75));
      float2 q = p + float2(fi * 0.75 * sl, frac(abs(fi) * 0.5) > 0.25 ? dv : -dv);
      float y = luma(tap(q));
      Y += w * y; wsum += w; if (abs(fi) >= 2.0) Yring += y * 0.25;
      wide += w * tap(p + float2(fi * 0.75 * sc + 0.3 * sc, 0.0)); wcs += w;
    }
    Y /= wsum; wide /= wcs;
    // the camera's edge enhancement and the goggles' peaking: crisp edges with halos
    Y += (0.6 + sharpen * 1.6) * (Y - Yring);
    yiq = float3(Y, toYiq(wide).yz);
    // composite: fine detail near the subcarrier decodes as colour (rainbows)
    float hp = 0.5 / subcarrier;
    float yc = luma(tap(p));
    float yhf = yc - 0.5 * (luma(tap(p + float2(hp, 0.0))) + luma(tap(p - float2(hp, 0.0))));
    yiq.yz += 0.45 * yhf * carrier;
  } else {
    yiq = toYiq(tap(p));
  }
  // The OSD chip switches the video to its black or white level after the camera: sharp
  // pixels, a little softened by the link, its colour gone, and faint colour dots where
  // its steps ring in the decoder's colour filter.
  float osdA = 0.0;
  [branch] if (osdIn > 0.5 && !blank) {
    float s1 = 1.0 / 720.0;
    float4 oc = osdChip(x, ln), ol = osdChip(x - s1, ln), orr = osdChip(x + s1, ln);
    float4 ol2 = osdChip(x - 2.0 * s1, ln), or2 = osdChip(x + 2.0 * s1, ln);
    float a = 0.7 * oc.a + 0.15 * (ol.a + orr.a);
    float oy = 0.7 * oc.r + 0.15 * (ol.r + orr.r);
    float y0 = yiq.x;
    yiq.x = yiq.x * (1.0 - a) + oy;
    yiq.yz *= 1.0 - a;
    osdA = a;
    // NTSC cross-colour: the decoder's notch can't tell the OSD's one-pixel strokes (near
    // the subcarrier's period) from colour, and demodulates them into a solid hue over a
    // glyph, set by where its strokes sit against the subcarrier; the subcarrier turns half
    // a cycle a field, so the next field shows the complementary hue (a green 4, a magenta
    // mA). Wide strokes stay white.
    float2 cx = float2(0.0, 0.0); float cw = 1.0;
    [branch] if (oc.a + ol.a + orr.a + ol2.a + or2.a > 0.0) { cw = 0.0; [loop] for (int k = -4; k <= 4; k++) {
      float fk = float(k), xs = x + fk * s1;
      float4 o = osdChip(xs, ln);
      float w = exp(-0.08 * fk * fk), phk = 2.0 * PI * (xs * subcarrier + 0.5 * parity);
      cx += w * (o.r + (1.0 - o.a) * y0) * float2(cos(phk), sin(phk)); cw += w;
    } }
    yiq.yz += 0.6 * cx / cw;
  }
  // the black porch where a torn line starts, the blanking bar of a rolling picture:
  // black while there is a signal, the receiver's snow when there is none
  if (porch) yiq = float3(0.0, 0.0, 0.0);
  [branch] if (noise > 0.5) {
    float grain = t0.r * 0.5, density = t0.g * t0.g * t0.g, chroma = t0.b, pic = t0.a;
    float hue = (t1.a - 0.5) * 2.0 * PI + 2.0 * PI * upset.w * x;
    float3 sd = float3(0.0, ln, seed);
    // grain: the link's bandwidth stretches it along the line
    float gc = x * lumaCycles * 2.0; float gi = floor(gc), gf = gc - gi;
    float g = lerp(gn(sd + float3(gi, 0.0, 0.0)), gn(sd + float3(gi + 1.0, 0.0, 0.0)), gf);
    float cc = x * 360.0; float ci = floor(cc), cf = cc - ci;
    float2 gcol = lerp(float2(gn(sd + float3(ci, 51.0, 0.0)), gn(sd + float3(ci, 77.0, 0.0))),
                       float2(gn(sd + float3(ci + 1.0, 51.0, 0.0)), gn(sd + float3(ci + 1.0, 77.0, 0.0))), cf);
    // no picture: the receiver shows its own noise, coarse grey streaks
    float sc2 = x * 260.0; float si = floor(sc2), sf = sc2 - si;
    float snow = 0.42 + 0.30 * lerp(gn(sd + float3(si, 9.0, 1.0)), gn(sd + float3(si + 1.0, 9.0, 1.0)), sf) + 0.12 * g;
    if (blank) { yiq = float3(0.01, 0.0, 0.0); chroma = 0.0; }
    yiq.x = lerp(snow, yiq.x, pic);
    yiq.yz *= pic;
    // colour: the burst's phase (hue), the colour killer, and the decoder thrown off
    // by a hit (oversaturated, the hue sweeping across, the highlights tinted)
    float cs = cos(hue), sn = sin(hue);
    yiq.yz = float2(cs * yiq.y - sn * yiq.z, sn * yiq.y + cs * yiq.z) * chroma * (1.0 + upset.x);
    yiq.yz += upset.yz * saturate(yiq.x) * chroma;
    yiq.x += grain * g;
    // FM noise is strongest high up the spectrum, where the colour lives: saturated
    // colour speckle while the burst holds; the colour killer leaves grey snow
    yiq.yz += grain * 1.0 * gcol * chroma;
    // over the OSD's white strokes the noise turns the colour much harder (thin bright
    // strokes put their energy where the colour lives): glyphs come out tinted in patches
    [branch] if (osdA > 0.0) {
      float wc = x * 90.0; float wi = floor(wc), wf = wc - wi; float3 wd = float3(0.0, floor(ln * 0.25), seed);
      float2 wash = lerp(float2(gn(wd + float3(wi, 23.0, 0.0)), gn(wd + float3(wi, 41.0, 0.0))),
                         float2(gn(wd + float3(wi + 1.0, 23.0, 0.0)), gn(wd + float3(wi + 1.0, 41.0, 0.0))), wf);
      yiq.yz += osdA * saturate(yiq.x) * (grain * 3.0 * wash + grain * 2.0 * gcol) * chroma;
    }
    // FM clicks: specks a sample or two long, as often coloured as dark: black or a
    // colour on bright, white on dark. The whites click first (the carrier swings
    // furthest there), the darks hardly. Goggles show them in dense bands; the odd
    // lone click of a fair line doesn't show, so a sparse line keeps few of them.
    [branch] if (density > 0.0) {
      float lum = saturate(yiq.x);
      float dens = density * (0.15 + 1.7 * lum * lum) * smoothstep(0.004, 0.04, density);
      float kc = x * 360.0; float k0 = floor(kc);
      [loop] for (int j = 0; j < 2; j++) {
        float cell = k0 - float(j);
        float3 q = sd + float3(cell, 0.0, 33.0);
        [branch] if (h13(q) < dens) {
          float dist = kc - cell - h13(q + 1.3) * 0.5;
          float len = 0.35 + 0.6 * h13(q + 2.9);
          float v = dist < 0.0 || dist > len ? 0.0 : 1.0;
          float pol = h13(q + 4.7) < 0.85 ? (lum < 0.4 ? 1.0 : -1.0) : (h13(q + 6.1) < 0.5 ? 1.0 : -1.0);
          float hueK = 2.0 * PI * h13(q + 7.7);
          float sat = h13(q + 8.3);
          yiq.x += pol * v * (1.0 - 0.6 * sat) * 0.9;
          yiq.yz += float2(cos(hueK), sin(hueK)) * v * sat * 0.6 * max(chroma, 0.2);
        }
      }
    }
    // motor and ESC noise on the power wiring, and the line's own offset
    yiq.x += interf.x * sin(2.0 * PI * (x * interf.y + ln * interf.z + interf.w));
    yiq.x += t1.b - 0.5;
  }
  return saturate(toRgb(yiq));
}
#endif

#if MODE == 2
float3 boxTap(float2 p, float2 size) {
  float2 o = 0.25 * size;
  // the UI once, at the middle: its four taps would overflow ps_3_0's 1024 slots
  return withUi(0.25 * (lensTap(p + float2(-o.x, -o.y)) + lensTap(p + float2(o.x, -o.y)) +
                        lensTap(p + float2(-o.x, o.y)) + lensTap(p + float2(o.x, o.y))), p);
}
float3 digitalPix(float2 uv) {
  float2 p = uv;
  float3 c = res > 0.5 ? boxTap(p, 1.0 / raster) : tap(p);
  float2 bsize = 16.0 / digital;
  float2 b = floor(p / bsize);
  [branch] if (softness > 0.0) {  // the bitrate drops: detail goes first, in blocks
    float2 bc = (b + 0.5) * bsize;
    float3 dc = boxTap(bc, bsize);
    float3 mid = boxTap(p, bsize * 0.5);
    c = lerp(c, lerp(mid, dc, 0.35 * softness), softness);
  }
  [branch] if (noise > 0.5 && blocks > 0.0) {
    float3 q = float3(b, seed);
    // a lost packet takes a run of blocks along the row, then the decoder resyncs
    float runStart = floor(b.x / 6.0);
    float run = h13(float3(runStart, b.y, seed + 3.1));
    float runLen = 1.0 + floor(h13(float3(runStart, b.y, seed + 9.7)) * 6.0);
    bool inRun = run < blocks * 0.7 && b.x - runStart * 6.0 < runLen;
    [branch] if (inRun || h13(q) < blocks * 0.3) {
      float kind = h13(q + 5.5);
      [branch] if (kind < 0.6) {  // stale: last frame's block, moved by a broken motion vector
        float2 mv = (float2(h13(q + 1.1), h13(q + 2.2)) - 0.5) * bsize * 3.0;
        c = S(history, p + mv).rgb;
      } else if (kind < 0.9) {  // only the DC: a flat block, colour off
        float3 dc = boxTap((b + 0.5) * bsize, bsize);
        float3 y = toYiq(dc);
        y.yz += (float2(h13(float3(runStart, b.y, seed + 3.3)), h13(float3(runStart, b.y, seed + 4.4))) - 0.5) * 0.18;
        float2 f = frac(p / bsize) * 2.0 - 1.0;
        y.x += 0.08 * (h13(q + 6.6) - 0.5) * (f.x + f.y);
        c = toRgb(y);
      } else {  // garbage coefficients: stripes
        float2 f = frac(p / bsize);
        float s = cos(PI * floor(h13(q + 7.7) * 8.0) * f.x) * cos(PI * floor(h13(q + 8.8) * 8.0) * f.y);
        c = saturate(boxTap((b + 0.5) * bsize, bsize) + 0.25 * s * float3(0.8, 1.0, 0.7));
      }
    }
  }
  return saturate(c);
}
#endif

float4 main(float2 vpos : VPOS) : COLOR {
  // GL's uv: pixel centres, y up
  float2 uv = float2((vpos.x + 0.5) / raster.x, 1.0 - (vpos.y + 0.5) / raster.y);
#if MODE == 1
  float3 c = analog(uv);
#elif MODE == 2
  float3 c = digitalPix(uv);
#else
  float3 c = tap(uv);
  if (osdIn > 0.5) { float4 od = osdTap(uv); c = c * (1.0 - od.a) + od.rgb; }
#endif
  return float4(c, 1.0);
}
)hlsl";

/* The goggles: the signal scaled into the picture, black around it. */
static const char *fpv_video_d3d9__present_ps = R"hlsl(
sampler2D signal : register(s0);
float4 rect : register(c0);   // the picture: x, y (from the bottom), width, height
float4 rows : register(c1);   // field lines, parity (0 lines = progressive), framebuffer height
float2 texSize : register(c2);
float4 main(float2 vpos : VPOS) : COLOR {
  float2 fc = float2(vpos.x + 0.5, rows.z - (vpos.y + 0.5));  // gl_FragCoord
  float2 p = (fc - rect.xy) / rect.zw;
  if (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0) return float4(0.0, 0.0, 0.0, 1.0);
  if (rows.x > 0.0) {
    // bob: a field's lines sit half a line lower when it carries the odd lines
    float fromTop = (1.0 - p.y) * rows.x - 0.5 * rows.y;
    p.y = 1.0 - fromTop / rows.x;
  }
  // the goggles' scaler: bilinear, but each step between two samples kept to about a
  // screen pixel, so the OSD's pixels stay crisp; the picture is soft anyway
  float2 t = p * texSize - 0.5; float2 i = floor(t);
  float2 f = saturate((t - i - 0.5) * max(rect.zw / texSize, float2(1.0, 1.0)) + 0.5);
  float2 q = (i + f + 0.5) / texSize;
  return float4(tex2Dlod(signal, float4(q.x, 1.0 - q.y, 0.0, 0.0)).rgb, 1.0);
}
)hlsl";

/* A full-screen quad given in clip space: ps_3_0 needs a vs_3_0. */
static const char *fpv_video_d3d9__vs = R"hlsl(
float4 main(float4 p : POSITION) : POSITION { return p; }
)hlsl";

typedef HRESULT(WINAPI *fpv_video_d3d9__compile_fn)(LPCVOID, SIZE_T, LPCSTR, const D3D_SHADER_MACRO *, ID3DInclude *,
                                                   LPCSTR, LPCSTR, UINT, UINT, ID3DBlob **, ID3DBlob **);

static int fpv_video_d3d9__compile1(fpv_video_d3d9__compile_fn fn, const char *src, const char *mode,
                                    const char *target, void **code, unsigned int *size) {
    D3D_SHADER_MACRO macros[] = {{"MODE", mode}, {NULL, NULL}};
    ID3DBlob *blob = NULL, *errors = NULL;
    HRESULT hr = fn(src, strlen(src), "fpv_video", macros, NULL, "main", target, D3DCOMPILE_OPTIMIZATION_LEVEL3, 0,
                    &blob, &errors);
    if (errors) {
        FPV_VIDEO_D3D9_LOG("fpv_video_d3d9: shader %s MODE=%s: %s", target, mode,
                           (const char *)errors->GetBufferPointer());
        errors->Release();
    }
    if (FAILED(hr) || !blob) return 0;
    *size = (unsigned int)blob->GetBufferSize();
    *code = malloc(*size);
    if (*code) memcpy(*code, blob->GetBufferPointer(), *size);
    blob->Release();
    return *code != NULL;
}

int fpv_video_d3d9_compile(fpv_video_d3d9 *v) {
    if (v->compiled) return v->compiled > 0;
    /* the compiler stays loaded until the process ends: unloading it from
       another thread than the one that loaded it is not worth the trouble */
    HMODULE dll = LoadLibraryA("d3dcompiler_47.dll");
    fpv_video_d3d9__compile_fn fn =
        dll ? (fpv_video_d3d9__compile_fn)(void *)GetProcAddress(dll, "D3DCompile") : NULL;
    if (!fn) {
        FPV_VIDEO_D3D9_LOG("fpv_video_d3d9: no d3dcompiler_47.dll: no video feed");
        v->compiled = -1;
        return 0;
    }
    int ok = fpv_video_d3d9__compile1(fn, fpv_video_d3d9__signal_ps, "0", "ps_3_0", &v->code[0], &v->code_size[0]) &&
             fpv_video_d3d9__compile1(fn, fpv_video_d3d9__signal_ps, "1", "ps_3_0", &v->code[1], &v->code_size[1]) &&
             fpv_video_d3d9__compile1(fn, fpv_video_d3d9__signal_ps, "2", "ps_3_0", &v->code[2], &v->code_size[2]) &&
             fpv_video_d3d9__compile1(fn, fpv_video_d3d9__present_ps, "0", "ps_3_0", &v->code[3], &v->code_size[3]) &&
             fpv_video_d3d9__compile1(fn, fpv_video_d3d9__vs, "0", "vs_3_0", &v->code[4], &v->code_size[4]);
    v->compiled = ok ? 1 : -1;
    return ok;
}

#define FPV_VIDEO_D3D9_LINE_ROWS 512 /* >= FPV_VIDEO_MAX_LINES */

template <class T> static void fpv_video_d3d9__drop(T *&p) {
    if (p) p->Release();
    p = NULL;
}

static int fpv_video_d3d9__create(fpv_video_d3d9 *v, IDirect3DDevice9 *dev) {
    D3DCAPS9 caps;
    memset(&caps, 0, sizeof caps);
    dev->GetDeviceCaps(&caps);
    if (caps.PixelShaderVersion < D3DPS_VERSION(3, 0) || caps.MaxPixelShader30InstructionSlots < 1024) {
        FPV_VIDEO_D3D9_LOG("fpv_video_d3d9: no ps_3_0 with 1024 slots (version %08lx, %lu slots): no video feed",
                           (unsigned long)caps.PixelShaderVersion, (unsigned long)caps.MaxPixelShader30InstructionSlots);
        return 0;
    }
    for (int i = 0; i < 4; i++)
        if (FAILED(dev->CreatePixelShader((const DWORD *)v->code[i], &v->ps[i]))) {
            FPV_VIDEO_D3D9_LOG("fpv_video_d3d9: pixel shader %d failed", i);
            return 0;
        }
    if (FAILED(dev->CreateVertexShader((const DWORD *)v->code[4], &v->vs))) return 0;
    const D3DVERTEXELEMENT9 elements[] = {{0, 0, D3DDECLTYPE_FLOAT4, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_POSITION, 0},
                                          D3DDECL_END()};
    if (FAILED(dev->CreateVertexDeclaration(elements, &v->decl))) return 0;
    if (FAILED(dev->CreateTexture(2, FPV_VIDEO_D3D9_LINE_ROWS, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &v->lines, NULL)))
        return 0;
    if (FAILED(dev->CreateStateBlock(D3DSBT_ALL, &v->state))) return 0;
    return 1;
}

static int fpv_video_d3d9__target(IDirect3DDevice9 *dev, IDirect3DTexture9 *&tex, int w, int h) {
    fpv_video_d3d9__drop(tex);
    return SUCCEEDED(dev->CreateTexture((UINT)w, (UINT)h, 1, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT,
                                        &tex, NULL));
}

static int fpv_video_d3d9__set_target(IDirect3DDevice9 *dev, IDirect3DTexture9 *tex) {
    IDirect3DSurface9 *s = NULL;
    if (FAILED(tex->GetSurfaceLevel(0, &s))) return 0;
    HRESULT hr = dev->SetRenderTarget(0, s); /* the viewport becomes all of it */
    s->Release();
    return SUCCEEDED(hr);
}

static void fpv_video_d3d9__sampler(IDirect3DDevice9 *dev, DWORD stage, IDirect3DBaseTexture9 *tex, int linear) {
    dev->SetTexture(stage, tex);
    DWORD f = linear ? D3DTEXF_LINEAR : D3DTEXF_POINT;
    dev->SetSamplerState(stage, D3DSAMP_MINFILTER, f);
    dev->SetSamplerState(stage, D3DSAMP_MAGFILTER, f);
    dev->SetSamplerState(stage, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
    dev->SetSamplerState(stage, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
    dev->SetSamplerState(stage, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
    dev->SetSamplerState(stage, D3DSAMP_SRGBTEXTURE, FALSE);
}

static void fpv_video_d3d9__quad(fpv_video_d3d9 *v, IDirect3DDevice9 *dev, IDirect3DPixelShader9 *ps) {
    static const float q[4][4] = {{-1, 1, 0, 1}, {1, 1, 0, 1}, {-1, -1, 0, 1}, {1, -1, 0, 1}};
    dev->SetVertexDeclaration(v->decl);
    dev->SetVertexShader(v->vs);
    dev->SetPixelShader(ps);
    dev->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, q, sizeof q[0]);
}

static void fpv_video_d3d9__upload_lines(fpv_video_d3d9 *v, const fpv_video_feed *vf) {
    D3DLOCKED_RECT lr;
    if (FAILED(v->lines->LockRect(0, &lr, NULL, 0))) return;
    UINT rows = vf->line_count < FPV_VIDEO_MAX_LINES ? vf->line_count : FPV_VIDEO_MAX_LINES;
    for (UINT y = 0; y < rows; y++) {
        const uint8_t *src = vf->lines + y * 8;
        uint8_t *dst = (uint8_t *)lr.pBits + (size_t)y * lr.Pitch;
        for (int t = 0; t < 2; t++, src += 4, dst += 4) dst[0] = src[2], dst[1] = src[1], dst[2] = src[0], dst[3] = src[3];
    }
    v->lines->UnlockRect(0);
}

/* The OSD chip's picture (MAX7456 / AT7456E): it writes Betaflight's grid
 * of 12 x 18 font pixels straight into the video, a font pixel two of the
 * decoder's 720 samples wide and one field line tall, so 360 x (rows x 18)
 * here (black, white or clear). NTSC's 240 field lines show 13 rows (234
 * lines, Betaflight keeps its elements within them), PAL's 288 all 16. */
static int fpv_video_d3d9__osd_rows(int active_lines) { return active_lines >= 288 ? FPV_OSD_ROWS : 13; }
static int fpv_video_d3d9__osd_chip(fpv_video_d3d9 *v, IDirect3DDevice9 *dev, const fpv_frame *f, int rows) {
    const int per = 18, skip = 0, lines = rows * per;
    if (v->osd && v->osd_lines == lines && !memcmp(v->osd_grid, f->osd, sizeof v->osd_grid)) return 1;
    fpv_texture font;
    if (FPV_VIDEO_D3D9_TEXTURE_GET(FPV_TEXTURE_OSD_FONT, &font) != FPV_OK || !font.rgba) return 0;
    if (!v->osd || v->osd_lines != lines) {
        fpv_video_d3d9__drop(v->osd);
        if (FAILED(dev->CreateTexture(360, (UINT)lines, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &v->osd, NULL))) return 0;
    }
    D3DLOCKED_RECT lr;
    if (FAILED(v->osd->LockRect(0, &lr, NULL, 0))) return 0;
    for (int y = 0; y < lines; y++) memset((uint8_t *)lr.pBits + (size_t)y * lr.Pitch, 0, 360 * 4);
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < FPV_OSD_COLS; c++) {
            unsigned g = f->osd[r * FPV_OSD_COLS + c];
            if (g == 0 || g == ' ') continue;
            for (int y = 0; y < per; y++) {
                const uint8_t *src = font.rgba + (((size_t)(g / 16) * 18 + y + skip) * font.width + (g % 16) * 12) * 4;
                uint8_t *dst = (uint8_t *)lr.pBits + (size_t)(r * per + y) * lr.Pitch + (size_t)c * 12 * 4;
                for (int x = 0; x < 12; x++, src += 4, dst += 4) dst[0] = src[2], dst[1] = src[1], dst[2] = src[0], dst[3] = src[3];
            }
        }
    v->osd->UnlockRect(0);
    memcpy(v->osd_grid, f->osd, sizeof v->osd_grid);
    v->osd_lines = lines;
    return 1;
}

uint32_t fpv_video_d3d9_flags(const fpv_frame *f) {
    /* an older libfpv fills no video_feed: struct_size tells */
    if (f->struct_size < offsetof(fpv_frame, video_feed) + offsetof(fpv_video_feed, chroma_boost)) return 0;
    return f->video_feed.flags;
}

int fpv_video_d3d9_usable(const fpv_video_d3d9 *v) { return v->compiled > 0 && (!v->tried || v->ok); }

int fpv_video_d3d9_draw(fpv_video_d3d9 *v, IDirect3DDevice9 *dev, const fpv_frame *f, IDirect3DTexture9 *scene,
                        unsigned int scene_w, unsigned int scene_h, IDirect3DTexture9 *ui, unsigned int fb_w,
                        unsigned int fb_h, fpv_video_d3d9_osd_fn draw_osd, void *user) {
    if (v->compiled <= 0 || !scene || !fb_w || !fb_h || !scene_w || !scene_h) return 0;
    if (!v->tried) {
        v->tried = 1;
        v->ok = fpv_video_d3d9__create(v, dev);
    }
    if (!v->ok) return 0;
    const fpv_video_feed *vf = &f->video_feed;
    uint32_t fl = fpv_video_d3d9_flags(f);
    int has_upset = f->struct_size >= offsetof(fpv_frame, video_feed) + offsetof(fpv_video_feed, hue_sweep) + sizeof(float);
    int noise = (fl & FPV_VIDEO_FEED_NOISE) != 0, res = (fl & FPV_VIDEO_FEED_RESOLUTION) != 0;
    int mode = !fl ? 0 : vf->system != FPV_VIDEO_DIGITAL ? 1 : 2;
    int osd_in = draw_osd && mode == 1 && (fl & FPV_VIDEO_FEED_OSD_IN_SIGNAL);
    float active = vf->system == FPV_VIDEO_PAL ? 288.0f : 240.0f;
    float total = vf->system == FPV_VIDEO_PAL ? 313.0f : 263.0f;

    /* the picture on the screen: the camera's aspect with black bars, or all of it */
    float rx = 0, ry = 0, rw = (float)fb_w, rh = (float)fb_h;
    float pic_aspect = rw / rh;
    if (res && vf->picture_aspect > 0.1f) {
        pic_aspect = vf->picture_aspect;
        if (rw / rh > pic_aspect) rw = rh * pic_aspect, rx = ((float)fb_w - rw) * 0.5f;
        else rh = rw / pic_aspect, ry = ((float)fb_h - rh) * 0.5f;
    }
    /* the signal's raster: an analog field, a digital frame, or the screen's pixels */
    int w = (int)rw, h = (int)rh;
    if (res && mode == 1) w = (int)vf->raster_width, h = (int)active;
    else if (res && mode == 2) w = (int)vf->raster_width, h = (int)vf->raster_height;
    if (w < 1) w = 1;
    if (h < 1) h = 1;
    if (w > 4096) w = 4096;
    if (h > 4096) h = 4096;
    if (w != v->raster_w || h != v->raster_h || !v->tex[0] || !v->tex[1]) {
        if (!fpv_video_d3d9__target(dev, v->tex[0], w, h) || !fpv_video_d3d9__target(dev, v->tex[1], w, h)) {
            fpv_video_d3d9__drop(v->tex[0]), fpv_video_d3d9__drop(v->tex[1]);
            v->raster_w = v->raster_h = 0;
            return 0;
        }
        v->raster_w = w, v->raster_h = h;
        v->have = 0;
    }
    if (osd_in && !fpv_video_d3d9__osd_chip(v, dev, f, fpv_video_d3d9__osd_rows((int)active))) osd_in = 0;

    /* save state: the state block, the render target and the depth surface */
    IDirect3DSurface9 *rt = NULL, *depth = NULL;
    dev->GetRenderTarget(0, &rt);
    dev->GetDepthStencilSurface(&depth);
    v->state->Capture();
    dev->SetDepthStencilSurface(NULL); /* the targets may be larger than the game's depth */
    dev->SetRenderState(D3DRS_ZENABLE, FALSE);
    dev->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    dev->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    dev->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
    dev->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
    dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    dev->SetRenderState(D3DRS_STENCILENABLE, FALSE);
    dev->SetRenderState(D3DRS_FOGENABLE, FALSE);
    dev->SetRenderState(D3DRS_SRGBWRITEENABLE, FALSE);
    dev->SetRenderState(D3DRS_COLORWRITEENABLE, 0xf);

    /* a new field (frame), a change of settings, or no effects at all: draw the signal */
    int fresh = !v->have || !fl || vf->field != v->field || fl != v->flags;
    int hold = mode == 2 && noise && vf->frozen && v->have; /* the goggles hold the last frame */
    int ok = 1;
    if (fresh && !hold) {
        if (noise && mode == 1) fpv_video_d3d9__upload_lines(v, vf);
        int next = v->have ? 1 - v->cur : v->cur;
        ok = fpv_video_d3d9__set_target(dev, v->tex[next]);
        if (ok) {
            fpv_video_d3d9__sampler(dev, 0, scene, 1);
            fpv_video_d3d9__sampler(dev, 1, osd_in ? v->osd : NULL, 0);
            fpv_video_d3d9__sampler(dev, 2, v->lines, 0);
            fpv_video_d3d9__sampler(dev, 3, v->tex[1 - next], 1); /* the last frame (never the target itself) */
            fpv_video_d3d9__sampler(dev, 4, ui, 1);
            float c[25][4];
            memset(c, 0, sizeof c);
            c[0][0] = (float)w, c[0][1] = (float)h;
            c[1][0] = (float)noise;
            c[2][0] = (float)res;
            c[3][0] = (float)osd_in;
            c[4][0] = pic_aspect;
            c[5][0] = (float)scene_w / (float)scene_h;
            c[6][0] = f->camera.distortion_k1, c[6][1] = f->camera.distortion_k2;
            c[7][0] = f->camera.distortion_zoom > 0 ? f->camera.distortion_zoom : 1.0f;
            c[8][0] = vf->luma_cycles > 1 ? vf->luma_cycles : 250.0f;
            c[9][0] = vf->chroma_cycles > 1 ? vf->chroma_cycles : 53.0f;
            c[10][0] = vf->subcarrier_cycles > 1 ? vf->subcarrier_cycles : 188.5f;
            c[11][0] = vf->sharpening;
            c[12][0] = (float)(vf->field_parity & 1);
            c[13][0] = vf->roll;
            c[14][0] = vf->seed;
            c[15][0] = active;
            c[16][0] = total;
            for (int i = 0; i < 4; i++) c[17][i] = vf->interference[i];
            c[18][0] = vf->raster_width ? (float)vf->raster_width : (float)w;
            c[18][1] = vf->raster_height ? (float)vf->raster_height : (float)h;
            c[19][0] = vf->block_errors;
            c[20][0] = noise ? vf->softness : 0.0f;
            c[21][0] = (float)FPV_VIDEO_D3D9_LINE_ROWS;
            if (ui) { /* the picture's place on the screen, where the game drew its UI */
                c[22][0] = rx / (float)fb_w, c[22][1] = ry / (float)fb_h;
                c[22][2] = rw / (float)fb_w, c[22][3] = rh / (float)fb_h;
            }
            c[23][0] = (float)(18 * fpv_video_d3d9__osd_rows((int)active));
            if (has_upset && noise)
                c[24][0] = vf->chroma_boost, c[24][1] = vf->chroma_tint[0], c[24][2] = vf->chroma_tint[1],
                c[24][3] = vf->hue_sweep;
            dev->SetPixelShaderConstantF(0, &c[0][0], 25);
            fpv_video_d3d9__quad(v, dev, v->ps[mode]);
            v->cur = next;
            v->have = 1;
            v->field = vf->field;
            v->flags = fl;
        }
    }

    /* the goggles */
    if (ok) {
        dev->SetRenderTarget(0, rt);
        fpv_video_d3d9__sampler(dev, 0, v->tex[v->cur], 1);
        for (DWORD s = 1; s < 5; s++) dev->SetTexture(s, NULL);
        float c[3][4] = {{rx, ry, rw, rh},
                         {res && mode == 1 ? active : 0.0f, (float)(vf->field_parity & 1), (float)fb_h, 0},
                         {(float)w, (float)h, 0, 0}};
        dev->SetPixelShaderConstantF(0, &c[0][0], 3);
        fpv_video_d3d9__quad(v, dev, v->ps[3]);
    }

    /* restore state (the render target first: setting it resets the viewport) */
    dev->SetRenderTarget(0, rt);
    dev->SetDepthStencilSurface(depth);
    v->state->Apply();
    if (rt) rt->Release();
    if (depth) depth->Release();
    if (!ok) return 0;

    /* the OSD over the picture (digital: the goggles draw it; or effects off) */
    if (draw_osd && !osd_in) {
        fpv_osd_style style = fl ? (fpv_osd_style)vf->osd_style : FPV_OSD_STYLE_ANALOG;
        const fpv_draw_data *dd = FPV_VIDEO_D3D9_OSD_DRAW_DATA(f, (float)fb_w, (float)fb_h, style);
        if (dd) draw_osd(user, dev, dd, (float)fb_w, (float)fb_h);
    }
    return 1;
}

void fpv_video_d3d9_release(fpv_video_d3d9 *v) {
    for (int i = 0; i < 4; i++) fpv_video_d3d9__drop(v->ps[i]);
    fpv_video_d3d9__drop(v->vs);
    fpv_video_d3d9__drop(v->decl);
    fpv_video_d3d9__drop(v->state);
    fpv_video_d3d9__drop(v->tex[0]);
    fpv_video_d3d9__drop(v->tex[1]);
    fpv_video_d3d9__drop(v->osd);
    fpv_video_d3d9__drop(v->lines);
    v->tried = v->ok = 0;
    v->cur = v->raster_w = v->raster_h = v->osd_lines = 0;
    v->field = v->flags = 0;
    v->have = 0;
}

void fpv_video_d3d9_shutdown(fpv_video_d3d9 *v) {
    fpv_video_d3d9_release(v);
    for (int i = 0; i < 5; i++) {
        free(v->code[i]);
        v->code[i] = NULL;
        v->code_size[i] = 0;
    }
    v->compiled = 0;
}
#endif
