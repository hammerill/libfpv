// libfpv - Betaflight OSD rendering.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "osd/osd.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "util/embedded.h"
#include "util/log.h"

namespace fpv {

bool parseMcmFont(const char *text, size_t size, std::vector<uint8_t> &rgba, int atlasW, int xOffset) {
    std::vector<uint8_t> bytes;
    bytes.reserve(256 * 64);
    size_t i = 0;
    // skip header line
    while (i < size && text[i] != '\n') i++;
    while (i < size && bytes.size() < 256 * 64) {
        while (i < size && (text[i] == '\n' || text[i] == '\r' || text[i] == ' ')) i++;
        if (i + 8 > size) break;
        uint8_t b = 0;
        for (int k = 0; k < 8; k++) {
            char c = text[i + k];
            if (c != '0' && c != '1') return false;
            b = (uint8_t)((b << 1) | (c == '1'));
        }
        bytes.push_back(b);
        i += 8;
    }
    if (bytes.size() < 256 * 64) return false;
    for (int g = 0; g < 256; g++) {
        int gx = (g % 16) * OsdRenderer::kGlyphW + xOffset, gy = (g / 16) * OsdRenderer::kGlyphH;
        for (int p = 0; p < OsdRenderer::kGlyphW * OsdRenderer::kGlyphH; p++) {
            uint8_t byte = bytes[g * 64 + p / 4];
            int v = (byte >> (6 - 2 * (p % 4))) & 3;
            int px = gx + p % OsdRenderer::kGlyphW, py = gy + p / OsdRenderer::kGlyphW;
            uint8_t *o = &rgba[((size_t)py * atlasW + px) * 4];
            if (v == 0) {  // black (outline)
                o[0] = o[1] = o[2] = 0;
                o[3] = 255;
            } else if (v == 2) {  // white
                o[0] = o[1] = o[2] = 255;
                o[3] = 255;
            } else {  // transparent
                o[0] = o[1] = o[2] = o[3] = 0;
            }
        }
    }
    return true;
}

OsdRenderer::OsdRenderer() {
    m_atlas.assign((size_t)kAtlasW * kAtlasH * 4, 0);
    const char *names[kFonts] = {"betaflight-default.mcm", "betaflight-bold.mcm"};
    int loaded = 0;
    for (int f = 0; f < kFonts; f++) {
        for (size_t i = 0; i < kEmbeddedFontsCount; i++) {
            if (std::string(kEmbeddedFonts[i].path) == names[f] &&
                parseMcmFont((const char *)kEmbeddedFonts[i].data, kEmbeddedFonts[i].size, m_atlas, kAtlasW,
                             f * kGlyphW * 16))
                loaded++;
        }
    }
    m_ok = loaded == kFonts;
    if (!m_ok) FPV_LOGE("OSD font missing or corrupt");
}

void OsdRenderer::viewport(float width, float height, int style, float &x, float &y, float &w, float &h) {
    float aspect = style == FPV_OSD_STYLE_HD ? 16.0f / 9.0f : 4.0f / 3.0f;
    w = width;
    h = width / aspect;
    if (h > height) {
        h = height;
        w = height * aspect;
    }
    x = (width - w) * 0.5f;
    y = (height - h) * 0.5f;
}

static bool blank(uint8_t c) { return c == 0 || c == ' '; }

const fpv_draw_data *OsdRenderer::drawData(const uint8_t *osd, float width, float height, int style, int rows) {
    rows = std::clamp(rows, 1, FPV_OSD_ROWS);
    m_buffer.clear();
    float vx, vy, vw, vh;
    viewport(width, height, style, vx, vy, vw, vh);
    // Analog: the OSD chip's grid (13 rows of 18 of NTSC's 240 lines, 16 of
    // PAL's 288) scaled to 0.95 around the centre, as the analog renderers
    // draw it; HD uses a margin and bigger cells the way HD goggles render the
    // same 30x16 Betaflight canvas.
    float marginX = style == FPV_OSD_STYLE_HD ? vw * 0.06f : vw * 0.025f;
    float marginY = style == FPV_OSD_STYLE_HD ? vh * 0.04f
                    : vh * 0.5f * (1.0f - 0.95f * (rows < FPV_OSD_ROWS ? 234.0f / 240.0f : 1.0f));
    float cw = (vw - 2 * marginX) / FPV_OSD_COLS, ch = (vh - 2 * marginY) / rows;
    DrawListStorage &l = m_buffer.newList();
    int font = style == FPV_OSD_STYLE_HD ? 1 : 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < FPV_OSD_COLS; c++) {
            uint8_t g = osd[r * FPV_OSD_COLS + c];
            if (blank(g)) continue;
            float x0 = vx + marginX + c * cw, y0 = vy + marginY + r * ch;
            float u0 = (float)((g % 16) * kGlyphW + font * kGlyphW * 16) / kAtlasW;
            float v0 = (float)((g / 16) * kGlyphH) / kAtlasH;
            float u1 = u0 + (float)kGlyphW / kAtlasW, v1 = v0 + (float)kGlyphH / kAtlasH;
            uint32_t base = (uint32_t)l.vertices.size();
            const uint32_t white = 0xFFFFFFFFu;
            l.vertices.push_back({{x0, y0}, {u0, v0}, white});
            l.vertices.push_back({{x0 + cw, y0}, {u1, v0}, white});
            l.vertices.push_back({{x0 + cw, y0 + ch}, {u1, v1}, white});
            l.vertices.push_back({{x0, y0 + ch}, {u0, v1}, white});
            uint32_t q[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
            l.indices.insert(l.indices.end(), q, q + 6);
        }
    }
    if (!l.indices.empty()) {
        fpv_draw_cmd cmd{};
        cmd.clip_rect[0] = vx;
        cmd.clip_rect[1] = vy;
        cmd.clip_rect[2] = vx + vw;
        cmd.clip_rect[3] = vy + vh;
        cmd.texture_id = FPV_TEXTURE_OSD_FONT;
        cmd.index_count = (uint32_t)l.indices.size();
        l.commands.push_back(cmd);
    }
    return m_buffer.finish(width, height);
}

void OsdRenderer::rasterize(const uint8_t *osd, uint8_t *rgba, uint32_t width, uint32_t height, uint32_t stride,
                            int style, int rows) {
    for (uint32_t y = 0; y < height; y++) std::memset(rgba + (size_t)y * stride, 0, (size_t)width * 4);
    const fpv_draw_data *dd = drawData(osd, (float)width, (float)height, style, rows);
    if (!dd->list_count) return;
    const fpv_draw_list &l = dd->lists[0];
    // every glyph is an axis-aligned quad: 4 vertices
    for (uint32_t q = 0; q + 3 < l.vertex_count; q += 4) {
        const fpv_draw_vertex &a = l.vertices[q], &c = l.vertices[q + 2];
        int x0 = (int)std::floor(a.pos[0]), y0 = (int)std::floor(a.pos[1]);
        int x1 = (int)std::ceil(c.pos[0]), y1 = (int)std::ceil(c.pos[1]);
        for (int y = std::max(0, y0); y < std::min((int)height, y1); y++) {
            float tv = (y + 0.5f - a.pos[1]) / (c.pos[1] - a.pos[1]);
            int ay = (int)((a.uv[1] + (c.uv[1] - a.uv[1]) * tv) * kAtlasH);
            for (int x = std::max(0, x0); x < std::min((int)width, x1); x++) {
                float tu = (x + 0.5f - a.pos[0]) / (c.pos[0] - a.pos[0]);
                int ax = (int)((a.uv[0] + (c.uv[0] - a.uv[0]) * tu) * kAtlasW);
                ax = std::clamp(ax, 0, kAtlasW - 1);
                ay = std::clamp(ay, 0, kAtlasH - 1);
                const uint8_t *s = &m_atlas[((size_t)ay * kAtlasW + ax) * 4];
                if (s[3]) std::memcpy(rgba + (size_t)y * stride + (size_t)x * 4, s, 4);
            }
        }
    }
}

}  // namespace fpv
