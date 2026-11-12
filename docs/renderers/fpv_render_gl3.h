/*
 * fpv_render_gl3.h - draws libfpv overlay/OSD draw data with OpenGL 3.3 core
 * (or OpenGL ES 3.0 and WebGL 2 with FPV_GL3_GLES). Single header:
 *
 *   #include <your GL loader>            // glad, GLEW, gl3w, SDL_opengl...
 *   #define FPV_RENDER_GL3_IMPLEMENTATION
 *   #include "fpv_render_gl3.h"
 *
 *   fpv_gl3 r; fpv_gl3_init(&r);                       // once, GL context current
 *   fpv_gl3_render(&r, fpv_osd_draw_data(&frame, w, h, style), fb_w, fb_h);
 *   fpv_gl3_render(&r, fpv_overlay_render(dt), fb_w, fb_h);   // NULL is fine
 *   fpv_gl3_shutdown(&r);
 *
 * Saves and restores the GL state it changes, so it can be called from a
 * SwapBuffers hook in a game. SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef FPV_RENDER_GL3_H
#define FPV_RENDER_GL3_H

#include "libfpv.h"

typedef struct fpv_gl3 {
    unsigned int program, vao, vbo, ebo;
    int loc_proj, loc_tex;
    unsigned int textures[3];   /* index = FPV_TEXTURE_* id */
    uint32_t revisions[3];
} fpv_gl3;

int fpv_gl3_init(fpv_gl3 *r);
void fpv_gl3_render(fpv_gl3 *r, const fpv_draw_data *dd, int fb_width, int fb_height);
void fpv_gl3_shutdown(fpv_gl3 *r);

#endif

#if defined(FPV_RENDER_GL3_IMPLEMENTATION) && !defined(FPV_RENDER_GL3_IMPLEMENTED)
#define FPV_RENDER_GL3_IMPLEMENTED /* included again (fpv_video_gl3.h): one copy */
#include <string.h>

#ifdef FPV_GL3_GLES
#define FPV_GL3_VERSION "#version 300 es\nprecision mediump float;\n"
#else
#define FPV_GL3_VERSION "#version 330 core\n"
#endif

static unsigned int fpv_gl3__shader(unsigned int type, const char *src) {
    unsigned int s = glCreateShader(type);
    const char *srcs[2] = {FPV_GL3_VERSION, src};
    glShaderSource(s, 2, srcs, NULL);
    glCompileShader(s);
    return s;
}

/* Vertex layout, with the buffer starting `base` vertices in. */
static void fpv_gl3__attribs(size_t base) {
    size_t b = base * sizeof(fpv_draw_vertex);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(fpv_draw_vertex), (void *)(b + offsetof(fpv_draw_vertex, pos)));
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(fpv_draw_vertex), (void *)(b + offsetof(fpv_draw_vertex, uv)));
    glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(fpv_draw_vertex),
                          (void *)(b + offsetof(fpv_draw_vertex, color)));
}

int fpv_gl3_init(fpv_gl3 *r) {
    memset(r, 0, sizeof(*r));
    const char *vs =
        "layout(location=0) in vec2 pos; layout(location=1) in vec2 uv; layout(location=2) in vec4 col;\n"
        "uniform mat4 proj; out vec2 f_uv; out vec4 f_col;\n"
        "void main() { f_uv = uv; f_col = col; gl_Position = proj * vec4(pos, 0.0, 1.0); }\n";
    const char *fs =
        "in vec2 f_uv; in vec4 f_col; uniform sampler2D tex; out vec4 o;\n"
        "void main() { o = f_col * texture(tex, f_uv); }\n";
    unsigned int v = fpv_gl3__shader(GL_VERTEX_SHADER, vs), f = fpv_gl3__shader(GL_FRAGMENT_SHADER, fs);
    r->program = glCreateProgram();
    glAttachShader(r->program, v);
    glAttachShader(r->program, f);
    glLinkProgram(r->program);
    glDeleteShader(v);
    glDeleteShader(f);
    int ok = 0;
    glGetProgramiv(r->program, GL_LINK_STATUS, &ok);
    if (!ok) return 0;
    r->loc_proj = glGetUniformLocation(r->program, "proj");
    r->loc_tex = glGetUniformLocation(r->program, "tex");
    glGenVertexArrays(1, &r->vao);
    glGenBuffers(1, &r->vbo);
    glGenBuffers(1, &r->ebo);
    glBindVertexArray(r->vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r->ebo);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);
    fpv_gl3__attribs(0);
    glBindVertexArray(0);
    return 1;
}

/* Uploads (or re-uploads after a revision change) a libfpv texture. */
static unsigned int fpv_gl3__texture(fpv_gl3 *r, uint64_t id) {
    if (id >= 3) return 0;
    fpv_texture t;
    if (fpv_texture_get(id, &t) != FPV_OK) return 0;
    if (r->textures[id] && r->revisions[id] == t.revision) return r->textures[id];
    if (!r->textures[id]) glGenTextures(1, &r->textures[id]);
    glBindTexture(GL_TEXTURE_2D, r->textures[id]);
    /* the OSD font is pixel art: keep it crisp */
    int filter = id == FPV_TEXTURE_OSD_FONT ? GL_NEAREST : GL_LINEAR;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, (int)t.width, (int)t.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, t.rgba);
    r->revisions[id] = t.revision;
    return r->textures[id];
}

void fpv_gl3_render(fpv_gl3 *r, const fpv_draw_data *dd, int fbw, int fbh) {
    if (!dd || !dd->list_count || fbw <= 0 || fbh <= 0) return;
    /* save state */
    int last_program, last_vao, last_tex, last_active, last_viewport[4], last_scissor[4];
    int last_blend_src_rgb, last_blend_dst_rgb, last_blend_src_a, last_blend_dst_a;
    glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vao);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &last_active);
    glActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_tex);
    glGetIntegerv(GL_VIEWPORT, last_viewport);
    glGetIntegerv(GL_SCISSOR_BOX, last_scissor);
    glGetIntegerv(GL_BLEND_SRC_RGB, &last_blend_src_rgb);
    glGetIntegerv(GL_BLEND_DST_RGB, &last_blend_dst_rgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &last_blend_src_a);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &last_blend_dst_a);
    unsigned char was_blend = glIsEnabled(GL_BLEND), was_cull = glIsEnabled(GL_CULL_FACE);
    unsigned char was_depth = glIsEnabled(GL_DEPTH_TEST), was_scissor = glIsEnabled(GL_SCISSOR_TEST);

    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);
    glViewport(0, 0, fbw, fbh);
    float W = dd->display_size[0], H = dd->display_size[1];
    float sx = fbw / W, sy = fbh / H;
    const float proj[16] = {2.0f / W, 0, 0, 0, 0, -2.0f / H, 0, 0, 0, 0, -1, 0, -1, 1, 0, 1};
    glUseProgram(r->program);
    glUniformMatrix4fv(r->loc_proj, 1, GL_FALSE, proj);
    glUniform1i(r->loc_tex, 0);
    glBindVertexArray(r->vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r->ebo);
#ifdef FPV_GL3_GLES
    uint32_t base = 0; /* vertex offset the layout points at */
#endif
    for (uint32_t l = 0; l < dd->list_count; l++) {
        const fpv_draw_list *list = &dd->lists[l];
        glBufferData(GL_ARRAY_BUFFER, (long)(list->vertex_count * sizeof(fpv_draw_vertex)), list->vertices, GL_STREAM_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, (long)(list->index_count * sizeof(uint32_t)), list->indices, GL_STREAM_DRAW);
        for (uint32_t c = 0; c < list->command_count; c++) {
            const fpv_draw_cmd *cmd = &list->commands[c];
            int x1 = (int)(cmd->clip_rect[0] * sx), y1 = (int)(cmd->clip_rect[1] * sy);
            int x2 = (int)(cmd->clip_rect[2] * sx), y2 = (int)(cmd->clip_rect[3] * sy);
            if (x2 <= x1 || y2 <= y1) continue;
            glScissor(x1, fbh - y2, x2 - x1, y2 - y1); /* GL scissor origin is bottom-left */
            glBindTexture(GL_TEXTURE_2D, fpv_gl3__texture(r, cmd->texture_id));
#ifdef FPV_GL3_GLES
            /* ES 3.0 and WebGL 2 have no base vertex: move the vertex layout instead */
            if (cmd->vertex_offset != base) fpv_gl3__attribs(base = cmd->vertex_offset);
            glDrawElements(GL_TRIANGLES, (int)cmd->index_count, GL_UNSIGNED_INT,
                           (void *)(size_t)(cmd->index_offset * sizeof(uint32_t)));
#else
            glDrawElementsBaseVertex(GL_TRIANGLES, (int)cmd->index_count, GL_UNSIGNED_INT,
                                     (void *)(size_t)(cmd->index_offset * sizeof(uint32_t)), (int)cmd->vertex_offset);
#endif
        }
    }
#ifdef FPV_GL3_GLES
    if (base) fpv_gl3__attribs(0);
#endif
    /* restore state */
    glUseProgram((unsigned)last_program);
    glBindVertexArray((unsigned)last_vao);
    glBindTexture(GL_TEXTURE_2D, (unsigned)last_tex);
    glActiveTexture((unsigned)last_active);
    glViewport(last_viewport[0], last_viewport[1], last_viewport[2], last_viewport[3]);
    glScissor(last_scissor[0], last_scissor[1], last_scissor[2], last_scissor[3]);
    glBlendFuncSeparate((unsigned)last_blend_src_rgb, (unsigned)last_blend_dst_rgb, (unsigned)last_blend_src_a,
                        (unsigned)last_blend_dst_a);
    if (was_blend) glEnable(GL_BLEND); else glDisable(GL_BLEND);
    if (was_cull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    if (was_depth) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (was_scissor) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST);
}

void fpv_gl3_shutdown(fpv_gl3 *r) {
    for (int i = 0; i < 3; i++)
        if (r->textures[i]) glDeleteTextures(1, &r->textures[i]);
    glDeleteBuffers(1, &r->vbo);
    glDeleteBuffers(1, &r->ebo);
    glDeleteVertexArrays(1, &r->vao);
    glDeleteProgram(r->program);
    memset(r, 0, sizeof(*r));
}
#endif /* FPV_RENDER_GL3_IMPLEMENTATION */
