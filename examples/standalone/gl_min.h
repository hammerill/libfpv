/* Minimal OpenGL 3.3 core loader for the standalone example (loaded via
 * SDL_GL_GetProcAddress). A mod uses whatever loader the game already has. */
#ifndef GL_MIN_H
#define GL_MIN_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  define GLAPIENTRY __stdcall
#else
#  define GLAPIENTRY
#endif

typedef unsigned int GLenum, GLuint, GLbitfield;
typedef int GLint, GLsizei;
typedef unsigned char GLboolean;
typedef float GLfloat;
typedef char GLchar;
typedef ptrdiff_t GLsizeiptr, GLintptr;

#define GL_FALSE 0
#define GL_TRUE 1
#define GL_TRIANGLES 0x0004
#define GL_UNSIGNED_BYTE 0x1401
#define GL_UNSIGNED_INT 0x1405
#define GL_FLOAT 0x1406
#define GL_DEPTH_BUFFER_BIT 0x00000100
#define GL_COLOR_BUFFER_BIT 0x00004000
#define GL_DEPTH_TEST 0x0B71
#define GL_CULL_FACE 0x0B44
#define GL_BLEND 0x0BE2
#define GL_SCISSOR_TEST 0x0C11
#define GL_LESS 0x0201
#define GL_SRC_ALPHA 0x0302
#define GL_ONE_MINUS_SRC_ALPHA 0x0303
#define GL_ONE 1
#define GL_VIEWPORT 0x0BA2
#define GL_SCISSOR_BOX 0x0C10
#define GL_BLEND_DST_RGB 0x80C8
#define GL_BLEND_SRC_RGB 0x80C9
#define GL_BLEND_DST_ALPHA 0x80CA
#define GL_BLEND_SRC_ALPHA 0x80CB
#define GL_TEXTURE_2D 0x0DE1
#define GL_TEXTURE_BINDING_2D 0x8069
#define GL_TEXTURE0 0x84C0
#define GL_ACTIVE_TEXTURE 0x84E0
#define GL_TEXTURE_MAG_FILTER 0x2800
#define GL_TEXTURE_MIN_FILTER 0x2801
#define GL_TEXTURE_WRAP_S 0x2802
#define GL_TEXTURE_WRAP_T 0x2803
#define GL_NEAREST 0x2600
#define GL_LINEAR 0x2601
#define GL_CLAMP_TO_EDGE 0x812F
#define GL_RGBA 0x1908
#define GL_RGBA8 0x8058
#define GL_RGB 0x1907
#define GL_UNPACK_ALIGNMENT 0x0CF5
#define GL_PACK_ALIGNMENT 0x0D05
#define GL_ARRAY_BUFFER 0x8892
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#define GL_STATIC_DRAW 0x88E4
#define GL_STREAM_DRAW 0x88E0
#define GL_DYNAMIC_DRAW 0x88E8
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_VERTEX_SHADER 0x8B31
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82
#define GL_CURRENT_PROGRAM 0x8B8D
#define GL_VERTEX_ARRAY_BINDING 0x85B5
#define GL_FRAMEBUFFER 0x8D40
#define GL_RENDERBUFFER 0x8D41
#define GL_COLOR_ATTACHMENT0 0x8CE0
#define GL_DEPTH_ATTACHMENT 0x8D00
#define GL_DEPTH_COMPONENT24 0x81A6
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5

#define GL_MIN_FUNCS(X)                                                                                         \
    X(void, glClear, (GLbitfield))                                                                              \
    X(void, glClearColor, (GLfloat, GLfloat, GLfloat, GLfloat))                                                 \
    X(void, glEnable, (GLenum))                                                                                 \
    X(void, glDisable, (GLenum))                                                                                \
    X(GLboolean, glIsEnabled, (GLenum))                                                                         \
    X(void, glDepthFunc, (GLenum))                                                                              \
    X(void, glViewport, (GLint, GLint, GLsizei, GLsizei))                                                       \
    X(void, glScissor, (GLint, GLint, GLsizei, GLsizei))                                                        \
    X(void, glGetIntegerv, (GLenum, GLint *))                                                                   \
    X(void, glBlendFuncSeparate, (GLenum, GLenum, GLenum, GLenum))                                              \
    X(void, glPixelStorei, (GLenum, GLint))                                                                     \
    X(void, glReadPixels, (GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, void *))                             \
    X(void, glGenTextures, (GLsizei, GLuint *))                                                                 \
    X(void, glDeleteTextures, (GLsizei, const GLuint *))                                                        \
    X(void, glBindTexture, (GLenum, GLuint))                                                                    \
    X(void, glActiveTexture, (GLenum))                                                                          \
    X(void, glTexParameteri, (GLenum, GLenum, GLint))                                                           \
    X(void, glTexImage2D, (GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void *))        \
    X(GLuint, glCreateShader, (GLenum))                                                                         \
    X(void, glShaderSource, (GLuint, GLsizei, const GLchar *const *, const GLint *))                            \
    X(void, glCompileShader, (GLuint))                                                                          \
    X(void, glGetShaderiv, (GLuint, GLenum, GLint *))                                                           \
    X(void, glGetShaderInfoLog, (GLuint, GLsizei, GLsizei *, GLchar *))                                         \
    X(void, glDeleteShader, (GLuint))                                                                           \
    X(GLuint, glCreateProgram, (void))                                                                          \
    X(void, glAttachShader, (GLuint, GLuint))                                                                   \
    X(void, glLinkProgram, (GLuint))                                                                            \
    X(void, glGetProgramiv, (GLuint, GLenum, GLint *))                                                          \
    X(void, glGetProgramInfoLog, (GLuint, GLsizei, GLsizei *, GLchar *))                                        \
    X(void, glDeleteProgram, (GLuint))                                                                          \
    X(void, glUseProgram, (GLuint))                                                                             \
    X(GLint, glGetUniformLocation, (GLuint, const GLchar *))                                                    \
    X(void, glUniform1i, (GLint, GLint))                                                                        \
    X(void, glUniform1f, (GLint, GLfloat))                                                                      \
    X(void, glUniform2f, (GLint, GLfloat, GLfloat))                                                             \
    X(void, glUniform3f, (GLint, GLfloat, GLfloat, GLfloat))                                                    \
    X(void, glUniform4f, (GLint, GLfloat, GLfloat, GLfloat, GLfloat))                                           \
    X(void, glUniformMatrix4fv, (GLint, GLsizei, GLboolean, const GLfloat *))                                   \
    X(void, glGenVertexArrays, (GLsizei, GLuint *))                                                             \
    X(void, glDeleteVertexArrays, (GLsizei, const GLuint *))                                                    \
    X(void, glBindVertexArray, (GLuint))                                                                        \
    X(void, glGenBuffers, (GLsizei, GLuint *))                                                                  \
    X(void, glDeleteBuffers, (GLsizei, const GLuint *))                                                         \
    X(void, glBindBuffer, (GLenum, GLuint))                                                                     \
    X(void, glBufferData, (GLenum, GLsizeiptr, const void *, GLenum))                                           \
    X(void, glEnableVertexAttribArray, (GLuint))                                                                \
    X(void, glVertexAttribPointer, (GLuint, GLint, GLenum, GLboolean, GLsizei, const void *))                   \
    X(void, glDrawArrays, (GLenum, GLint, GLsizei))                                                             \
    X(void, glDrawElements, (GLenum, GLsizei, GLenum, const void *))                                            \
    X(void, glDrawElementsBaseVertex, (GLenum, GLsizei, GLenum, const void *, GLint))                           \
    X(void, glGenFramebuffers, (GLsizei, GLuint *))                                                             \
    X(void, glDeleteFramebuffers, (GLsizei, const GLuint *))                                                    \
    X(void, glBindFramebuffer, (GLenum, GLuint))                                                                \
    X(void, glFramebufferTexture2D, (GLenum, GLenum, GLenum, GLuint, GLint))                                    \
    X(void, glGenRenderbuffers, (GLsizei, GLuint *))                                                            \
    X(void, glDeleteRenderbuffers, (GLsizei, const GLuint *))                                                   \
    X(void, glBindRenderbuffer, (GLenum, GLuint))                                                               \
    X(void, glRenderbufferStorage, (GLenum, GLenum, GLsizei, GLsizei))                                          \
    X(void, glFramebufferRenderbuffer, (GLenum, GLenum, GLenum, GLuint))                                        \
    X(GLenum, glCheckFramebufferStatus, (GLenum))

#define GL_MIN_DECLARE(ret, name, args) typedef ret(GLAPIENTRY *PFN_##name) args; extern PFN_##name name;
GL_MIN_FUNCS(GL_MIN_DECLARE)
#undef GL_MIN_DECLARE

/* Returns 0 if a function is missing. `get` is e.g. SDL_GL_GetProcAddress. */
int gl_min_load(void *(*get)(const char *));

#ifdef GL_MIN_IMPLEMENTATION
#define GL_MIN_DEFINE(ret, name, args) PFN_##name name;
GL_MIN_FUNCS(GL_MIN_DEFINE)
#undef GL_MIN_DEFINE
int gl_min_load(void *(*get)(const char *)) {
    int ok = 1;
#define GL_MIN_LOAD(ret, name, args)        \
    name = (PFN_##name)get(#name);          \
    if (!name) ok = 0;
    GL_MIN_FUNCS(GL_MIN_LOAD)
#undef GL_MIN_LOAD
    return ok;
}
#endif

#endif
