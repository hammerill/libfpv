/* Force-included into every Betaflight / SimITL C translation unit.
 * Routes their console output to libfpv's log callback instead of the
 * host game's stdout. */
#ifndef LIBFPV_BF_LOG_REDIRECT_H
#define LIBFPV_BF_LOG_REDIRECT_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
int libfpv_bf_printf(const char *fmt, ...);
int libfpv_bf_fprintf(FILE *stream, const char *fmt, ...);
#ifdef __cplusplus
}
#endif

#if defined(_WIN32)
/* not in msvcrt; provided by src/server/core/bf_compat.c */
char *strcasestr(const char *haystack, const char *needle);
/* not in msvcrt either; provided by SimITL's target.c. Without this
 * prototype cli.c calls it as an implicit int function, and the returned
 * pointer is cut to 32 bits on 64-bit Windows (the serial CLI commands). */
char *strsep(char **stringp, const char *delim);
#endif

/* Function-like macros so `__attribute__((format(printf, 1, 2)))` is untouched. */
#define printf(...) libfpv_bf_printf(__VA_ARGS__)
#define fprintf(...) libfpv_bf_fprintf(__VA_ARGS__)

#endif
