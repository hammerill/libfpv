/*
 * libfpv - C library functions Betaflight uses that a platform may lack.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <ctype.h>
#include <string.h>

#if defined(_WIN32)
char *strcasestr(const char *haystack, const char *needle)
{
    size_t n = strlen(needle);
    if (n == 0) {
        return (char *)haystack;
    }
    for (; *haystack; haystack++) {
        if (tolower((unsigned char)*haystack) == tolower((unsigned char)*needle) && _strnicmp(haystack, needle, n) == 0) {
            return (char *)haystack;
        }
    }
    return NULL;
}
#else
typedef int libfpv_bf_compat_empty_translation_unit;
#endif
