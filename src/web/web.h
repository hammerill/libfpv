// libfpv - browser builds (Emscripten): functions of the JS runtime in
// src/web/libfpv_web.js.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#if defined(__EMSCRIPTEN__)

extern "C" {
// The folder kept in IndexedDB across visits, "" without IndexedDB (Node)
// (malloc'd, free() it).
char *libfpv_web_data_dir(void);
void libfpv_web_clipboard_write(const char *utf8);
// Reads the clipboard asynchronously; the text arrives through
// libfpv_web_on_paste (overlay.cpp), for the Import tab or the focused field.
void libfpv_web_clipboard_read(int toImport);
// Opens the address in a new browser tab.
void libfpv_web_open_url(const char *utf8);
}

#endif
