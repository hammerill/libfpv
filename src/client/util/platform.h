// libfpv - OS specifics: pinning the module in memory, process exit hooks.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <string>

namespace fpv {

// An environment variable as UTF-8, "" when unset.
std::string envUtf8(const char *name);

// The per-user settings folder (UTF-8), "" when unknown: Roaming AppData on
// Windows, ~/Library/Application Support on macOS, $XDG_CONFIG_HOME or
// ~/.config elsewhere.
std::string userConfigDir();

// The folder libfpv's own library file is in (UTF-8), "" when unknown (or
// when it is linked into the program, as on the web).
std::string libraryDirectory();

// Prevents the host from unmapping libfpv (FreeLibrary/dlclose) while its
// threads exist. Returns false if pinning is not possible.
bool pinLibraryInMemory();
// True once pinLibraryInMemory() succeeded (for tests).
bool libraryIsPinned();

// Runs fn at process exit on platforms where libfpv's threads are still
// alive at that point (POSIX). On Windows, ExitProcess has already killed
// them before DLL detach, so nothing is registered there.
void installExitHandler(void (*fn)());

}  // namespace fpv
