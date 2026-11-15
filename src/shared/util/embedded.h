// libfpv - files compiled into the library (presets, fonts; split builds
// ship them as files instead).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstddef>

namespace fpv {

struct EmbeddedFile {
    const char *path;  // relative, forward slashes
    const unsigned char *data;
    size_t size;
};

extern const EmbeddedFile kEmbeddedPresets[];
extern const size_t kEmbeddedPresetsCount;
extern const EmbeddedFile kEmbeddedFonts[];
extern const size_t kEmbeddedFontsCount;

}  // namespace fpv
