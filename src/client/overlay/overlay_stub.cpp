// libfpv - overlay disabled at build time (LIBFPV_WITH_OVERLAY=OFF).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "overlay/overlay.h"

#include "lib.h"

namespace fpv {

Overlay &overlay() {
    Lib &L = lib();
    if (!L.overlay) L.overlay = new Overlay();
    return *L.overlay;
}

Overlay::Overlay() {}
void Overlay::setOpen(bool) {}
void Overlay::setDisplay(float, float, float) {}
void Overlay::mouseMove(float, float) {}
void Overlay::mouseButton(int, bool) {}
void Overlay::mouseWheel(float, float) {}
void Overlay::key(int, bool) {}
void Overlay::text(const char *) {}
bool Overlay::capturingKey() const { return false; }
const fpv_draw_data *Overlay::render(float) { return nullptr; }
bool Overlay::fontTexture(fpv_texture &) { return false; }
void Overlay::notifyUnmappedRadio(const std::string &) {}

}  // namespace fpv
