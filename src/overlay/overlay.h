// libfpv - built-in Dear ImGui overlay. libfpv owns the ImGui context and
// hands backend-agnostic draw data to the host, which renders it.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "libfpv.h"
#include "osd/draw_buffer.h"
#include "overlay/bf_cache.h"
#include "overlay/radio_wizard.h"

struct ImGuiContext;

namespace fpv {

class Overlay {
public:
    Overlay();

    void toggle() { setOpen(!m_open); }
    void setOpen(bool open);
    bool isOpen() const { return m_open; }
    void setDisplay(float w, float h, float scale);
    void mouseMove(float x, float y);
    void mouseButton(int b, bool down);
    void mouseWheel(float dx, float dy);
    void key(int key, bool down);
    void text(const char *utf8);
    // Clipboard text read by the browser (web builds): into the Import tab,
    // or typed into the focused field.
    void paste(const char *utf8, bool toImport);
    // True while the Controls tab waits for a key to bind.
    bool capturingKey() const;

    const fpv_draw_data *render(float dt);
    bool fontTexture(fpv_texture &out);

    // Opens the radio wizard automatically for a newly seen unmapped radio.
    void notifyUnmappedRadio(const std::string &guid);

private:
    void ensureContext();
    void rebuildFonts();
    void buildUi();
    void tabFly();
    void windSection();
    void tabRadio();
    void tabDrone();
    void tabRates();
    void tabTuning();
    void tabOsd();
    void tabImport();
    void tabCamera();
    void tabControls();
    void tabConfigurator();
#if defined(LIBFPV_KAMIKAZE)
    void tabKamikaze();
    void warheadStatus();
#endif
    void wizardUi(const std::vector<RawDevice> &devs);
    void channelBars(const float *ch);
    const fpv_draw_data *convert();

    ImGuiContext *m_ctx = nullptr;
    bool m_open = false;
    float m_w = 1280, m_h = 720, m_scale = 1.0f;
    float m_fontScaleBuilt = 0;
    std::vector<uint8_t> m_fontPixels;
    int m_fontW = 0, m_fontH = 0;
    uint32_t m_fontRevision = 0;
    DrawBuffer m_draw;
    double m_time = 0;

    BfSettingsCache m_bf;
    RadioWizard m_wizard;
    std::string m_wizardHint;
    bool m_wizardSaved = false;
#if defined(LIBFPV_KAMIKAZE)
    DeviceMapping m_previousMapping;  // before a full setup: its bomb switches are kept
#endif
    std::vector<std::string> m_promptedRadios;
    using TabFn = void (Overlay::*)();
    TabFn m_requestTab = nullptr;  // the tab to bring to the front
    bool m_showRawAxes = false;

    std::string m_importText, m_importReport;
    std::string m_cliInput, m_cliOutput;
    int m_osdSelected = 0;
    int m_rateAxis = 0;
    bool m_rebindingToggle = false;
    bool m_ctrl = false;
    std::string m_status;
};

Overlay &overlay();

}  // namespace fpv
