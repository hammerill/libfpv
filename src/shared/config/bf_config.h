// libfpv - Betaflight configuration through Betaflight's own CLI: libfpv's
// base defaults, filtering of host/player commands and `diff all` import.
// Pure text processing; the runner executes the resulting scripts on the
// simulation thread.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <string>
#include <vector>

namespace fpv {

// Settings libfpv needs regardless of the tune: PWM (the only protocol
// SimITL drives), ARM on AUX1, no power-on grace, MSP receiver, OSD.
std::string betaflightBaseDefaults();
// The out-of-the-box tune for a fresh EEPROM: base defaults + modes on
// AUX2-6 + a readable OSD layout (for an NTSC camera's 13 rows, or 16).
std::string betaflightFirstRunScript(bool ntsc = false);
// libfpv's default OSD layout as `set` lines: for NTSC's 13 rows or PAL's 16.
std::string betaflightOsdLayout(bool ntsc);
#if defined(LIBFPV_KAMIKAZE)
// Where the warhead state goes: Betaflight's custom message 1 element,
// centred on row 3 (a `set` line).
std::string betaflightBombOsdLine();
#endif

// A mode switch libfpv sets up: Betaflight box id (permanent id, as the
// `aux` command takes it) on the high position of an aux channel (0 = AUX1).
struct ModeSwitch {
    int boxId, channel;
};
constexpr ModeSwitch kTurtleSwitch{35, 3};   // FLIP OVER AFTER CRASH on AUX4
constexpr ModeSwitch kAirModeSwitch{28, 4};  // AIR MODE on AUX5
constexpr ModeSwitch kOsdSwitch{19, 5};      // OSD DISABLE on AUX6

// For EEPROMs made before a switch existed: given the CLI's `aux` output,
// the `aux` lines that add `modes` in free slots. A mode is left out when it
// is configured already or its channel is used by another mode.
std::string betaflightSwitchMigration(const std::string &auxDump, const std::vector<ModeSwitch> &modes);

struct CliFilterResult {
    std::string script;                 // what to run
    std::vector<std::string> refused;   // "<line>: <reason>"
    bool needsBaseDefaults = false;     // a "defaults" was rewritten
};

// Rewrites save/defaults/exit to their no-reboot forms and refuses
// commands that make no sense in a simulator (bootloader, passthrough...).
// ntsc: the quad's camera, for what a `defaults` turns into (libfpv's first-run
// configuration keeps Betaflight's video system with the camera).
CliFilterResult filterCliCommands(const std::string &commands, bool ntsc = false);

struct DiffImport {
    std::string script;                // full script to execute
    std::vector<std::string> applied;  // informational
    std::vector<std::string> skipped;  // "<line>  -> <reason>"
    std::vector<std::string> remapped; // e.g. "ANGLE: AUX3 -> AUX2"
    std::string boardName, firmware;   // from the diff header, if present
};

// Builds the import script for a real quad's `diff all` / `dump`.
// ntsc: the quad's camera: the import resets to defaults, then keeps
// Betaflight's video system with it.
DiffImport buildDiffImport(const std::string &diffText, bool ntsc = false);
std::string formatImportReport(const DiffImport &imp, const std::string &cliOutput);

// Names of Betaflight modes by box id (as used by the `aux` command).
const char *betaflightModeName(int boxId);

}  // namespace fpv
