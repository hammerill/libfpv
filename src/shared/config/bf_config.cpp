// libfpv - Betaflight configuration scripts.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "config/bf_config.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <map>
#include <sstream>

namespace fpv {

namespace {

std::string trim(const std::string &s) {
    size_t b = 0, e = s.size();
    while (b < e && std::isspace((unsigned char)s[b])) b++;
    while (e > b && std::isspace((unsigned char)s[e - 1])) e--;
    return s.substr(b, e - b);
}

std::string lower(std::string s) {
    for (char &c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}

std::vector<std::string> splitWords(const std::string &s) {
    std::vector<std::string> out;
    std::istringstream in(s);
    std::string w;
    while (in >> w) out.push_back(w);
    return out;
}

std::vector<std::string> splitLines(const std::string &text) {
    std::vector<std::string> out;
    std::string line;
    for (char c : text) {
        if (c == '\n' || c == '\r') {
            out.push_back(line);
            line.clear();
        } else {
            line += c;
        }
    }
    if (!line.empty()) out.push_back(line);
    return out;
}

bool startsWith(const std::string &s, const char *p) { return s.rfind(p, 0) == 0; }

// Profile counts of the embedded Betaflight (PID_PROFILE_COUNT,
// CONTROL_RATE_PROFILE_COUNT).
constexpr int kPidProfileCount = 4;
constexpr int kRateProfileCount = 4;

// OSD position: x | y << 5, visible in OSD profile 1.
int osdPos(int x, int y) { return (x & 31) | ((y & 31) << 5) | (1 << 11); }

}  // namespace

namespace {
// Mode `boxId` on the high position of aux channel `channel` (0 = AUX1), in
// mode slot `slot`.
std::string modeLine(int slot, ModeSwitch m) {
    return "aux " + std::to_string(slot) + " " + std::to_string(m.boxId) + " " + std::to_string(m.channel) +
           " 1700 2100 0 0";
}
}  // namespace

namespace {
// PWM with unsynced updates: with synced PWM, Betaflight slows the PID loop
// down to the ESCs' 480 Hz (pid_process_denom), see D5. Right after any
// `defaults`: Betaflight's default protocol is DShot, and a `profile` line
// sets the mixer's output range for the protocol of that moment.
const char *const kMotorDefaults =
    "set motor_pwm_protocol = PWM\n"
    "set use_unsynced_pwm = ON\n";
}  // namespace

std::string betaflightBaseDefaults() {
    return std::string(kMotorDefaults) +
           "set pwr_on_arm_grace = 0\n"
           "feature OSD\n"
           "feature GPS\n"
           "aux 0 0 0 1700 2100 0 0\n";
}

#if defined(LIBFPV_KAMIKAZE)
std::string betaflightBombOsdLine() { return "set osd_custom_msg0_pos = " + std::to_string(osdPos(10, 3)) + "\n"; }
#endif

std::string betaflightOsdLayout(bool ntsc) {
    // NTSC shows 13 rows: the bottom elements go 2 rows up, the middle ones 1
    int lo = ntsc ? 2 : 0, mid = ntsc ? 1 : 0;
    std::ostringstream s;
    s << "set osd_avg_cell_voltage_pos = " << osdPos(1, 14 - lo) << "\n";
    s << "set osd_vbat_pos = " << osdPos(1, 13 - lo) << "\n";
    s << "set osd_current_pos = " << osdPos(23, 13 - lo) << "\n";
    s << "set osd_mah_drawn_pos = " << osdPos(23, 14 - lo) << "\n";
    s << "set osd_tim_2_pos = " << osdPos(12, 14 - lo) << "\n";
    s << "set osd_flymode_pos = " << osdPos(24, 1) << "\n";
    s << "set osd_throttle_pos = " << osdPos(1, 1) << "\n";
    s << "set osd_crosshairs_pos = " << osdPos(13, 7 - mid) << "\n";
    s << "set osd_warnings_pos = " << osdPos(9, 10 - lo) << "\n";
    s << "set osd_disarmed_pos = " << osdPos(10, 4) << "\n";
    s << "set osd_gps_speed_pos = " << osdPos(12, 1) << "\n";
    s << "set osd_craft_name_pos = " << osdPos(11, 12 - lo) << "\n";
    return s.str();
}

std::string betaflightFirstRunScript(bool ntsc) {
    std::ostringstream s;
    s << betaflightBaseDefaults();
    s << "aux 1 1 1 1700 2100 0 0\n";   // ANGLE on AUX2 high
    s << "aux 2 13 2 1700 2100 0 0\n";  // BEEPER on AUX3 high
    s << modeLine(3, kTurtleSwitch) << "\n";   // FLIP OVER AFTER CRASH on AUX4 high
    s << modeLine(4, kAirModeSwitch) << "\n";  // AIR MODE on AUX5 high
    s << modeLine(5, kOsdSwitch) << "\n";      // OSD DISABLE on AUX6 high
    s << "set crashflip_rate = 30\n";   // turtle cuts power past ~90 deg and fast spins
    s << "set vcd_video_system = " << (ntsc ? "NTSC" : "PAL") << "\n";
    s << betaflightOsdLayout(ntsc);
    s << "set craft_name = LIBFPV\n";
#if defined(LIBFPV_KAMIKAZE)
    s << betaflightBombOsdLine();
#endif
    return s.str();
}

std::string betaflightSwitchMigration(const std::string &auxDump, const std::vector<ModeSwitch> &modes) {
    struct Slot {
        int slot, mode, channel;
        bool used;
    };
    std::vector<Slot> slots;
    for (const std::string &raw : splitLines(auxDump)) {
        std::vector<std::string> w = splitWords(trim(raw));
        if (w.size() < 6 || w[0] != "aux") continue;
        int start = std::atoi(w[4].c_str()), end = std::atoi(w[5].c_str());
        Slot s{std::atoi(w[1].c_str()), std::atoi(w[2].c_str()), std::atoi(w[3].c_str()), start < end};
        // a later line for the same slot (a script run after the dump) wins
        auto it = std::find_if(slots.begin(), slots.end(), [&](const Slot &o) { return o.slot == s.slot; });
        if (it != slots.end()) *it = s;
        else slots.push_back(s);
    }
    std::string out;
    for (const ModeSwitch &m : modes) {
        bool taken = false;  // the mode is set up already, or its channel is used
        for (const Slot &s : slots) taken |= s.used && (s.mode == m.boxId || s.channel == m.channel);
        if (taken) continue;
        for (Slot &s : slots) {
            if (s.used) continue;
            s = {s.slot, m.boxId, m.channel, true};
            out += modeLine(s.slot, m) + "\n";
            break;
        }
    }
    return out;
}

CliFilterResult filterCliCommands(const std::string &commands, bool ntsc) {
    CliFilterResult r;
    static const char *kRefused[] = {"bl",       "dfu",        "msc",       "escprog", "serialpassthrough",
                                     "gpspassthrough", "flash_erase", "flash_write", "flash_read", "flash_scan",
                                     "bind_rx",  "dshotprog",  "reboot"};
    std::ostringstream out;
    for (const std::string &raw : splitLines(commands)) {
        std::string line = trim(raw);
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> w = splitWords(line);
        std::string cmd = lower(w[0]);
        bool refused = false;
        for (const char *k : kRefused) {
            if (cmd == k) {
                r.refused.push_back(line + ": not available in the simulator");
                refused = true;
            }
        }
        if (refused) continue;
        if (cmd == "save") {
            out << "save noreboot\n";
        } else if (cmd == "exit") {
            // the CLI session ends by itself; never reboot from here
        } else if (cmd == "defaults") {
            // libfpv's own defaults (modes, OSD), never a non-arming config
            out << "defaults nosave\n" << betaflightFirstRunScript(ntsc);
            r.needsBaseDefaults = true;
        } else {
            out << line << "\n";
        }
    }
    r.script = out.str();
    return r;
}

// A `dump`, not a `diff`: a setting the pilot put back to Betaflight's
// default (an OSD element hidden, a switch cleared) must also reach a new
// model, whose first-run setup is not Betaflight's defaults. Empty names
// dump as "-", which `set` takes back as empty. vcd_video_system stays out:
// the new quad's camera sets it (SimServer::syncVideoSystem), and Betaflight
// moves elements below an NTSC camera's last row up to it.
std::string filterDefaultsProfile(const std::string &dump, unsigned categories, unsigned *found) {
    std::ostringstream s;
    unsigned seen = 0;
    enum { kMaster, kPidProfile, kRateProfile } section = kMaster;
    auto keep = [&](unsigned category, const std::string &line) {
        if (!(categories & category)) return;
        s << line << "\n";
        seen |= category;
    };
    for (const std::string &raw : splitLines(dump)) {
        std::string line = trim(raw);
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> w = splitWords(line);
        std::string cmd = lower(w[0]);
        if (cmd == "profile") {
            section = kPidProfile;
        } else if (cmd == "rateprofile") {
            section = kRateProfile;
            if (categories & kDefaultsRates) s << line << "\n";
        } else if (cmd == "aux" || cmd == "adjrange") {
            keep(kDefaultsModes, line);
        } else if (cmd == "led" || cmd == "color" || cmd == "mode_color") {
            keep(kDefaultsLeds, line);
        } else if (cmd == "feature" && w.size() > 1 && (w[1] == "LED_STRIP" || w[1] == "-LED_STRIP")) {
            keep(kDefaultsLeds, line);
        } else if (cmd == "set" && section == kRateProfile) {
            keep(kDefaultsRates, line);
        } else if (cmd == "set" && section == kMaster) {
            std::string rest = trim(line.substr(3));
            std::string name = lower(trim(rest.substr(0, rest.find('='))));
            if (name == "craft_name" || name == "pilot_name")
                keep(kDefaultsNames, line);
            else if (startsWith(name, "osd_") && name != "osd_displayport_device")
                keep(kDefaultsOsd, line);
            else if (startsWith(name, "ledstrip_"))
                keep(kDefaultsLeds, line);
        }
    }
    // ARM stays on AUX1 whatever an edited file says
    if (seen & kDefaultsModes) s << "aux 0 0 0 1700 2100 0 0\n";
    if (found) *found = seen;
    return s.str();
}

const char *betaflightModeName(int boxId) {
    static const std::map<int, const char *> names = {
        {0, "ARM"},          {1, "ANGLE"},       {2, "HORIZON"},     {3, "ALTHOLD"},     {4, "ANTI GRAVITY"},
        {5, "MAG"},          {6, "HEADFREE"},    {7, "HEADADJ"},     {8, "CAMSTAB"},     {11, "POS HOLD"},
        {12, "PASSTHRU"},    {13, "BEEPER"},     {15, "LEDLOW"},     {17, "CALIB"},      {19, "OSD DISABLE"},
        {20, "TELEMETRY"},   {26, "BLACKBOX"},   {27, "FAILSAFE"},   {28, "AIR MODE"},   {29, "3D DISABLE"},
        {30, "FPV ANGLE MIX"}, {35, "FLIP OVER AFTER CRASH"}, {36, "PREARM"}, {39, "VTX PIT MODE"},
        {44, "PID AUDIO"},   {45, "PARALYZE"},   {46, "GPS RESCUE"}, {47, "ACRO TRAINER"}, {49, "LAUNCH CONTROL"},
        {51, "STICK COMMANDS DISABLE"}, {52, "BEEPER MUTE"}, {53, "READY"}};
    auto it = names.find(boxId);
    return it == names.end() ? "MODE" : it->second;
}

DiffImport buildDiffImport(const std::string &diffText, bool ntsc) {
    DiffImport imp;

    // Hardware-specific commands: they describe the real flight controller,
    // not the flying behaviour.
    static const std::map<std::string, const char *> kSkipCommands = {
        {"resource", "hardware pin assignment"},
        {"timer", "hardware timer assignment"},
        {"dma", "hardware DMA assignment"},
        {"serial", "serial ports: the simulator receives RC over MSP"},
        {"beacon", "DShot beacon needs DShot ESCs"},
        {"vtxtable", "VTX frequency table: no video transmitter hardware"},
        {"vtx", "VTX channel switching: no video transmitter hardware"},
        {"led", "LED strip"},
        {"color", "LED strip colours"},
        {"mode_color", "LED strip colours"},
        {"board_name", "board identity"},
        {"manufacturer_id", "board identity"},
        {"mcu_id", "board identity"},
        {"signature", "board identity"},
        {"map", "channel order: libfpv's radio setup maps sticks to AETR"},
        {"mixer", "only Quad X is simulated"},
        {"mmix", "only Quad X is simulated"},
        {"smix", "servos are not simulated"},
        {"servo", "servos are not simulated"},
        {"rxrange", "receiver endpoints: calibrated by libfpv's radio setup"},
        {"batch", ""},
        {"save", ""},
        {"exit", ""},
        {"defaults", ""},
        {"dshotprog", "ESC programming"},
        {"version", ""},
    };
    // `set` variables that are hardware specific (prefix match).
    static const std::vector<std::pair<std::string, const char *>> kSkipVars = {
        {"motor_pwm_protocol", "forced to PWM, the only protocol the simulator drives"},
        {"motor_pwm_rate", "ESC hardware"},
        {"dshot_", "DShot ESC hardware"},
        {"motor_poles", "ESC telemetry hardware"},
        {"use_unsynced_pwm", "forced on, with the PWM protocol"},
        {"serialrx_", "receiver hardware: RC arrives over MSP"},
        {"rx_spi_", "receiver hardware"},
        {"spektrum_", "receiver hardware"},
        {"srxl2_", "receiver hardware"},
        {"crsf_", "receiver hardware"},
        {"sbus_", "receiver hardware"},
        {"rssi_", "receiver hardware"},
        {"elrs_", "receiver hardware"},
        {"gyro_to_use", "sensor hardware"},
        {"gyro_1_", "sensor hardware"},
        {"gyro_2_", "sensor hardware"},
        {"gyro_hardware", "sensor hardware"},
        {"acc_hardware", "sensor hardware"},
        {"acc_calibration", "calibration of the real accelerometer (the simulated one is perfect)"},
        {"acc_zero", "calibration of the real accelerometer (the simulated one is perfect)"},
        {"gyro_offset", "calibration of the real gyro"},
        {"baro_", "sensor hardware"},
        {"mag_", "sensor hardware"},
        {"max7456_", "OSD chip hardware"},
        {"osd_displayport_device", "the simulator renders the OSD itself"},
        {"vcd_", "camera hardware"},
        {"vbat_scale", "battery sensor calibration"},
        {"vbat_divider", "battery sensor calibration"},
        {"vbat_multiplier", "battery sensor calibration"},
        {"ibata_", "current sensor calibration"},
        {"current_meter", "the simulator provides current"},
        {"battery_meter", "the simulator provides voltage"},
        {"adc_", "hardware ADC"},
        {"sdcard_", "SD card hardware"},
        {"flash_", "flash chip hardware"},
        {"blackbox_device", "no blackbox storage"},
        {"pinio_", "hardware pins"},
        {"beeper_inversion", "beeper hardware"},
        {"beeper_od", "beeper hardware"},
        {"beeper_frequency", "beeper hardware"},
        {"vtx_", "video transmitter hardware"},
        {"frsky_", "telemetry hardware"},
        {"tlm_", "telemetry hardware"},
        {"ledstrip_", "LED strip"},
        {"camera_control_", "camera hardware"},
        {"system_hse_mhz", "MCU clock"},
        {"cpu_overclock", "MCU clock"},
        {"task_statistics", "MCU"},
        {"debug_mode", "debug output"},
        {"pwr_on_arm_grace", "set to 0 by libfpv"},
        {"gps_provider", "the simulator provides GPS"},
        {"gps_ublox_", "GPS hardware"},
        {"gps_sbas_", "GPS hardware"},
        {"gps_auto_", "GPS hardware"},
        {"gps_baud", "GPS hardware"},
    };
    // Features the simulator can honour.
    static const std::vector<std::string> kAllowedFeatures = {"AIRMODE", "ANTI_GRAVITY", "DYNAMIC_FILTER", "OSD",
                                                              "INFLIGHT_ACC_CAL", "GPS"};

    struct AuxLine {
        std::string original;
        int index, mode, channel, start, end, logic, linked;
    };
    std::vector<AuxLine> auxLines;
    std::vector<std::string> body;
    // Lines after a profile this Betaflight does not have would land in the
    // wrong profile: skip until the next profile selection.
    const char *skippingProfile = nullptr;

    for (const std::string &raw : splitLines(diffText)) {
        std::string line = trim(raw);
        if (line.empty()) continue;
        if (line[0] == '#') {
            // "# Betaflight / STM32F405 (S405) 4.5.1 Nov  1 2024 / 12:00:00 ..."
            if (line.find("Betaflight /") != std::string::npos && imp.firmware.empty()) imp.firmware = trim(line.substr(1));
            continue;
        }
        if (startsWith(line, "Entering CLI Mode") || startsWith(line, "###")) continue;
        std::vector<std::string> w = splitWords(line);
        std::string cmd = lower(w[0]);

        if (cmd == "profile" || cmd == "rateprofile") {
            int n = w.size() > 1 ? std::atoi(w[1].c_str()) : 0;
            int max = cmd == "profile" ? kPidProfileCount : kRateProfileCount;
            if (n >= max) {
                skippingProfile = cmd == "profile" ? "PID profile" : "rate profile";
                imp.skipped.push_back(line + "  -> this Betaflight has " + std::to_string(max) + " " +
                                      skippingProfile + "s");
                continue;
            }
            skippingProfile = nullptr;
            body.push_back(line);
            continue;
        }
        if (skippingProfile && cmd == "set") {
            imp.skipped.push_back(line + "  -> belongs to a skipped " + skippingProfile);
            continue;
        }

        auto skipCmd = kSkipCommands.find(cmd);
        if (skipCmd != kSkipCommands.end()) {
            if (cmd == "board_name" && w.size() > 1) imp.boardName = w[1];
            if (skipCmd->second[0]) imp.skipped.push_back(line + "  -> " + skipCmd->second);
            continue;
        }
        if (cmd == "feature") {
            if (w.size() < 2) continue;
            std::string f = w[1];
            bool negate = !f.empty() && f[0] == '-';
            std::string name = negate ? f.substr(1) : f;
            if (std::find(kAllowedFeatures.begin(), kAllowedFeatures.end(), name) == kAllowedFeatures.end()) {
                imp.skipped.push_back(line + "  -> hardware feature, not simulated");
                continue;
            }
            body.push_back(line);
            continue;
        }
        if (cmd == "set") {
            // "set name = value"
            std::string rest = trim(line.substr(3));
            std::string name = lower(trim(rest.substr(0, rest.find('='))));
            const char *reason = nullptr;
            for (auto &kv : kSkipVars) {
                if (startsWith(name, kv.first.c_str())) {
                    reason = kv.second;
                    break;
                }
            }
            if (reason) {
                imp.skipped.push_back(line + "  -> " + reason);
                continue;
            }
            body.push_back(line);
            continue;
        }
        if (cmd == "aux" && w.size() >= 6) {
            AuxLine a;
            a.original = line;
            a.index = std::atoi(w[1].c_str());
            a.mode = std::atoi(w[2].c_str());
            a.channel = std::atoi(w[3].c_str());
            a.start = std::atoi(w[4].c_str());
            a.end = std::atoi(w[5].c_str());
            a.logic = w.size() > 6 ? std::atoi(w[6].c_str()) : 0;
            a.linked = w.size() > 7 ? std::atoi(w[7].c_str()) : 0;
            auxLines.push_back(a);
            continue;
        }
        if (cmd == "adjrange") {
            imp.skipped.push_back(line + "  -> in-flight adjustments are not mapped to the simulator's switches");
            continue;
        }
        body.push_back(line);
    }

    // --- modes: ARM -> AUX1, other used channels -> AUX2..AUX6 ---------------
    int armChannel = -1;
    for (auto &a : auxLines) {
        if (a.mode == 0 && a.start < a.end && !(a.start == 900 && a.end == 900)) {
            armChannel = a.channel;
            break;
        }
    }
    std::map<int, int> channelMap;  // original aux channel -> new aux channel
    if (armChannel >= 0) channelMap[armChannel] = 0;
    int nextAux = 1;
    std::vector<std::string> auxOut;
    int outIndex = 1;
    for (auto &a : auxLines) {
        if (a.mode == 0) continue;
        if (a.start >= a.end || (a.start == 900 && a.end == 900)) continue;  // unused slot
        if (channelMap.find(a.channel) == channelMap.end()) {
            if (nextAux > 5) {
                imp.skipped.push_back(a.original + "  -> " + betaflightModeName(a.mode) +
                                      ": more switches than AUX2-AUX6 available");
                continue;
            }
            channelMap[a.channel] = nextAux++;
        }
        int ch = channelMap[a.channel];
        if (ch != a.channel) {
            imp.remapped.push_back(std::string(betaflightModeName(a.mode)) + ": AUX" + std::to_string(a.channel + 1) +
                                   " -> AUX" + std::to_string(ch + 1));
        }
        if (outIndex >= 20) break;
        std::ostringstream l;
        l << "aux " << outIndex++ << " " << a.mode << " " << ch << " " << a.start << " " << a.end << " " << a.logic
          << " " << a.linked;
        auxOut.push_back(l.str());
    }
    if (armChannel > 0) imp.remapped.push_back("ARM: AUX" + std::to_string(armChannel + 1) + " -> AUX1");

    // --- assemble ------------------------------------------------------------
    std::ostringstream s;
    s << "defaults nosave\n" << kMotorDefaults;
    for (auto &l : body) {
        s << l << "\n";
        imp.applied.push_back(l);
    }
    // clear all mode slots, then libfpv's base and the remapped modes
    for (int i = 0; i < 20; i++) s << "aux " << i << " 0 0 900 900 0 0\n";
    s << betaflightBaseDefaults();
    s << "set vcd_video_system = " << (ntsc ? "NTSC" : "PAL") << "\n";
    for (auto &l : auxOut) {
        s << l << "\n";
        imp.applied.push_back(l);
    }
    s << "save noreboot\n";
    imp.script = s.str();
    return imp;
}

std::string formatImportReport(const DiffImport &imp, const std::string &cliOutput) {
    std::ostringstream r;
    if (!imp.firmware.empty()) r << "Source: " << imp.firmware << "\n";
    if (!imp.boardName.empty()) r << "Board: " << imp.boardName << "\n";
    r << "Applied " << imp.applied.size() << " settings.\n";
    if (!imp.remapped.empty()) {
        r << "\nSwitches moved (flip these on your radio):\n";
        for (auto &m : imp.remapped) r << "  " << m << "\n";
    }
    r << "\nAlways set by libfpv: motor protocol PWM, ARM on AUX1, no power-on arming delay, MSP receiver.\n";
    if (!imp.skipped.empty()) {
        r << "\nSkipped " << imp.skipped.size() << " lines:\n";
        for (auto &l : imp.skipped) r << "  " << l << "\n";
    }
    // Lines Betaflight itself rejected. The non-interactive CLI does not
    // always separate responses with newlines, so extract the markers.
    std::vector<std::string> errors;
    size_t pos = 0;
    while ((pos = cliOutput.find("###ERROR", pos)) != std::string::npos) {
        size_t end = cliOutput.find("###", pos + 8);
        if (end == std::string::npos) end = cliOutput.size();
        errors.push_back(trim(cliOutput.substr(pos + 3, end - pos - 3)));
        pos = end + 3;
    }
    pos = 0;
    while ((pos = cliOutput.find("ERR_CMD_NA: ", pos)) != std::string::npos) {
        size_t end = cliOutput.find_first_of("\r\n", pos);
        if (end == std::string::npos) end = cliOutput.size();
        errors.push_back("unknown command: " + trim(cliOutput.substr(pos + 12, end - pos - 12)));
        pos = end;
    }
    if (!errors.empty()) {
        r << "\nRejected by Betaflight " << "(" << errors.size() << "):\n";
        for (auto &e : errors) r << "  " << e << "\n";
    }
    return r.str();
}

}  // namespace fpv
