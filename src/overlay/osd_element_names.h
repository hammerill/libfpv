// libfpv - names and groups for Betaflight's OSD elements, for the
// overlay's OSD tab. The list of elements itself comes from Betaflight's
// CLI at run time (every `osd_*_pos` setting `get osd_` reports), so it
// always matches what this build compiled in; this table only adds the
// Betaflight Configurator's names and a group. An element missing here
// still shows, under "Other" with a name made from its setting.
// Header-only: the test suite checks it against the CLI.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstring>
#include <string>

namespace fpv {

enum OsdGroup {
    OSD_GROUP_BATTERY,
    OSD_GROUP_FLIGHT,
    OSD_GROUP_GPS,
    OSD_GROUP_TIMERS,
    OSD_GROUP_STATUS,
    OSD_GROUP_LINK,
    OSD_GROUP_TUNING,
    OSD_GROUP_GOGGLES,
    OSD_GROUP_OTHER,
    OSD_GROUP_COUNT
};

inline const char *osdGroupName(int g) {
    static const char *const names[OSD_GROUP_COUNT] = {
        "Battery", "Flight", "GPS", "Timers", "Status and messages", "Radio and video link",
        "Tuning and diagnostics", "HD goggles (MSP DisplayPort)", "Other"};
    return g >= 0 && g < OSD_GROUP_COUNT ? names[g] : names[OSD_GROUP_OTHER];
}

struct OsdElementName {
    const char *setting;  // Betaflight CLI setting
    const char *label;    // Betaflight Configurator's name
    int group;
};

// Every element Betaflight 4.5/4.6 can compile in (settings.c), in the
// order the groups list them; #ifdef'd ones simply never appear at run time.
inline const OsdElementName *osdElementNames(int *count) {
    static const OsdElementName table[] = {
        {"osd_vbat_pos", "Battery voltage", OSD_GROUP_BATTERY},
        {"osd_avg_cell_voltage_pos", "Average cell voltage", OSD_GROUP_BATTERY},
        {"osd_current_pos", "Current draw", OSD_GROUP_BATTERY},
        {"osd_mah_drawn_pos", "mAh drawn", OSD_GROUP_BATTERY},
        {"osd_wh_drawn_pos", "Watt hours drawn", OSD_GROUP_BATTERY},
        {"osd_power_pos", "Power", OSD_GROUP_BATTERY},
        {"osd_battery_usage_pos", "Battery usage", OSD_GROUP_BATTERY},
        {"osd_efficiency_pos", "Efficiency", OSD_GROUP_BATTERY},
        {"osd_remaining_time_estimate_pos", "Remaining time estimate", OSD_GROUP_BATTERY},

        {"osd_flymode_pos", "Fly mode", OSD_GROUP_FLIGHT},
        {"osd_throttle_pos", "Throttle position", OSD_GROUP_FLIGHT},
        {"osd_crosshairs_pos", "Crosshairs", OSD_GROUP_FLIGHT},
        {"osd_ah_pos", "Artificial horizon", OSD_GROUP_FLIGHT},
        {"osd_ah_sbar_pos", "Horizon sidebars", OSD_GROUP_FLIGHT},
        {"osd_altitude_pos", "Altitude", OSD_GROUP_FLIGHT},
        {"osd_nvario_pos", "Numerical vario", OSD_GROUP_FLIGHT},
        {"osd_nheading_pos", "Numerical heading", OSD_GROUP_FLIGHT},
        {"osd_pit_ang_pos", "Pitch angle", OSD_GROUP_FLIGHT},
        {"osd_rol_ang_pos", "Roll angle", OSD_GROUP_FLIGHT},
        {"osd_g_force_pos", "G force", OSD_GROUP_FLIGHT},
        {"osd_anti_gravity_pos", "Anti gravity", OSD_GROUP_FLIGHT},
        {"osd_flip_arrow_pos", "Flip arrow", OSD_GROUP_FLIGHT},
        {"osd_up_down_reference_pos", "Up/down reference", OSD_GROUP_FLIGHT},
        {"osd_stick_overlay_left_pos", "Stick overlay left", OSD_GROUP_FLIGHT},
        {"osd_stick_overlay_right_pos", "Stick overlay right", OSD_GROUP_FLIGHT},
        {"osd_camera_frame_pos", "Camera frame", OSD_GROUP_FLIGHT},
        {"osd_lidar_dist_pos", "Lidar distance", OSD_GROUP_FLIGHT},

        {"osd_gps_speed_pos", "Speed", OSD_GROUP_GPS},
        {"osd_gps_sats_pos", "GPS satellites", OSD_GROUP_GPS},
        {"osd_gps_lat_pos", "Latitude", OSD_GROUP_GPS},
        {"osd_gps_lon_pos", "Longitude", OSD_GROUP_GPS},
        {"osd_home_dir_pos", "Home direction", OSD_GROUP_GPS},
        {"osd_home_dist_pos", "Home distance", OSD_GROUP_GPS},
        {"osd_flight_dist_pos", "Flight distance", OSD_GROUP_GPS},
        {"osd_compass_bar_pos", "Compass bar", OSD_GROUP_GPS},
        {"osd_gps_lap_curr_pos", "Lap time current", OSD_GROUP_GPS},
        {"osd_gps_lap_prev_pos", "Lap time previous", OSD_GROUP_GPS},
        {"osd_gps_lap_best3_pos", "Lap time best 3", OSD_GROUP_GPS},

        {"osd_tim_1_pos", "Timer 1", OSD_GROUP_TIMERS},
        {"osd_tim_2_pos", "Timer 2", OSD_GROUP_TIMERS},
        {"osd_rtc_date_time_pos", "Date and time", OSD_GROUP_TIMERS},
        {"osd_total_flights_pos", "Total flights", OSD_GROUP_TIMERS},

        {"osd_warnings_pos", "Warnings", OSD_GROUP_STATUS},
        {"osd_disarmed_pos", "Disarmed", OSD_GROUP_STATUS},
        {"osd_ready_mode_pos", "Ready mode", OSD_GROUP_STATUS},
        {"osd_craft_name_pos", "Craft name", OSD_GROUP_STATUS},
        {"osd_pilot_name_pos", "Pilot name", OSD_GROUP_STATUS},
#if defined(LIBFPV_KAMIKAZE)
        {"osd_custom_msg0_pos", "Bomb status", OSD_GROUP_STATUS},  // the warhead state (Flight::step)
#else
        {"osd_custom_msg0_pos", "Custom message 1", OSD_GROUP_STATUS},
#endif
        {"osd_custom_msg1_pos", "Custom message 2", OSD_GROUP_STATUS},
        {"osd_custom_msg2_pos", "Custom message 3", OSD_GROUP_STATUS},
        {"osd_custom_msg3_pos", "Custom message 4", OSD_GROUP_STATUS},
        {"osd_pidrate_profile_pos", "PID and rate profile", OSD_GROUP_STATUS},
        {"osd_pid_profile_name_pos", "PID profile name", OSD_GROUP_STATUS},
        {"osd_rate_profile_name_pos", "Rate profile name", OSD_GROUP_STATUS},
        {"osd_profile_name_pos", "OSD profile name", OSD_GROUP_STATUS},
        {"osd_log_status_pos", "Log status", OSD_GROUP_STATUS},
        {"osd_adjustment_range_pos", "Adjustment range", OSD_GROUP_STATUS},
        {"osd_aux_pos", "AUX value", OSD_GROUP_STATUS},

        {"osd_rssi_pos", "RSSI value", OSD_GROUP_LINK},
        {"osd_rssi_dbm_pos", "RSSI dBm value", OSD_GROUP_LINK},
        {"osd_rsnr_pos", "RSNR value", OSD_GROUP_LINK},
        {"osd_link_quality_pos", "Link quality", OSD_GROUP_LINK},
        {"osd_link_tx_power_pos", "TX uplink power", OSD_GROUP_LINK},
        {"osd_vtx_channel_pos", "VTX channel", OSD_GROUP_LINK},
        {"osd_rcchannels_pos", "RC channels", OSD_GROUP_LINK},

        {"osd_pid_roll_pos", "Roll PIDs", OSD_GROUP_TUNING},
        {"osd_pid_pitch_pos", "Pitch PIDs", OSD_GROUP_TUNING},
        {"osd_pid_yaw_pos", "Yaw PIDs", OSD_GROUP_TUNING},
        {"osd_motor_diag_pos", "Motor diagnostics", OSD_GROUP_TUNING},
        {"osd_esc_tmp_pos", "ESC temperature", OSD_GROUP_TUNING},
        {"osd_esc_rpm_pos", "ESC RPM", OSD_GROUP_TUNING},
        {"osd_esc_rpm_freq_pos", "ESC RPM frequency", OSD_GROUP_TUNING},
        {"osd_core_temp_pos", "Core temperature", OSD_GROUP_TUNING},
        {"osd_debug_pos", "Debug", OSD_GROUP_TUNING},
        {"osd_debug2_pos", "Debug 2", OSD_GROUP_TUNING},

        {"osd_sys_goggle_voltage_pos", "Goggle voltage", OSD_GROUP_GOGGLES},
        {"osd_sys_vtx_voltage_pos", "VTX voltage", OSD_GROUP_GOGGLES},
        {"osd_sys_bitrate_pos", "Bitrate", OSD_GROUP_GOGGLES},
        {"osd_sys_delay_pos", "Delay", OSD_GROUP_GOGGLES},
        {"osd_sys_distance_pos", "Distance", OSD_GROUP_GOGGLES},
        {"osd_sys_lq_pos", "Link quality (goggles)", OSD_GROUP_GOGGLES},
        {"osd_sys_goggle_dvr_pos", "Goggle DVR", OSD_GROUP_GOGGLES},
        {"osd_sys_vtx_dvr_pos", "VTX DVR", OSD_GROUP_GOGGLES},
        {"osd_sys_warnings_pos", "Warnings (goggles)", OSD_GROUP_GOGGLES},
        {"osd_sys_vtx_temp_pos", "VTX temperature", OSD_GROUP_GOGGLES},
        {"osd_sys_fan_speed_pos", "Fan speed", OSD_GROUP_GOGGLES},
    };
    if (count) *count = (int)(sizeof(table) / sizeof(table[0]));
    return table;
}

// An OSD element's position setting: osd_<element>_pos.
inline bool isOsdElementSetting(const std::string &name) {
    return name.size() > 8 && name.compare(0, 4, "osd_") == 0 && name.compare(name.size() - 4, 4, "_pos") == 0;
}

// The table entry, or null for an element this table does not know.
inline const OsdElementName *findOsdElementName(const std::string &setting) {
    int n = 0;
    const OsdElementName *t = osdElementNames(&n);
    for (int i = 0; i < n; i++)
        if (setting == t[i].setting) return &t[i];
    return nullptr;
}

// Sort key: the table's order, unknown elements last.
inline int osdElementOrder(const std::string &setting) {
    int n = 0;
    const OsdElementName *t = osdElementNames(&n);
    for (int i = 0; i < n; i++)
        if (setting == t[i].setting) return i;
    return n;
}

// The Configurator's name, or one made from the setting ("osd_foo_bar_pos"
// becomes "Foo bar").
inline std::string osdElementLabel(const std::string &setting) {
    if (const OsdElementName *e = findOsdElementName(setting)) return e->label;
    std::string s = setting;
    if (s.compare(0, 4, "osd_") == 0) s.erase(0, 4);
    if (s.size() > 4 && s.compare(s.size() - 4, 4, "_pos") == 0) s.erase(s.size() - 4);
    for (char &c : s)
        if (c == '_') c = ' ';
    if (!s.empty() && s[0] >= 'a' && s[0] <= 'z') s[0] = (char)(s[0] - 'a' + 'A');
    return s;
}

inline int osdElementGroup(const std::string &setting) {
    const OsdElementName *e = findOsdElementName(setting);
    return e ? e->group : OSD_GROUP_OTHER;
}

// Position values (osd.h): column bits 0-4 plus bit 10 for HD columns
// 32-63, row bits 5-9, one visibility bit per OSD profile from bit 11,
// element type in bits 14-15.
inline int osdPosColumn(int pos) { return (pos & 31) | ((pos >> 5) & 32); }
inline int osdPosRow(int pos) { return (pos >> 5) & 31; }
inline int osdProfileFlag(int profile) { return 0x800 << ((profile >= 1 && profile <= 3 ? profile : 1) - 1); }
// Moves an element, keeping its type and other profiles' visibility, and
// shows it in `profileFlag`.
inline int osdPosMoved(int pos, int col, int row, int profileFlag) {
    return (pos & ~0x7FF) | (col & 31) | ((col & 32) << 5) | ((row & 31) << 5) | profileFlag;
}

}  // namespace fpv
