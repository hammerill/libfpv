/*
 * libfpv - Betaflight-side glue, C interface usable from libfpv's C++ code.
 * All functions must be called on the simulation thread.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef LIBFPV_BF_BRIDGE_H
#define LIBFPV_BF_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    BFB_CLI_OK = 0,
    BFB_CLI_BUSY = 1,   /* the Configurator is in CLI mode */
    BFB_CLI_ERROR = 2,
};

/* Runs CLI commands (newline separated) and returns the CLI output in
 * *output (free with bfb_free). Never send save/exit/defaults without
 * "noreboot"/"nosave": libfpv filters those before calling this. */
int bfb_cli_run(const char *script, char **output);
void bfb_free(void *p);

typedef struct {
    uint8_t ratesType;
    uint8_t rcRate[3];
    uint8_t rate[3];
    uint8_t expo[3];
    uint16_t rateLimit[3];
    uint8_t profileIndex;
} bfbRates_t;

bool bfb_is_armed(void);
uint32_t bfb_arming_disable_flags(void);
const char *bfb_arming_disable_flag_name(int index);
int bfb_arming_disable_flag_count(void);
bool bfb_failsafe_active(void);
uint32_t bfb_flight_mode_flags(void);
bool bfb_config_dirty(void);
void bfb_write_eeprom(void);
int bfb_cli_mode(void);
bfbRates_t bfb_get_rates(void);

/* Saves pending changes, then makes `path` the active EEPROM (one file per
 * quad preset). A missing file is created with Betaflight defaults.
 * Returns 1 if the file existed, 0 if created, -1 on error. */
int bfb_switch_eeprom(const char *path);
/* Betaflight's own reboot path (re-runs init in-process, like the
 * Configurator's "Save and Reboot"). */
void bfb_reboot(void);
int bfb_reboot_required(void);
/* Forgets the battery so Betaflight detects it again on the next update:
 * the warning and critical voltages follow the new pack's cell count. */
void bfb_battery_reset(void);
/* Zeroes the total armed time (the OSD flight timer). */
void bfb_flight_time_reset(void);

/* Gyro/accelerometer/baro calibration after boot still running. */
int bfb_calibrating(void);
/* Betaflight's "props out" setting; the simulated props follow it. */
int bfb_yaw_motors_reversed(void);
/* Armed in crash flip (turtle) mode: the motors spin backwards. */
int bfb_crash_flip_active(void);
/* The OSD DISABLE mode is on: Betaflight clears the OSD. */
int bfb_osd_disabled(void);
/* The BEEPER mode (the pilot's beeper switch) is on. */
int bfb_beeper_switch(void);
/* Text of OSD custom message element `index` (0..3, "Custom message N" in
 * the Configurator), drawn wherever the player put that element; up to 16
 * characters, "" draws nothing, NULL gives the element back to Betaflight. */
void bfb_osd_set_host_message(int index, const char *text);
/* Calibration, RX recovery or boot grace still pending. */
int bfb_settling(void);

/* The GPS feature is on in the active EEPROM. */
int bfb_gps_configured(void);
/* Betaflight booted with the GPS feature on (features apply at boot). */
int bfb_gps_running(void);
/* The active EEPROM turns a feature on or off that the running Betaflight
 * does not have that way yet (features apply at boot). */
int bfb_features_pending(void);
/* Feeds Betaflight's virtual GPS: position (metres) and velocity (m/s) in
 * a local east/north/up frame. */
void bfb_gps_update(float east, float north, float up, float vEast, float vNorth, float vUp);
/* Feeds Betaflight's virtual barometer: altitude in metres. */
void bfb_baro_update(float up);
/* The simulated RC link's statistics (D36), like an ExpressLRS receiver
 * reports them over CRSF: link quality 0..100 %, the RF mode (RFMD) the
 * link quality element shows, RSSI dBm, SNR dB and the transmitter's power
 * in mW. Call every step while the link is simulated; Betaflight's OSD,
 * warnings and statistics read them. */
void bfb_rc_link_stats(int lq, int rfMode, int rssiDbm, int snrDb, int txPowerMw);
/* Hands link quality and RSSI back to Betaflight's own sources. While the
 * link is not simulated, the OSD leaves out its elements (RSSI, RSSI dBm,
 * link quality, RSNR, TX power): they would show only defaults. */
void bfb_rc_link_release(void);
/* Makes the home point (same frame) override the one Betaflight sets when
 * arming. Call every step while the host's home applies; when it stops,
 * Betaflight takes the next arming point again. */
void bfb_gps_set_home(float east, float north, float up);

void bfb_ws_set_enabled(int enabled);
void bfb_ws_set_base_port(int basePort);
int bfb_ws_base_port(void);
int bfb_ws_client_count(void);

#ifdef __cplusplus
}
#endif

#endif
