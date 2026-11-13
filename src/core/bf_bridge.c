/*
 * libfpv - Betaflight-side glue.
 *
 * Compiled together with the Betaflight sources (same include paths and
 * defines). Everything here must only be called from the simulation thread,
 * the same thread that runs Betaflight's scheduler.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "cli/cli.h"
#include "config/config.h"
#include "drivers/serial.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "io/beeper.h"
#include "io/gps.h"
#include "io/gps_virtual.h"
#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "drivers/barometer/barometer_virtual.h"
#include "rx/rx.h"
#include "pg/pg.h"
#include "sensors/battery.h"
#include "scheduler/scheduler.h"
#include "config/feature.h"
#include "config/config_eeprom.h"
#include "drivers/system.h"

#include "drivers/serial_ws.h"

#include "bf_bridge.h"

/* ---------------------------------------------------------------------------
 * In-memory serial port used to talk to Betaflight's CLI from libfpv.
 * ------------------------------------------------------------------------- */

typedef struct {
    serialPort_t port;
    const char *in;
    size_t inLen;
    size_t inPos;
    char *out;
    size_t outLen;
    size_t outCap;
} memPort_t;

static uint8_t memPortDummyBuffer[1];

static void memWrite(serialPort_t *instance, uint8_t ch)
{
    memPort_t *p = (memPort_t *)instance;
    /* strip the CLI's STX/ETX flow control bytes */
    if (ch == 0x2 || ch == 0x3) {
        return;
    }
    if (p->outLen + 2 > p->outCap) {
        size_t cap = p->outCap ? p->outCap * 2 : 4096;
        char *grown = (char *)realloc(p->out, cap);
        if (!grown) {
            return;
        }
        p->out = grown;
        p->outCap = cap;
    }
    p->out[p->outLen++] = (char)ch;
    p->out[p->outLen] = 0;
}

static uint32_t memRxWaiting(const serialPort_t *instance)
{
    const memPort_t *p = (const memPort_t *)instance;
    return (uint32_t)(p->inLen - p->inPos);
}

static uint32_t memTxFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return 0x10000;
}

static uint8_t memRead(serialPort_t *instance)
{
    memPort_t *p = (memPort_t *)instance;
    return p->inPos < p->inLen ? (uint8_t)p->in[p->inPos++] : 0;
}

static bool memTxEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

static const struct serialPortVTable memVTable = {
    .serialWrite = memWrite,
    .serialTotalRxWaiting = memRxWaiting,
    .serialTotalTxFree = memTxFree,
    .serialRead = memRead,
    .serialSetBaudRate = NULL,
    .isSerialTransmitBufferEmpty = memTxEmpty,
    .setMode = NULL,
    .setCtrlLineStateCb = NULL,
    .setBaudRateCb = NULL,
    .writeBuf = NULL,
    .beginWrite = NULL,
    .endWrite = NULL,
};

int bfb_cli_run(const char *script, char **output)
{
    *output = NULL;
    if (cliMode) {
        /* the Configurator currently owns the CLI */
        return BFB_CLI_BUSY;
    }

    /* every line must be terminated, then ETX ends the session */
    size_t len = strlen(script);
    char *in = (char *)malloc(len + 3);
    if (!in) {
        return BFB_CLI_ERROR;
    }
    memcpy(in, script, len);
    in[len] = '\n';
    in[len + 1] = 0x3;
    in[len + 2] = 0;

    memPort_t port;
    memset(&port, 0, sizeof(port));
    port.port.vTable = &memVTable;
    port.port.rxBuffer = memPortDummyBuffer;
    port.port.txBuffer = memPortDummyBuffer;
    port.port.rxBufferSize = 1;
    port.port.txBufferSize = 1;
    port.port.identifier = -1;
    port.in = in;
    port.inLen = len + 2;

    cliEnter(&port.port, false);
    while (cliMode && port.inPos < port.inLen) {
        cliProcess();
    }
    free(in);
#ifdef USE_OSD
    /* a real CLI "save" reboots, which rebuilds the list of OSD elements to
     * draw; "save noreboot" does not, so moved or enabled elements would
     * stay hidden until the next reboot */
    osdAnalyzeActiveElements();
#endif

    if (!port.out) {
        port.out = (char *)calloc(1, 1);
    }
    *output = port.out;
    return cliMode ? BFB_CLI_ERROR : BFB_CLI_OK;
}

void bfb_free(void *p)
{
    free(p);
}

/* ---------------------------------------------------------------------------
 * State accessors
 * ------------------------------------------------------------------------- */

bool bfb_is_armed(void)
{
    return ARMING_FLAG(ARMED) != 0;
}

uint32_t bfb_arming_disable_flags(void)
{
    return (uint32_t)getArmingDisableFlags();
}

const char *bfb_arming_disable_flag_name(int index)
{
    return getArmingDisableFlagName((armingDisableFlags_e)(1u << index));
}

int bfb_arming_disable_flag_count(void)
{
    return ARMING_DISABLE_FLAGS_COUNT;
}

bool bfb_failsafe_active(void)
{
    return failsafeIsActive();
}

uint32_t bfb_flight_mode_flags(void)
{
    return flightModeFlags;
}

bool bfb_config_dirty(void)
{
    return isConfigDirty();
}

void bfb_write_eeprom(void)
{
    writeEEPROM();
}

int bfb_cli_mode(void)
{
    return cliMode ? 1 : 0;
}

bfbRates_t bfb_get_rates(void)
{
    bfbRates_t r;
    memset(&r, 0, sizeof(r));
    const controlRateConfig_t *c = controlRateProfiles(getCurrentControlRateProfileIndex());
    r.ratesType = c->rates_type;
    for (int axis = 0; axis < 3; axis++) {
        r.rcRate[axis] = c->rcRates[axis];
        r.rate[axis] = c->rates[axis];
        r.expo[axis] = c->rcExpo[axis];
        r.rateLimit[axis] = c->rate_limit[axis];
    }
    r.profileIndex = getCurrentControlRateProfileIndex();
    return r;
}

/* EEPROM_FILENAME and loadEEPROMFromFile() live in SimITL's target.c */
extern char *EEPROM_FILENAME;
extern bool loadEEPROMFromFile(void);

int bfb_switch_eeprom(const char *path)
{
    if (isConfigDirty()) {
        writeEEPROM();  /* keep the previous quad's pending changes */
    }
    FILE *probe = fopen(path, "rb");
    int existed = probe != NULL;
    if (probe) {
        fclose(probe);
    }
    size_t n = strlen(path);
    char *name = (char *)malloc(n + 1);
    if (!name) {
        return -1;
    }
    memcpy(name, path, n + 1);
    EEPROM_FILENAME = name; /* previous buffer intentionally kept: SimITL owns it */

    initEEPROM();
    if (!existed) {
        /* a new file was created from the previous quad's memory image */
        resetEEPROM();
    }
    ensureEEPROMStructureIsValid();
    if (!readEEPROM()) {
        resetEEPROM();
    }
    return existed;
}

void bfb_reboot(void)
{
    systemReset();
}

int bfb_reboot_required(void)
{
    return (getArmingDisableFlags() & ARMING_DISABLED_REBOOT_REQUIRED) != 0;
}

void bfb_ws_set_enabled(int enabled)
{
    wsSetEnabled(enabled);
}

int bfb_yaw_motors_reversed(void)
{
    return mixerConfig()->yaw_motors_reversed ? 1 : 0;
}

int bfb_crash_flip_active(void)
{
    /* on a real quad, disarming sends the ESCs back to normal direction */
    return ARMING_FLAG(ARMED) && isCrashFlipModeActive();
}

void bfb_osd_set_host_message(int index, const char *text)
{
    if (index < 0 || index >= OSD_CUSTOM_MSG_COUNT) {
        return;
    }
    osdHostCustomMsgSet[index] = text != NULL;
    strncpy(osdHostCustomMsg[index], text ? text : "", MAX_NAME_LENGTH);
    osdHostCustomMsg[index][MAX_NAME_LENGTH] = '\0';
}

int bfb_osd_disabled(void)
{
    return IS_RC_MODE_ACTIVE(BOXOSD) ? 1 : 0;
}

int bfb_beeper_switch(void)
{
    return IS_RC_MODE_ACTIVE(BOXBEEPERON) ? 1 : 0;
}

void bfb_battery_reset(void)
{
    batteryInit();
}

void bfb_flight_time_reset(void)
{
    osdFlyTime = 0;
}

int bfb_calibrating(void)
{
    return (getArmingDisableFlags() & ARMING_DISABLED_CALIBRATING) != 0;
}

int bfb_settling(void)
{
    return (getArmingDisableFlags() &
            (ARMING_DISABLED_CALIBRATING | ARMING_DISABLED_RX_FAILSAFE | ARMING_DISABLED_BOOT_GRACE_TIME |
             ARMING_DISABLED_FAILSAFE)) != 0;
}

void bfb_ws_set_base_port(int basePort)
{
    wsSetBasePort(basePort);
}

int bfb_ws_base_port(void)
{
    return wsGetBasePort();
}

int bfb_ws_client_count(void)
{
    return wsGetClientCount();
}

/* ---------------------------------------------------------------------------
 * GPS. Betaflight's virtual GPS provider (the default on this target) reads
 * whatever setVirtualGPS() last stored, at 10 Hz like a real module. The
 * simulator's local frame maps onto a flat patch of earth at 0 N 0 E, where
 * a degree of latitude and of longitude are both EARTH_ANGLE_TO_CM long,
 * so Betaflight's distance and bearing maths gives back exact metres.
 * ------------------------------------------------------------------------- */

static const double kMetresPerDegree = EARTH_ANGLE_TO_CM * GPS_DEGREES_DIVIDER / 100.0;

static gpsLocation_t toLocation(float east, float north, float up)
{
    gpsLocation_t l;
    l.lat = (int32_t)lrint(north / kMetresPerDegree * GPS_DEGREES_DIVIDER);
    l.lon = (int32_t)lrint(east / kMetresPerDegree * GPS_DEGREES_DIVIDER);
    l.altCm = (int32_t)lrintf(up * 100.0f);
    return l;
}

int bfb_gps_configured(void)
{
    return featureIsConfigured(FEATURE_GPS) ? 1 : 0;
}

int bfb_gps_running(void)
{
    /* the feature only takes effect at (re)boot, which enables the task */
    taskInfo_t info;
    getTaskInfo(TASK_GPS, &info);
    return featureIsEnabled(FEATURE_GPS) && info.isEnabled;
}

int bfb_features_pending(void)
{
    for (int bit = 0; bit < 32; bit++) {
        const uint32_t mask = 1u << bit;
        if (!featureIsEnabled(mask) != !featureIsConfigured(mask)) {
            return 1;
        }
    }
    return 0;
}

void bfb_gps_update(float east, float north, float up, float vEast, float vNorth, float vUp)
{
    const float ground = sqrtf(vEast * vEast + vNorth * vNorth);
    const float speed3d = sqrtf(ground * ground + vUp * vUp);
    float course = atan2f(vEast, vNorth) * (180.0f / 3.14159265f);
    if (course < 0) {
        course += 360.0f;
    }
    setVirtualGPS(north / kMetresPerDegree, east / kMetresPerDegree, up, ground, speed3d, course);
}

void bfb_gps_set_home(float east, float north, float up)
{
    const gpsLocation_t home = toLocation(east, north, up);
    if (STATE(GPS_FIX_HOME) && GPS_home_llh.lat == home.lat && GPS_home_llh.lon == home.lon &&
        GPS_home_llh.altCm == home.altCm) {
        return;
    }
    /* first call, a new home, or Betaflight just took the arming point */
    GPS_home_llh = home;
    GPS_calc_longitude_scaling(home.lat);
    ENABLE_STATE(GPS_FIX_HOME);
    if (STATE(GPS_FIX)) {
        /* what onGpsNewData() would show at the next fix, now, so the OSD
         * never shows the arming point for a GPS cycle */
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&gpsSol.llh, &GPS_home_llh, false, &dist, &dir);
        GPS_distanceToHome = dist / 100;
        GPS_distanceToHomeCm = dist;
        GPS_directionToHome = dir / 10;
    }
}

/* The virtual barometer otherwise reads sea level forever, and Betaflight's
 * altitude estimate (which mixes baro and GPS, preferring the baro when
 * they disagree) would stay near zero. Inverse of pressureToAltitude() in
 * sensors/barometer.c. */
void bfb_baro_update(float up)
{
    const float pressure = 101325.0f * powf(1.0f - up * 100.0f / 4433000.0f, 1.0f / 0.190295f);
    virtualBaroSet((int32_t)lrintf(pressure), 2500);
}

/* ---------------------------------------------------------------------------
 * The simulated RC link (D36). Betaflight takes link statistics from the
 * receiver protocol only when the sources say so (rx/crsf.c sets them for a
 * CRSF receiver); the SITL receiver has none, so the sources are switched to
 * CRSF while libfpv reports, and restored after. Its OSD elements are shown
 * only meanwhile (patch 0014).
 * ------------------------------------------------------------------------- */

static bool rcLinkReporting = false;
static rssiSource_e rcLinkSavedRssiSource;
static linkQualitySource_e rcLinkSavedLqSource;

void bfb_rc_link_stats(int lq, int rfMode, int rssiDbm, int snrDb, int txPowerMw)
{
    if (!rcLinkReporting) {
        rcLinkSavedRssiSource = rssiSource;
        rcLinkSavedLqSource = linkQualitySource;
        rcLinkReporting = true;
    }
    /* every step: a reboot's rxInit may have set them again */
    rssiSource = RSSI_SOURCE_RX_PROTOCOL_CRSF;
    linkQualitySource = LQ_SOURCE_RX_PROTOCOL_CRSF;
    osdHostLinkElementsHidden = false;  /* the OSD shows the link's elements */
    rssiDbm = constrain(rssiDbm, -130, 0);
    setRssi(scaleRange(rssiDbm, -130, 0, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_PROTOCOL_CRSF);
#ifdef USE_RX_RSSI_DBM
    setRssiDbm(rssiDbm, RSSI_SOURCE_RX_PROTOCOL_CRSF);
#endif
#ifdef USE_RX_RSNR
    setRsnr(constrain(snrDb, -30, 20));
#else
    UNUSED(snrDb);
#endif
    setLinkQualityDirect(constrain(lq, 0, 100));
#ifdef USE_RX_LINK_QUALITY_INFO
    rxSetRfMode(constrain(rfMode, 0, 255));
#else
    UNUSED(rfMode);
#endif
#ifdef USE_RX_LINK_UPLINK_POWER
    rxSetUplinkTxPwrMw(constrain(txPowerMw, 0, 65535));
#else
    UNUSED(txPowerMw);
#endif
}

void bfb_rc_link_release(void)
{
    if (!rcLinkReporting) {
        return;
    }
    rcLinkReporting = false;
    osdHostLinkElementsHidden = true;  /* nothing real to show */
    rssiSource = rcLinkSavedRssiSource;
    linkQualitySource = rcLinkSavedLqSource;
    setRssiDirect(0, rssiSource);
#ifdef USE_RX_LINK_QUALITY_INFO
    rxSetRfMode(0);
#endif
#ifdef USE_RX_LINK_UPLINK_POWER
    rxSetUplinkTxPwrMw(0);
#endif
}
