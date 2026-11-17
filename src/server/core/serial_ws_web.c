// libfpv - Betaflight's UARTs in browser builds (Emscripten). Replaces
// SimITL's serial_ws.c, whose libwebsockets server cannot exist in a web
// page: a browser can open connections but not accept them, so the
// Betaflight Configurator cannot connect. The ports open as usual so
// Betaflight's serial and MSP code runs unchanged; nothing is ever received,
// and what Betaflight transmits is dropped.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"
#include "io/serial.h"
#include "drivers/serial_ws.h"

static int wsBasePort = 5760;
static const struct serialPortVTable wsVTable;
static wsPort_t wsSerialPorts[SERIAL_PORT_COUNT];

serialPort_t *serialWsOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback,
                           void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    int id = findSerialPortIndexByIdentifier(identifier);
    if (id < 0 || id >= (int)ARRAYLEN(wsSerialPorts)) {
        return NULL;
    }
    wsPort_t *s = &wsSerialPorts[id];
    s->id = id;
    s->port.vTable = &wsVTable;
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;
    return (serialPort_t *)s;
}

static uint32_t wsTotalRxBytesWaiting(const serialPort_t *instance)
{
    UNUSED(instance);
    return 0;
}

static uint32_t wsTotalTxBytesFree(const serialPort_t *instance)
{
    return instance->txBufferSize - 1;
}

static bool isWsTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

static uint8_t wsRead(serialPort_t *instance)
{
    UNUSED(instance);
    return 0;
}

static void wsWrite(serialPort_t *instance, uint8_t ch)
{
    UNUSED(instance);
    UNUSED(ch);
}

void wsDataIn(wsPort_t *instance, uint8_t *ch, int size)
{
    UNUSED(instance);
    UNUSED(ch);
    UNUSED(size);
}

void wsDataOut(wsPort_t *instance) { UNUSED(instance); }
void wsUpdate(void) {}
void wsCancel(void) {}
void wsStop(void) {}
void wsRestart(void) {}
void wsSetEnabled(int enabled) { UNUSED(enabled); }
void wsSetBasePort(int basePort) { wsBasePort = basePort; }
int wsGetBasePort(void) { return wsBasePort; }
int wsGetClientCount(void) { return 0; }
int wsActivePortCount(void) { return 0; }

static const struct serialPortVTable wsVTable = {
    .serialWrite = wsWrite,
    .serialTotalRxWaiting = wsTotalRxBytesWaiting,
    .serialTotalTxFree = wsTotalTxBytesFree,
    .serialRead = wsRead,
    .serialSetBaudRate = NULL,
    .isSerialTransmitBufferEmpty = isWsTransmitBufferEmpty,
    .setMode = NULL,
    .setCtrlLineStateCb = NULL,
    .setBaudRateCb = NULL,
    .writeBuf = NULL,
    .beginWrite = NULL,
    .endWrite = NULL,
};
