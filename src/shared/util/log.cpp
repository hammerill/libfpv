// libfpv - logging to the host's callback.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "util/log.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

namespace fpv {

namespace {
std::mutex g_logMutex;
LogSink g_sink = nullptr;
void *g_user = nullptr;
std::atomic<int> g_minLevel{(int)LogLevel::Info};

// Betaflight/SimITL print partial lines; assemble them per line.
thread_local std::string t_pending;
}  // namespace

void logSetSink(LogSink sink, void *user, LogLevel minLevel) {
    std::lock_guard<std::mutex> lock(g_logMutex);
    g_sink = sink;
    g_user = user;
    g_minLevel = (int)minLevel;
}

LogLevel logMinLevel() { return (LogLevel)g_minLevel.load(); }

static void emit(LogLevel level, const char *msg) {
    if ((int)level < g_minLevel.load()) return;
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_sink) g_sink(g_user, (int)level, msg);
}

void logWriteV(LogLevel level, const char *fmt, va_list args) {
    if ((int)level < g_minLevel.load()) return;
    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, args);
    emit(level, buf);
}

void logWrite(LogLevel level, const char *fmt, ...) {
    va_list a;
    va_start(a, fmt);
    logWriteV(level, fmt, a);
    va_end(a);
}

// Output of embedded Betaflight / SimITL / libwebsockets code.
static void embeddedOutput(bool isError, const char *fmt, va_list args) {
    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, args);
    t_pending += buf;
    size_t nl;
    while ((nl = t_pending.find('\n')) != std::string::npos) {
        std::string line = t_pending.substr(0, nl);
        t_pending.erase(0, nl + 1);
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line.empty()) continue;
        // per-word EEPROM writes and similar chatter are debug-only
        bool chatter = line.rfind("[FLASH_ProgramWord]", 0) == 0;
        LogLevel lvl = chatter ? LogLevel::Debug : (isError ? LogLevel::Info : LogLevel::Debug);
        std::string tagged = "[betaflight] " + line;
        emit(lvl, tagged.c_str());
    }
    if (t_pending.size() > 4096) t_pending.clear();
}

}  // namespace fpv

extern "C" int libfpv_bf_printf(const char *fmt, ...) {
    va_list a;
    va_start(a, fmt);
    fpv::embeddedOutput(false, fmt, a);
    va_end(a);
    return 0;
}

extern "C" int libfpv_bf_fprintf(FILE *stream, const char *fmt, ...) {
    va_list a;
    va_start(a, fmt);
    int r = 0;
    if (stream == stdout || stream == stderr) {
        fpv::embeddedOutput(stream == stderr, fmt, a);
    } else {
        r = vfprintf(stream, fmt, a);
    }
    va_end(a);
    return r;
}
