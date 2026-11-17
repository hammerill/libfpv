// libfpv - logging to the host's callback.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstdarg>

namespace fpv {

enum class LogLevel { Debug = 0, Info = 1, Warn = 2, Error = 3 };

typedef void (*LogSink)(void *user, int level, const char *message);

void logSetSink(LogSink sink, void *user, LogLevel minLevel);
LogLevel logMinLevel();
void logWrite(LogLevel level, const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;
void logWriteV(LogLevel level, const char *fmt, va_list args);

}  // namespace fpv

#define FPV_LOGD(...) ::fpv::logWrite(::fpv::LogLevel::Debug, __VA_ARGS__)
#define FPV_LOGI(...) ::fpv::logWrite(::fpv::LogLevel::Info, __VA_ARGS__)
#define FPV_LOGW(...) ::fpv::logWrite(::fpv::LogLevel::Warn, __VA_ARGS__)
#define FPV_LOGE(...) ::fpv::logWrite(::fpv::LogLevel::Error, __VA_ARGS__)
