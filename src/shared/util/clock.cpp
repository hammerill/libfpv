// libfpv - the monotonic wall clock.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "util/clock.h"

#include <atomic>
#include <chrono>
#include <limits>

namespace fpv {

namespace {
constexpr int64_t kUnset = std::numeric_limits<int64_t>::min();
std::atomic<int64_t> g_epoch{kUnset};

int64_t nowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

int64_t wallEpoch() {
    int64_t e = g_epoch.load(std::memory_order_acquire);
    if (e != kUnset) return e;
    int64_t now = nowNs();
    g_epoch.compare_exchange_strong(e, now, std::memory_order_acq_rel);
    return g_epoch.load(std::memory_order_acquire);
}

void setWallEpoch(int64_t steadyNanoseconds) { g_epoch.store(steadyNanoseconds, std::memory_order_release); }

double wallSeconds() { return (double)(nowNs() - wallEpoch()) * 1e-9; }

}  // namespace fpv
