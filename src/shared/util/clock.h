// libfpv - the monotonic wall clock the client and the simulation share.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <cstdint>

namespace fpv {

// Seconds since the clock's epoch, monotonic. The epoch is the first call
// in this process, or the one setWallEpoch gave.
double wallSeconds();

// The epoch as a std::chrono::steady_clock count in nanoseconds. Split
// builds give the client's to libfpv-server, so wall times (the snapshots')
// mean the same in both processes: the steady clock is system-wide.
int64_t wallEpoch();
void setWallEpoch(int64_t steadyNanoseconds);

}  // namespace fpv
