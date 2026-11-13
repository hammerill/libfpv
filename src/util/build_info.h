// libfpv - what this binary was built from (overlay About tab).
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace fpv {

struct GitRef {
    const char *hash;  // full commit hash, or "unknown" outside a git checkout
    bool dirty;        // tracked files changed since that commit
    const char *date;  // commit date, YYYY-MM-DD; empty when unknown
};

struct BuildInfo {
    const char *version;  // project(VERSION) in CMakeLists.txt
    GitRef libfpv, betaflight, simitl;
    int betaflightPatches, simitlPatches;  // libfpv's patch series on top
    const char *buildDate;                 // YYYY-MM-DD, UTC (or SOURCE_DATE_EPOCH)
};

// Generated on every build by cmake/BuildInfo.cmake.
const BuildInfo &buildInfo();

}  // namespace fpv
