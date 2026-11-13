// libfpv - overlay view of Betaflight settings. Reads and writes go
// through Betaflight's CLI on the simulation thread, so the overlay, the
// Configurator and the EEPROM never disagree. Values are re-read every
// second while the overlay is open, picking up Configurator edits.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <map>
#include <set>
#include <string>

namespace fpv {

class BfSettingsCache {
public:
    // Names (or name prefixes, as `get` matches substrings) to keep fresh.
    void watch(const std::string &query) { m_queries.insert(query); }
    void refresh(double now, bool force = false);
    void invalidate() { m_lastRefresh = -1e9; }

    bool has(const std::string &name) const { return m_values.count(name) != 0; }
    std::string str(const std::string &name, const std::string &def = "") const;
    int num(const std::string &name, int def = 0) const;
    // Every setting read so far (the watched queries' matches), by name.
    const std::map<std::string, std::string> &values() const { return m_values; }

    // Writes a value; `save` persists the EEPROM (use on slider release).
    // Returns false while the Configurator owns the CLI.
    bool set(const std::string &name, const std::string &value, bool save);
    bool set(const std::string &name, int value, bool save) { return set(name, std::to_string(value), save); }
    bool run(const std::string &script, std::string *output = nullptr);  // raw CLI

    bool busy() const { return m_busy; }
    const std::string &lastError() const { return m_error; }

private:
    void parse(const std::string &out);
    std::set<std::string> m_queries;
    std::map<std::string, std::string> m_values;
    double m_lastRefresh = -1e9;
    bool m_busy = false;
    std::string m_error;
};

// Betaflight's rate curves (fc/rc.c), deg/s for a stick deflection -1..1.
struct RateParams {
    int type = 3;  // 0 BETAFLIGHT, 1 RACEFLIGHT, 2 KISS, 3 ACTUAL, 4 QUICK
    float rcRate = 7, rate = 67, expo = 0;
    float limit = 1998;
    bool quickRcExpo = false;
};
float betaflightRate(const RateParams &p, float stick);

}  // namespace fpv
