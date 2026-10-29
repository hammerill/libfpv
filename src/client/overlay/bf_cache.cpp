// libfpv - overlay view of Betaflight settings.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "overlay/bf_cache.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "lib.h"

namespace fpv {

static std::string trimStr(const std::string &s) {
    size_t b = s.find_first_not_of(" \t\r\n"), e = s.find_last_not_of(" \t\r\n");
    return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}

void BfSettingsCache::parse(const std::string &out) {
    std::istringstream in(out);
    std::string line;
    while (std::getline(in, line)) {
        size_t eq = line.find(" = ");
        if (eq == std::string::npos) continue;
        std::string name = trimStr(line.substr(0, eq));
        if (name.empty() || name.find(' ') != std::string::npos) continue;
        m_values[name] = trimStr(line.substr(eq + 3));
    }
}

void BfSettingsCache::refresh(double now, bool force) {
    if (!force && now - m_lastRefresh < 1.0) return;
    m_lastRefresh = now;
    if (m_queries.empty()) return;
    std::string script;
    for (auto &q : m_queries) script += "get " + q + "\n";
    std::string out;
    CliResult rc = runCli(script, out);
    m_busy = rc == CliResult::Busy;
    if (rc == CliResult::Ok) parse(out);
}

std::string BfSettingsCache::str(const std::string &name, const std::string &def) const {
    auto it = m_values.find(name);
    return it == m_values.end() ? def : it->second;
}

int BfSettingsCache::num(const std::string &name, int def) const {
    auto it = m_values.find(name);
    if (it == m_values.end()) return def;
    try {
        return std::stoi(it->second);
    } catch (...) {
        return def;
    }
}

bool BfSettingsCache::run(const std::string &script, std::string *output) {
    std::string out;
    CliResult rc = runCli(script, out);
    m_busy = rc == CliResult::Busy;
    if (output) *output = out;
    if (rc != CliResult::Ok) {
        m_error = m_busy ? "The Betaflight Configurator is in CLI mode; leave its CLI tab and try again."
                         : "Betaflight CLI error";
        return false;
    }
    size_t err = out.find("###ERROR");
    m_error = err == std::string::npos ? "" : out.substr(err, out.find("###", err + 8) - err);
    return true;
}

bool BfSettingsCache::set(const std::string &name, const std::string &value, bool save) {
    std::string script = "set " + name + " = " + value + "\n";
    if (save) script += "save noreboot\n";  // goes live without a reboot (bfb_cli_run)
    std::string out;
    if (!run(script, &out)) return false;
    m_values[name] = value;
    return m_error.empty();
}

static float power3(float x) { return x * x * x; }
static float power5(float x) { return x * x * x * x * x; }

float betaflightRate(const RateParams &p, float rc) {
    float a = std::fabs(rc);
    float out = 0;
    switch (p.type) {
    case 0: {  // BETAFLIGHT
        if (p.expo) {
            float e = p.expo / 100.0f;
            rc = rc * power3(a) * e + rc * (1 - e);
        }
        float rcRate = p.rcRate / 100.0f;
        if (rcRate > 2.0f) rcRate += 14.54f * (rcRate - 2.0f);
        out = 200.0f * rcRate * rc;
        if (p.rate) out *= 1.0f / std::clamp(1.0f - a * (p.rate / 100.0f), 0.01f, 1.0f);
        break;
    }
    case 1:  // RACEFLIGHT
        rc = (1.0f + 0.01f * p.expo * (rc * rc - 1.0f)) * rc;
        out = 10.0f * p.rcRate * rc;
        out *= 1 + a * p.rate * 0.01f;
        break;
    case 2: {  // KISS
        float curve = p.expo / 100.0f;
        float use = 1.0f / std::clamp(1.0f - a * (p.rate / 100.0f), 0.01f, 1.0f);
        float c = (power3(rc) * curve + rc * (1 - curve)) * (p.rcRate / 1000.0f);
        out = std::clamp(2000.0f * use * c, -1998.0f, 1998.0f);
        break;
    }
    case 4: {  // QUICK
        float rcRate = p.rcRate * 2;
        float maxDps = std::max(p.rate * 10, rcRate);
        float e = p.expo / 100.0f;
        float sfc = (maxDps / rcRate - 1) / (maxDps / rcRate);
        if (p.quickRcExpo) {
            float curve = power3(rc) * e + rc * (1 - e);
            float sf = 1.0f / std::clamp(1.0f - a * sfc, 0.01f, 1.0f);
            out = curve * rcRate * sf;
        } else {
            float curve = power3(a) * e + a * (1 - e);
            float sf = 1.0f / std::clamp(1.0f - curve * sfc, 0.01f, 1.0f);
            out = rc * rcRate * sf;
        }
        out = std::clamp(out, -1998.0f, 1998.0f);
        break;
    }
    case 3:  // ACTUAL
    default: {
        float e = p.expo / 100.0f;
        e = a * (power5(rc) * e + rc * (1 - e));
        float centre = p.rcRate * 10.0f;
        float move = std::max(0.0f, p.rate * 10.0f - centre);
        out = rc * centre + move * e;
        break;
    }
    }
    return std::clamp(out, -p.limit, p.limit);
}

}  // namespace fpv
