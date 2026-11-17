// libfpv - UTF-8 aware filesystem helpers.
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#include "util/fs.h"

#include <fstream>
#include <sstream>

namespace fpv {

namespace fs = std::filesystem;

fs::path pathFromUtf8(const std::string &utf8) {
#if defined(__cpp_char8_t)
    return fs::path(std::u8string(utf8.begin(), utf8.end()));
#else
    return fs::u8path(utf8);
#endif
}

std::string pathToUtf8(const fs::path &p) {
#if defined(__cpp_char8_t)
    auto s = p.u8string();
    return std::string(s.begin(), s.end());
#else
    return p.u8string();
#endif
}

bool readTextFile(const fs::path &p, std::string &out) {
    std::ifstream f(p, std::ios::binary);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

bool writeFileAtomic(const fs::path &p, const std::string &data) {
    fs::path tmp = p;
    tmp += ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f.write(data.data(), (std::streamsize)data.size());
        if (!f) return false;
    }
    std::error_code ec;
    fs::rename(tmp, p, ec);
    if (ec) {
        fs::remove(p, ec);
        fs::rename(tmp, p, ec);
    }
    return !ec;
}

}  // namespace fpv
