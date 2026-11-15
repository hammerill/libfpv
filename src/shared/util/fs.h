// libfpv - UTF-8 aware filesystem helpers (Windows paths are UTF-16).
// SPDX-License-Identifier: GPL-3.0-or-later OR MIT
#pragma once

#include <filesystem>
#include <string>

namespace fpv {

std::filesystem::path pathFromUtf8(const std::string &utf8);
std::string pathToUtf8(const std::filesystem::path &p);
bool readTextFile(const std::filesystem::path &p, std::string &out);
// Writes to a temporary file and renames it over the target.
bool writeFileAtomic(const std::filesystem::path &p, const std::string &data);

}  // namespace fpv
