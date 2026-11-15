# libfpv - the client/server boundary (docs/DECISIONS.md, D37): the client
# (src/client) and the shared code (src/shared) reach the simulation only
# through the Server interface, so they never include Betaflight, SimITL or
# the server's own headers. Their include roots already leave those out;
# this also catches a path through third_party/ (the client's root for
# Dear ImGui and nlohmann/json). Every client and shared file is also under
# MIT as well as the GPL ("GPL-3.0-or-later OR MIT", LICENSE-MIT).
#
#   cmake -DSRC=<libfpv source dir> -P tools/check-boundary.cmake
#
# Run by the license_boundary test.

if(NOT SRC)
  message(FATAL_ERROR "pass -DSRC=<libfpv source dir>")
endif()

# Included paths that belong to the server or the projects it embeds.
set(_forbidden
  "betaflight" "SimITL" "simitl"
  "^network/" "^core/" "^fc/" "^flight/" "^pg/" "^rx/" "^sensors/" "^drivers/"
  "^sim/")  # the server's and SimITL's; the client's view of the simulation is simapi/

set(_problems "")
foreach(_part client shared)
  file(GLOB_RECURSE _files "${SRC}/src/${_part}/*.h" "${SRC}/src/${_part}/*.hpp"
       "${SRC}/src/${_part}/*.c" "${SRC}/src/${_part}/*.cpp")
  foreach(_file ${_files})
    file(STRINGS "${_file}" _lines REGEX "^[ \t]*#[ \t]*include[ \t]*[\"<]")
    foreach(_line ${_lines})
      string(REGEX REPLACE "^[ \t]*#[ \t]*include[ \t]*[\"<]([^\">]+)[\">].*$" "\\1" _path "${_line}")
      foreach(_pattern ${_forbidden})
        if(_path MATCHES "${_pattern}")
          file(RELATIVE_PATH _rel "${SRC}" "${_file}")
          string(APPEND _problems "  ${_rel}: #include \"${_path}\"\n")
          break()
        endif()
      endforeach()
    endforeach()
    # every file is under both licenses
    file(STRINGS "${_file}" _spdx REGEX "SPDX-License-Identifier: GPL-3\\.0-or-later OR MIT" LIMIT_COUNT 1)
    if(NOT _spdx)
      file(RELATIVE_PATH _rel "${SRC}" "${_file}")
      string(APPEND _problems "  ${_rel}: no \"SPDX-License-Identifier: GPL-3.0-or-later OR MIT\" line\n")
    endif()
  endforeach()
endforeach()

if(_problems)
  message(FATAL_ERROR "The client and the shared code must not reach the server's side "
                      "(docs/DECISIONS.md, D37):\n${_problems}")
endif()
message(STATUS "client/server boundary: clean")
