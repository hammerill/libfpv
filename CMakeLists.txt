cmake_minimum_required(VERSION 3.24)

# The version lives only in include/libfpv.h; bump it there.
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/include/libfpv.h" LIBFPV_VERSION_LINES
  REGEX "^#define LIBFPV_VERSION_(MAJOR|MINOR|PATCH) +[0-9]+$")
foreach(part MAJOR MINOR PATCH)
  string(REGEX MATCH "LIBFPV_VERSION_${part} +([0-9]+)" _ "${LIBFPV_VERSION_LINES}")
  set(LIBFPV_VERSION_${part} "${CMAKE_MATCH_1}")
  if(LIBFPV_VERSION_${part} STREQUAL "")
    message(FATAL_ERROR "Can't read LIBFPV_VERSION_${part} from include/libfpv.h")
  endif()
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS include/libfpv.h)

project(libfpv
  VERSION ${LIBFPV_VERSION_MAJOR}.${LIBFPV_VERSION_MINOR}.${LIBFPV_VERSION_PATCH}
  LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LIBFPV_WITH_SDL "Read radios and gamepads with an embedded SDL3" ON)
option(LIBFPV_WITH_OVERLAY "Build the Dear ImGui overlay" ON)
option(LIBFPV_BUILD_TESTS "Build the test suite" ON)
option(LIBFPV_BUILD_EXAMPLES "Build the examples" ON)
option(LIBFPV_KAMIKAZE "Kamikaze FPV: warhead with bomb arm and explode switches (docs/INTEGRATION.md#kamikaze-builds)" OFF)
option(LIBFPV_SPLIT_PROCESS "Client-only libfpv: the simulation runs in libfpv-server (docs/DECISIONS.md, D37)" OFF)
option(LIBFPV_KAMIKAZE_PROMOTE "Kamikaze tab that points to the Kamikaze version instead (docs/INTEGRATION.md#kamikaze-builds)" OFF)
if(LIBFPV_KAMIKAZE AND LIBFPV_KAMIKAZE_PROMOTE)
  # both fill the overlay's Kamikaze tab
  message(FATAL_ERROR "LIBFPV_KAMIKAZE and LIBFPV_KAMIKAZE_PROMOTE can't both be ON: pick one")
endif()
if(EMSCRIPTEN)
  # Browser builds read radios and gamepads through the Gamepad API (docs/WEB.md)
  set(LIBFPV_WITH_SDL OFF)
endif()
if(LIBFPV_SPLIT_PROCESS)
  if(EMSCRIPTEN)
    message(FATAL_ERROR "LIBFPV_SPLIT_PROCESS: a web page cannot start libfpv-server; build without it")
  endif()
  message(STATUS "libfpv: split build (the simulation in libfpv-server, OSD fonts in libfpv-fonts/, shipped presets in libfpv-presets/)")
endif()

include(cmake/SimITLCore.cmake)
include(cmake/EmbedFiles.cmake)

# --- embedded assets -----------------------------------------------------------
file(GLOB_RECURSE LIBFPV_PRESET_FILES CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/assets/presets/*.json
  ${PROJECT_SOURCE_DIR}/assets/presets/*.diff.txt)
list(SORT LIBFPV_PRESET_FILES)
set(LIBFPV_GEN_DIR ${CMAKE_BINARY_DIR}/generated)
file(GLOB LIBFPV_FONT_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/assets/fonts/*.mcm)
if(NOT LIBFPV_SPLIT_PROCESS)
  # The shipped presets (some exported from SimITL's) and Betaflight's OSD
  # fonts, both GPL-3.0, inside the library; split builds ship them as
  # files instead (below)
  libfpv_embed_files(${LIBFPV_GEN_DIR}/embedded_presets.cpp kEmbeddedPresets
    ${PROJECT_SOURCE_DIR}/assets/presets ${LIBFPV_PRESET_FILES})
  libfpv_embed_files(${LIBFPV_GEN_DIR}/embedded_fonts.cpp kEmbeddedFonts
    ${PROJECT_SOURCE_DIR}/assets/fonts ${LIBFPV_FONT_FILES})
endif()

# --- libfpv --------------------------------------------------------------------
# Three parts with their own include roots (docs/DECISIONS.md, D37):
#   src/shared  libfpv's own code both sides use (presets, collision, the
#               links, the Server interface); never Betaflight or SimITL
#   src/server  the simulation server: Betaflight, SimITL's flight model and
#               the real-time loop (GPL-3.0-or-later, like Betaflight)
#   src/client  the C API, presentation, overlay, OSD, audio and input
# The client and the shared code never see the server's or the flight
# model's headers; tools/check-boundary.cmake also rejects any include of
# Betaflight or SimITL in them (the license_boundary test).
#   src/web     the browser runtime (libfpv_web.js); its path is public:
#               non-CMake web builds pass it to --js-library

# Build options every part compiles with.
add_library(fpv_options INTERFACE)
target_compile_definitions(fpv_options INTERFACE LIBFPV_BUILD FMT_HEADER_ONLY=1)
target_include_directories(fpv_options INTERFACE
  ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/shared)
if(LIBFPV_KAMIKAZE)
  target_compile_definitions(fpv_options INTERFACE LIBFPV_KAMIKAZE=1)
endif()
if(LIBFPV_KAMIKAZE_PROMOTE)
  target_compile_definitions(fpv_options INTERFACE LIBFPV_KAMIKAZE_PROMOTE=1)
endif()
if(EMSCRIPTEN)
  target_compile_options(fpv_options INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-fwasm-exceptions>)
endif()

function(libfpv_part target)
  set_target_properties(${target} PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    POSITION_INDEPENDENT_CODE ON)
  if(EMSCRIPTEN)
    set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE OFF)
  endif()
  target_link_libraries(${target} PRIVATE fpv_options)
endfunction()

add_library(fpv_shared OBJECT
  src/shared/config/bf_config.cpp
  src/shared/config/preset.cpp
  src/shared/rc/rc_link.cpp
  src/shared/util/clock.cpp
  src/shared/util/fs.cpp
  src/shared/util/log.cpp
  src/shared/video/video_link.cpp
  src/shared/world/collision.cpp
  src/shared/world/drone_shape.cpp
  src/shared/world/wind.cpp)
libfpv_part(fpv_shared)
target_include_directories(fpv_shared PRIVATE ${PROJECT_SOURCE_DIR}/third_party)
if(LIBFPV_SPLIT_PROCESS)
  # the Server calls as messages over loopback sockets, both ends
  target_sources(fpv_shared PRIVATE src/shared/ipc/socket.cpp src/shared/simapi/wire.cpp)
endif()

add_library(fpv_server OBJECT
  src/server/sim/flight.cpp
  src/server/sim/runner.cpp
  src/server/sim/sim_server.cpp)
if(LIBFPV_KAMIKAZE)
  target_sources(fpv_server PRIVATE src/server/sim/kamikaze.cpp)
endif()
libfpv_part(fpv_server)
target_include_directories(fpv_server PRIVATE
  ${PROJECT_SOURCE_DIR}/src/server
  ${PROJECT_SOURCE_DIR}/src/shared  # before SimITL's own util/ and sim/
  ${SIMITL_DIR}/src
  ${SIMITL_DIR}/lib
  ${FMT_DIR}/include)

set(LIBFPV_CLIENT_SOURCES
  src/client/api.cpp
  src/client/api_present.cpp
  src/client/frame.cpp
  src/client/input/input.cpp
  src/client/input/sdl_backend.cpp
  src/client/input/web_backend.cpp
  src/client/osd/osd.cpp
  src/client/audio/synth.cpp
  src/client/util/platform.cpp
  ${LIBFPV_GEN_DIR}/build_info.cpp
)
if(NOT LIBFPV_SPLIT_PROCESS)
  list(APPEND LIBFPV_CLIENT_SOURCES ${LIBFPV_GEN_DIR}/embedded_presets.cpp ${LIBFPV_GEN_DIR}/embedded_fonts.cpp)
endif()

# The About tab's build references (commits, dirty flags, dates), refreshed
# on every build: a commit made without re-running CMake still shows.
set(_build_info_args
  -DSRC=${PROJECT_SOURCE_DIR} -DOUT=${LIBFPV_GEN_DIR}/build_info.cpp -DVERSION=${PROJECT_VERSION}
  -DBF_DIR=${BF_DIR} -DSIMITL_DIR=${SIMITL_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} ${_build_info_args} -P ${PROJECT_SOURCE_DIR}/cmake/BuildInfo.cmake)
add_custom_target(libfpv_build_info
  COMMAND ${CMAKE_COMMAND} ${_build_info_args} -P ${PROJECT_SOURCE_DIR}/cmake/BuildInfo.cmake
  BYPRODUCTS ${LIBFPV_GEN_DIR}/build_info.cpp
  COMMENT "libfpv: build references"
  VERBATIM)

# libfpv itself: the client, the shared code and, in the game's process, the
# server. Split builds leave the server out: it runs in libfpv-server.
if(LIBFPV_SPLIT_PROCESS)
  list(APPEND LIBFPV_CLIENT_SOURCES src/client/remote_server.cpp)
  set(LIBFPV_PART_OBJECTS $<TARGET_OBJECTS:fpv_shared>)
else()
  set(LIBFPV_PART_OBJECTS $<TARGET_OBJECTS:fpv_shared> $<TARGET_OBJECTS:fpv_server>)
endif()
if(EMSCRIPTEN)
  # linked statically into the game's WebAssembly module
  add_library(fpv STATIC ${LIBFPV_CLIENT_SOURCES} ${LIBFPV_PART_OBJECTS})
else()
  add_library(fpv SHARED ${LIBFPV_CLIENT_SOURCES} ${LIBFPV_PART_OBJECTS})
endif()
add_library(libfpv::fpv ALIAS fpv)
add_dependencies(fpv libfpv_build_info)
libfpv_part(fpv)
set_target_properties(fpv PROPERTIES
  OUTPUT_NAME fpv
  VERSION ${PROJECT_VERSION}
  SOVERSION 0)
# the About tab's build type (multi-config generators decide it at build time)
target_compile_definitions(fpv PRIVATE LIBFPV_BUILD_CONFIG=$<CONFIG>)
target_include_directories(fpv
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src/client
    ${PROJECT_SOURCE_DIR}/third_party)
if(LIBFPV_KAMIKAZE)
  # public: the kamikaze API in libfpv.h exists only with this definition
  target_compile_definitions(fpv PUBLIC LIBFPV_KAMIKAZE=1)
endif()
if(LIBFPV_SPLIT_PROCESS)
  target_compile_definitions(fpv PRIVATE LIBFPV_SPLIT_PROCESS=1)
  # The OSD fonts and the shipped presets as files in libfpv-fonts/ and
  # libfpv-presets/ next to the library, each with its README and the GPL
  # text; a game ships both folders with libfpv.
  target_compile_definitions(fpv PRIVATE LIBFPV_FONTS_EXTERNAL=1 LIBFPV_PRESETS_EXTERNAL=1)
  set(_out $<TARGET_FILE_DIR:fpv>)
  set(_preset_copies "")
  foreach(_f ${LIBFPV_PRESET_FILES})
    file(RELATIVE_PATH _rel ${PROJECT_SOURCE_DIR}/assets/presets ${_f})
    get_filename_component(_reldir ${_rel} DIRECTORY)
    list(APPEND _preset_copies
      COMMAND ${CMAKE_COMMAND} -E make_directory ${_out}/libfpv-presets/${_reldir}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_f} ${_out}/libfpv-presets/${_rel})
  endforeach()
  add_custom_command(TARGET fpv POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${_out}/libfpv-fonts
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIBFPV_FONT_FILES}
            ${PROJECT_SOURCE_DIR}/assets/fonts/README.md ${_out}/libfpv-fonts/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/LICENSE
            ${_out}/libfpv-fonts/LICENSE.txt
    # a fresh copy, so a preset removed from assets/ doesn't linger
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${_out}/libfpv-presets
    ${_preset_copies}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/assets/presets/README.md
            ${_out}/libfpv-presets/README.md
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PROJECT_SOURCE_DIR}/LICENSE
            ${_out}/libfpv-presets/LICENSE.txt
    VERBATIM)
endif()

# Betaflight registers its parameter groups in a linker section that nothing
# references by name: link every object of the core.
if(EMSCRIPTEN)
  # libfpv.a becomes one object holding libfpv and the whole core, so the
  # game links it like any static library: whatever it calls pulls in every
  # parameter group, with no --whole-archive in the game's build. Its hidden
  # symbols are made local, as they are inside the native shared library,
  # so Betaflight's init() or libfpv's ImGui never meet the game's own.
  # (the stamp makes a changed core re-archive libfpv)
  set(_core_stamp ${LIBFPV_GEN_DIR}/core_stamp.c)
  if(NOT EXISTS ${_core_stamp})
    file(WRITE ${_core_stamp} "/* touched whenever libsimitl_core.a changes */\n")
  endif()
  add_custom_command(OUTPUT ${_core_stamp}
    COMMAND ${CMAKE_COMMAND} -E touch ${_core_stamp}
    DEPENDS $<TARGET_FILE:simitl_core>)
  target_sources(fpv PRIVATE ${_core_stamp})
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(_fpv_all ${CMAKE_CURRENT_BINARY_DIR}/libfpv-all.o)
  add_custom_command(TARGET fpv POST_BUILD
    # (warnings are fatal: a call through a mismatched signature, like an
    # implicitly declared function, would trap at run time)
    COMMAND ${CMAKE_C_COMPILER} -r -Wl,--fatal-warnings -Wl,--whole-archive $<TARGET_FILE:fpv>
            $<TARGET_FILE:simitl_core> -Wl,--no-whole-archive -o ${_fpv_all}.tmp
    COMMAND ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/tools/wasm-localize-hidden.py ${_fpv_all}.tmp ${_fpv_all}
    COMMAND ${CMAKE_COMMAND} -E rm -f $<TARGET_FILE:fpv> ${_fpv_all}.tmp
    COMMAND ${CMAKE_AR} qc $<TARGET_FILE:fpv> ${_fpv_all}
    COMMAND ${CMAKE_RANLIB} $<TARGET_FILE:fpv>
    VERBATIM)
  # The game's link needs: C++ exceptions as WebAssembly instructions (the
  # fastest kind; libfpv's JSON parsing catches them; fpv_options compiles
  # with them), the IndexedDB file system and libfpv's browser runtime
  # (docs/WEB.md).
  target_link_options(fpv INTERFACE -fwasm-exceptions -lidbfs.js
    "SHELL:--js-library ${PROJECT_SOURCE_DIR}/src/web/libfpv_web.js")
elseif(NOT LIBFPV_SPLIT_PROCESS)
  target_link_libraries(fpv PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,simitl_core>" websockets)
endif()

# Betaflight's parameter groups need SimITL's linker script, wherever the
# core is linked: libfpv, or libfpv-server in split builds.
set(_core_script "")
if(NOT LIBFPV_SPLIT_PROCESS AND (CMAKE_SYSTEM_NAME STREQUAL "Linux" OR WIN32))
  set(_core_script -T${SIMITL_LINKER_SCRIPT})
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(fpv PRIVATE
    ${_core_script}
    -Wl,--version-script=${PROJECT_SOURCE_DIR}/cmake/libfpv.map
    -Wl,--exclude-libs,ALL
    -Wl,-z,nodelete
    -Wl,-Bsymbolic
    -Wl,--no-undefined)
  target_link_libraries(fpv PRIVATE pthread dl m)
elseif(APPLE)
  target_link_options(fpv PRIVATE
    -Wl,-exported_symbols_list,${PROJECT_SOURCE_DIR}/cmake/libfpv.exp)
elseif(WIN32)
  # libfpv.def lets MSVC hosts make an import library: lib /def:libfpv.def /machine:x64
  target_link_options(fpv PRIVATE ${_core_script} -static-libgcc -static-libstdc++ -static
    -Wl,--output-def,${CMAKE_BINARY_DIR}/libfpv.def)
  target_link_libraries(fpv PRIVATE ws2_32 wsock32 winmm iphlpapi userenv)
endif()

if(LIBFPV_SPLIT_PROCESS)
  # libfpv-server: the simulation, next to the library (the game's libfpv
  # starts it from there). GPL-3.0-or-later, like Betaflight.
  add_executable(libfpv-server src/server/main.cpp $<TARGET_OBJECTS:fpv_shared> $<TARGET_OBJECTS:fpv_server>)
  libfpv_part(libfpv-server)
  target_include_directories(libfpv-server PRIVATE
    ${PROJECT_SOURCE_DIR}/src/server
    ${PROJECT_SOURCE_DIR}/src/shared  # before SimITL's own util/ and sim/
    ${SIMITL_DIR}/src
    ${SIMITL_DIR}/lib
    ${FMT_DIR}/include)
  # in libfpv's own folder (a DLL is a runtime file, a .so a library file)
  if(WIN32)
    get_target_property(_fpv_dir fpv RUNTIME_OUTPUT_DIRECTORY)
  else()
    get_target_property(_fpv_dir fpv LIBRARY_OUTPUT_DIRECTORY)
  endif()
  if(NOT _fpv_dir)
    set(_fpv_dir ${CMAKE_CURRENT_BINARY_DIR})
  endif()
  set_target_properties(libfpv-server PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${_fpv_dir})
  target_link_libraries(libfpv-server PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,simitl_core>" websockets)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_options(libfpv-server PRIVATE -T${SIMITL_LINKER_SCRIPT})
    target_link_libraries(libfpv-server PRIVATE pthread dl m)
  elseif(WIN32)
    target_link_options(libfpv-server PRIVATE -T${SIMITL_LINKER_SCRIPT} -static-libgcc -static-libstdc++ -static)
    target_link_libraries(libfpv-server PRIVATE ws2_32 wsock32 winmm iphlpapi userenv)
  endif()
  # built with libfpv: a game (or a test) that links libfpv gets both
  add_dependencies(fpv libfpv-server)
endif()

if(LIBFPV_WITH_OVERLAY)
  set(IMGUI_DIR ${PROJECT_SOURCE_DIR}/third_party/imgui)
  target_sources(fpv PRIVATE
    ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp
    src/client/overlay/overlay.cpp src/client/overlay/bf_cache.cpp src/client/overlay/radio_wizard.cpp
    src/client/overlay/collision_view.cpp)
  target_include_directories(fpv PRIVATE ${IMGUI_DIR})
  # 32-bit indices: the collision view draws far more than 65536 vertices
  # in one list, and fpv_draw_cmd indices are 32-bit anyway
  target_compile_definitions(fpv PRIVATE "ImDrawIdx=unsigned int")
else()
  target_sources(fpv PRIVATE src/client/overlay/overlay_stub.cpp)
endif()

if(LIBFPV_KAMIKAZE)
  target_sources(fpv PRIVATE src/client/api_kamikaze.cpp)
endif()

if(LIBFPV_WITH_SDL)
  include(cmake/InternalSDL.cmake)
  target_compile_definitions(fpv PRIVATE LIBFPV_WITH_SDL=1)
  target_link_libraries(fpv PRIVATE libfpv_sdl3)
endif()

# --- tests and examples ----------------------------------------------------------
if(LIBFPV_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
if(LIBFPV_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
