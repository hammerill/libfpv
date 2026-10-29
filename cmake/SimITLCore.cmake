# Builds SimITL (Betaflight SITL + SimITL physics) as a static, position
# independent library that libfpv links into its own shared library.
#
# This mirrors third_party/SimITL/extern/CMakeLists.txt and
# third_party/SimITL/src/CMakeLists.txt, but points at the flattened
# submodules in third_party/ (HTTPS, no nested SSH submodules) and never
# builds SimITL's executables or its shared-memory client library.

set(SIMITL_DIR      ${PROJECT_SOURCE_DIR}/third_party/SimITL)
set(BF_DIR          ${PROJECT_SOURCE_DIR}/third_party/betaflight)
set(BFEXT_DIR       ${SIMITL_DIR}/extern/betaflightext)
set(LWS_DIR         ${PROJECT_SOURCE_DIR}/third_party/libwebsockets)
set(FMT_DIR         ${PROJECT_SOURCE_DIR}/third_party/fmt)

# --- Patch series ------------------------------------------------------------
# Patches in patches/<name>/ are applied in order to a clean submodule. A
# submodule with local changes is assumed to carry the series already (or to
# be under development with tools/new-patch.sh) and is left untouched.
function(libfpv_apply_patches source_dir patch_dir)
  file(GLOB _patches "${patch_dir}/*.patch")
  list(SORT _patches)
  if(NOT _patches)
    return()
  endif()
  execute_process(COMMAND git diff --quiet WORKING_DIRECTORY "${source_dir}" RESULT_VARIABLE _dirty)
  if(NOT _dirty EQUAL 0)
    # A tree patched before newer patches were added to the series would
    # otherwise silently miss them. Find them by taking the series off a
    # copy of the tree's index, last patch first: a patch that comes off is
    # in the tree. (One patch at a time against the tree does not work: a
    # later patch that edits its lines makes it fit neither way.)
    execute_process(COMMAND git rev-parse --git-path index WORKING_DIRECTORY "${source_dir}"
      OUTPUT_VARIABLE _index OUTPUT_STRIP_TRAILING_WHITESPACE)
    get_filename_component(_index "${_index}" ABSOLUTE BASE_DIR "${source_dir}")
    get_filename_component(_name "${patch_dir}" NAME)
    set(_tmp_index "${CMAKE_BINARY_DIR}/libfpv-${_name}-patch-check.index")
    file(COPY_FILE "${_index}" "${_tmp_index}")
    set(_git ${CMAKE_COMMAND} -E env GIT_INDEX_FILE=${_tmp_index} git)
    execute_process(COMMAND ${_git} add -A WORKING_DIRECTORY "${source_dir}" OUTPUT_QUIET ERROR_QUIET)
    set(_reversed ${_patches})
    list(REVERSE _reversed)
    set(_missing)
    foreach(_patch IN LISTS _reversed)
      execute_process(COMMAND ${_git} apply --cached -R --whitespace=nowarn "${_patch}"
        WORKING_DIRECTORY "${source_dir}" RESULT_VARIABLE _rev OUTPUT_QUIET ERROR_QUIET)
      if(NOT _rev EQUAL 0)
        list(PREPEND _missing "${_patch}")
      endif()
    endforeach()
    file(REMOVE "${_tmp_index}")
    foreach(_patch IN LISTS _missing)
      execute_process(COMMAND git apply --whitespace=nowarn "${_patch}"
        WORKING_DIRECTORY "${source_dir}" RESULT_VARIABLE _fwd ERROR_VARIABLE _err)
      if(NOT _fwd EQUAL 0)
        message(FATAL_ERROR "libfpv: ${_patch} fits ${source_dir} neither forward nor in reverse. "
          "Either the patch is broken (for example, it has CRLF lines and the files have LF), "
          "or local changes edit its lines. Capture local changes with tools/new-patch.sh, "
          "or reset the submodule (see \"Updating\" in README.md) and configure again.\n${_err}")
      endif()
      message(STATUS "libfpv: applied missing ${_patch}")
    endforeach()
    message(STATUS "libfpv: ${source_dir} has local changes, assuming the rest of its patch series is applied")
    return()
  endif()
  foreach(_patch IN LISTS _patches)
    execute_process(COMMAND git apply --whitespace=nowarn "${_patch}"
      WORKING_DIRECTORY "${source_dir}" RESULT_VARIABLE _res ERROR_VARIABLE _err)
    if(NOT _res EQUAL 0)
      message(FATAL_ERROR "libfpv: failed to apply ${_patch}:\n${_err}")
    endif()
    message(STATUS "libfpv: applied ${_patch}")
  endforeach()
endfunction()
libfpv_apply_patches(${SIMITL_DIR} ${PROJECT_SOURCE_DIR}/patches/simitl)
libfpv_apply_patches(${BF_DIR} ${PROJECT_SOURCE_DIR}/patches/betaflight)

# --- libwebsockets (static, no TLS, no test apps) ----------------------------
# Not in browser builds: a web page cannot run the Configurator's server.
if(NOT EMSCRIPTEN)
set(LWS_WITH_MINIMAL_EXAMPLES OFF CACHE BOOL "" FORCE)
set(LWS_WITHOUT_TESTAPPS ON CACHE BOOL "" FORCE)
set(LWS_WITHOUT_TEST_SERVER ON CACHE BOOL "" FORCE)
set(LWS_WITHOUT_TEST_SERVER_EXTPOLL ON CACHE BOOL "" FORCE)
set(LWS_WITHOUT_TEST_PING ON CACHE BOOL "" FORCE)
set(LWS_WITHOUT_TEST_CLIENT ON CACHE BOOL "" FORCE)
set(LWS_WITH_SSL OFF CACHE BOOL "" FORCE)
set(LWS_WITH_SHARED OFF CACHE BOOL "" FORCE)
set(LWS_WITH_STATIC ON CACHE BOOL "" FORCE)
set(LWS_WITH_EXPORT_LWSTARGETS OFF CACHE BOOL "" FORCE)
set(DISABLE_WERROR ON CACHE BOOL "" FORCE)
if(APPLE)
  # lws's configure check accepts pipe2() on macOS, but the symbol resolves
  # to NULL at run time and lws_create_context crashes; use plain pipe().
  set(LWS_HAVE_PIPE2 "" CACHE INTERNAL "")
endif()
add_subdirectory(${LWS_DIR} ${CMAKE_BINARY_DIR}/third_party/libwebsockets EXCLUDE_FROM_ALL)
set_property(TARGET websockets PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET websockets PROPERTY C_VISIBILITY_PRESET hidden)
endif()

# --- Betaflight + SimITL extensions ------------------------------------------
set(BF_MAIN_SOURCES
  build/build_config.c build/debug.c build/version.c
  common/maths.c common/streambuf.c common/crc.c common/filter.c
  common/sensor_alignment.c common/bitarray.c common/printf.c
  common/printf_serial.c common/time.c common/typeconversion.c
  common/encoding.c common/vector.c common/gps_conversion.c
  config/config.c config/feature.c config/config_streamer.c
  config/config_eeprom.c config/simplified_tuning.c
  drivers/accgyro/accgyro_virtual.c drivers/accgyro/gyro_sync.c
  drivers/barometer/barometer_virtual.c drivers/timer_common.c
  drivers/motor.c drivers/bus.c drivers/dshot.c drivers/serial.c
  drivers/buf_writer.c drivers/stack_check.c drivers/io.c drivers/display.c
  drivers/light_led.c drivers/rx/rx_pwm.c drivers/pinio.c drivers/resource.c
  drivers/bus_i2c_busdev.c drivers/bus_i2c_soft.c drivers/vtx_common.c
  drivers/vtx_table.c drivers/pwm_output.c
  fc/init.c
  rx/rx.c rx/msp.c rx/msp_override.c rx/rc_stats.c
  sensors/initialisation.c sensors/compass.c sensors/acceleration.c
  sensors/acceleration_init.c sensors/gyro.c sensors/gyro_init.c
  sensors/boardalignment.c sensors/barometer.c sensors/voltage.c
  sensors/current.c sensors/adcinternal.c sensors/esc_sensor.c
  fc/core.c fc/runtime_config.c fc/rc_controls.c fc/tasks.c fc/rc_modes.c
  fc/controlrate_profile.c fc/rc.c fc/rc_adjustments.c fc/board_info.c
  fc/stats.c fc/dispatch.c
  flight/failsafe.c flight/mixer.c flight/mixer_init.c
  flight/mixer_tricopter.c flight/servos.c flight/servos_tricopter.c
  flight/imu.c flight/pid.c flight/pid_init.c flight/position.c
  flight/gps_rescue_multirotor.c flight/autopilot_multirotor.c
  io/beeper.c io/statusindicator.c io/rcdevice_cam.c io/rcdevice.c io/gps.c
  io/displayport_msp.c io/serial_resource.c io/gps_virtual.c
  cms/cms.c cms/cms_menu_blackbox.c cms/cms_menu_failsafe.c
  cms/cms_menu_firmware.c cms/cms_menu_gps_rescue_multirotor.c
  cms/cms_menu_imu.c cms/cms_menu_ledstrip.c cms/cms_menu_main.c
  cms/cms_menu_misc.c cms/cms_menu_osd.c cms/cms_menu_persistent_stats.c
  cms/cms_menu_power.c cms/cms_menu_saveexit.c cms/cms_menu_vtx_common.c
  cms/cms_menu_vtx_msp.c cms/cms_menu_vtx_rtc6705.c
  cms/cms_menu_vtx_smartaudio.c cms/cms_menu_vtx_tramp.c
  osd/osd.c osd/osd_elements.c osd/osd_warnings.c
  msp/msp.c msp/msp_box.c msp/msp_serial.c msp/msp_build_info.c
  cli/cli.c
  blackbox/blackbox.c blackbox/blackbox_encoding.c
)
list(TRANSFORM BF_MAIN_SOURCES PREPEND "${BF_DIR}/src/main/")
file(GLOB BF_PG_SOURCES "${BF_DIR}/src/main/pg/*.c")
file(GLOB BFEXT_SOURCES
  "${BFEXT_DIR}/src/main/target/simITL/*.c"
  "${BFEXT_DIR}/src/main/cli/*.c"
  "${BFEXT_DIR}/src/main/common/*.c"
  "${BFEXT_DIR}/src/main/io/*.c"
  "${BFEXT_DIR}/src/main/scheduler/*.c"
  "${BFEXT_DIR}/src/main/sensors/*.c"
  "${BFEXT_DIR}/src/main/blackbox/*.c"
)
# SimITL's string_light.c re-implements libc's ctype/strcasecmp functions;
# with every object linked that collides with msvcrt. The platform libc is
# used instead, plus src/core/bf_compat.c for what MinGW lacks.
list(FILTER BFEXT_SOURCES EXCLUDE REGEX "string_light\\.c$")
list(APPEND BFEXT_SOURCES
  "${PROJECT_SOURCE_DIR}/src/core/bf_compat.c"
  "${BFEXT_DIR}/src/main/drivers/pwm_output_fake.c"
  "${BFEXT_DIR}/src/main/drivers/sound_beeper.c"
  "${BF_DIR}/lib/main/google/olc/olc.c"
)
if(EMSCRIPTEN)
  # UARTs without a server (src/core/serial_ws_web.c)
  list(APPEND BFEXT_SOURCES "${PROJECT_SOURCE_DIR}/src/core/serial_ws_web.c")
else()
  list(APPEND BFEXT_SOURCES "${BFEXT_DIR}/src/main/drivers/serial_ws.c" "${BF_DIR}/lib/main/dyad/dyad.c")
endif()

set(SIMITL_CPP_SOURCES
  ${SIMITL_DIR}/src/main.cpp
  ${SIMITL_DIR}/src/util/SimplexNoise.cpp
  ${SIMITL_DIR}/src/sim/bf.cpp
  ${SIMITL_DIR}/src/sim/physics.cpp
  ${SIMITL_DIR}/src/sim/sim.cpp
  ${PROJECT_SOURCE_DIR}/src/core/bf_bridge.c
)

add_library(simitl_core STATIC
  ${BF_MAIN_SOURCES} ${BF_PG_SOURCES} ${BFEXT_SOURCES} ${SIMITL_CPP_SOURCES})

target_compile_features(simitl_core PUBLIC cxx_std_17)
target_compile_definitions(simitl_core PRIVATE
  _GNU_SOURCE SITL SIMULATOR_BUILD=1 "__TARGET__=\"SimITL\"" "__REVISION__=\"libfpv\""
  HSE_VALUE=8000000 FLASH_SIZE=2048 FMT_HEADER_ONLY=1)
target_include_directories(simitl_core
  PRIVATE
    ${BFEXT_DIR}/src/main
    ${BFEXT_DIR}/src/main/target/simITL
    ${BF_DIR}/lib/main/dyad
    ${BF_DIR}/src/main
    ${BF_DIR}/src/platform/SIMULATOR/include
    ${SIMITL_DIR}/extern
    ${SIMITL_DIR}/src
    ${SIMITL_DIR}/lib
    ${FMT_DIR}/include
  PUBLIC
    ${SIMITL_DIR}/src)
# Betaflight log output goes through libfpv's logger instead of stdout.
target_compile_options(simitl_core PRIVATE
  "$<$<COMPILE_LANGUAGE:C>:-include;${PROJECT_SOURCE_DIR}/src/core/bf_log_redirect.h>"
  $<$<COMPILE_LANGUAGE:C>:-w>)
if(CMAKE_C_COMPILER_ID MATCHES "Clang" OR
   (CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 14))
  # Betaflight/SimITL C code relies on implicit declarations (target.c calls
  # init()), which Clang 16+ and GCC 14+ reject, and on mismatched pointer
  # types (cms_menu_blackbox.c and the other CMS menus), which newer Clang and
  # GCC 14+ reject; older GCC only warns.
  target_compile_options(simitl_core PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types>)
endif()
if(MINGW)
  # Betaflight's structs use GCC bitfield layout; SimITL's C++ reads some of
  # them, so both languages must agree.
  target_compile_options(simitl_core PRIVATE -mno-ms-bitfields $<$<COMPILE_LANGUAGE:CXX>:-Wa,-mbig-obj>)
endif()
set_target_properties(simitl_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  C_VISIBILITY_PRESET hidden
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)
if(EMSCRIPTEN)
  set_target_properties(simitl_core PROPERTIES POSITION_INDEPENDENT_CODE OFF)
  target_compile_options(simitl_core PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fwasm-exceptions>)
else()
  target_link_libraries(simitl_core PRIVATE websockets)
  target_include_directories(simitl_core PRIVATE ${LWS_DIR}/include ${CMAKE_BINARY_DIR}/third_party/libwebsockets/include)
endif()

# Betaflight's parameter-group registry lives in a dedicated linker section.
# ELF and PE need the SITL linker script to emit the start/end symbols;
# Mach-O gets them natively from section$start$/section$end$ (see pg.h).
set(SIMITL_LINKER_SCRIPT ${BF_DIR}/src/platform/SIMULATOR/link/sitl.ld)
