add_executable(fpv_tests test_main.cpp)
target_link_libraries(fpv_tests PRIVATE libfpv::fpv)
target_include_directories(fpv_tests PRIVATE ${PROJECT_SOURCE_DIR}/src/client)  # overlay/osd_element_names.h
target_compile_definitions(fpv_tests PRIVATE LIBFPV_TEST_DIFF_DIR="${CMAKE_CURRENT_SOURCE_DIR}/diffs")
if(EMSCRIPTEN)
  # Node: real files (the diff corpus, a temporary data folder)
  target_link_options(fpv_tests PRIVATE -sNODERAWFS=1 -sALLOW_MEMORY_GROWTH=1 -sEXIT_RUNTIME=1)
elseif(UNIX AND NOT APPLE)
  target_link_libraries(fpv_tests PRIVATE dl)
endif()
add_test(NAME fpv_tests COMMAND fpv_tests WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_tests_properties(fpv_tests PROPERTIES TIMEOUT 900)

# Radio wizard and switch mapping, without the library (pure logic).
add_executable(fpv_wizard_tests test_wizard.cpp ${PROJECT_SOURCE_DIR}/src/client/overlay/radio_wizard.cpp)
target_include_directories(fpv_wizard_tests PRIVATE ${PROJECT_SOURCE_DIR}/src/client ${PROJECT_SOURCE_DIR}/src/shared)
target_compile_features(fpv_wizard_tests PRIVATE cxx_std_17)
if(LIBFPV_KAMIKAZE)
  target_compile_definitions(fpv_wizard_tests PRIVATE LIBFPV_KAMIKAZE=1)
endif()
add_test(NAME fpv_wizard_tests COMMAND fpv_wizard_tests)

# The drone sound's synth, without the library: Doppler without the frame rate's wobble.
add_executable(fpv_synth_tests test_synth.cpp ${PROJECT_SOURCE_DIR}/src/client/audio/synth.cpp)
target_include_directories(fpv_synth_tests PRIVATE ${PROJECT_SOURCE_DIR}/src/client ${PROJECT_SOURCE_DIR}/src/shared)
target_compile_features(fpv_synth_tests PRIVATE cxx_std_17)
add_test(NAME fpv_synth_tests COMMAND fpv_synth_tests)

# The video link's model, without the library: budget, fading, the line table.
add_executable(fpv_video_tests test_video.cpp ${PROJECT_SOURCE_DIR}/src/shared/video/video_link.cpp)
target_include_directories(fpv_video_tests PRIVATE ${PROJECT_SOURCE_DIR}/src/shared ${PROJECT_SOURCE_DIR}/include)
target_compile_features(fpv_video_tests PRIVATE cxx_std_17)
add_test(NAME fpv_video_tests COMMAND fpv_video_tests)

# The RC link's model, without the library: budget, packets, LQ, dynamic power.
add_executable(fpv_rc_link_tests test_rc_link.cpp ${PROJECT_SOURCE_DIR}/src/shared/rc/rc_link.cpp
  ${PROJECT_SOURCE_DIR}/src/shared/video/video_link.cpp)
target_include_directories(fpv_rc_link_tests PRIVATE ${PROJECT_SOURCE_DIR}/src/shared ${PROJECT_SOURCE_DIR}/include)
target_compile_features(fpv_rc_link_tests PRIVATE cxx_std_17)
add_test(NAME fpv_rc_link_tests COMMAND fpv_rc_link_tests)

# The input device the pilot picks ("Fly with"), with a fake device backend.
add_executable(fpv_input_tests test_input.cpp ${PROJECT_SOURCE_DIR}/src/client/input/input.cpp
  ${PROJECT_SOURCE_DIR}/src/shared/util/fs.cpp ${PROJECT_SOURCE_DIR}/src/shared/util/log.cpp)
target_include_directories(fpv_input_tests PRIVATE ${PROJECT_SOURCE_DIR}/src/client ${PROJECT_SOURCE_DIR}/src/shared
  ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/third_party)
target_compile_features(fpv_input_tests PRIVATE cxx_std_17)
add_test(NAME fpv_input_tests COMMAND fpv_input_tests)

# The client and the shared code never include Betaflight, SimITL or the
# server's headers (docs/DECISIONS.md, D37).
add_test(NAME license_boundary
  COMMAND ${CMAKE_COMMAND} -DSRC=${PROJECT_SOURCE_DIR} -P ${PROJECT_SOURCE_DIR}/tools/check-boundary.cmake)
