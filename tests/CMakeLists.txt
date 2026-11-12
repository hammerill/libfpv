add_executable(fpv_tests test_main.cpp)
target_link_libraries(fpv_tests PRIVATE libfpv::fpv)
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
add_executable(fpv_wizard_tests test_wizard.cpp ${PROJECT_SOURCE_DIR}/src/overlay/radio_wizard.cpp)
target_include_directories(fpv_wizard_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_features(fpv_wizard_tests PRIVATE cxx_std_17)
if(LIBFPV_KAMIKAZE)
  target_compile_definitions(fpv_wizard_tests PRIVATE LIBFPV_KAMIKAZE=1)
endif()
add_test(NAME fpv_wizard_tests COMMAND fpv_wizard_tests)

# The drone sound's synth, without the library: Doppler without the frame rate's wobble.
add_executable(fpv_synth_tests test_synth.cpp ${PROJECT_SOURCE_DIR}/src/audio/synth.cpp)
target_include_directories(fpv_synth_tests PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_features(fpv_synth_tests PRIVATE cxx_std_17)
add_test(NAME fpv_synth_tests COMMAND fpv_synth_tests)

# The video link's model, without the library: budget, fading, the line table.
add_executable(fpv_video_tests test_video.cpp ${PROJECT_SOURCE_DIR}/src/video/video_link.cpp)
target_include_directories(fpv_video_tests PRIVATE ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/third_party/SimITL/src)
target_compile_features(fpv_video_tests PRIVATE cxx_std_17)
add_test(NAME fpv_video_tests COMMAND fpv_video_tests)
