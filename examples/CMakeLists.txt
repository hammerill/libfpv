add_executable(fpv_minimal minimal.c)
target_link_libraries(fpv_minimal PRIVATE libfpv::fpv)

# The standalone "game": its own SDL3 (video + audio) + OpenGL 3.3.
if(EMSCRIPTEN)
  # libfpv has no SDL in browser builds; the game brings its own
  include(${PROJECT_SOURCE_DIR}/cmake/InternalSDL.cmake)
endif()
if(TARGET SDL3::SDL3-static)
  add_executable(fpv_standalone standalone/main.cpp)
  target_include_directories(fpv_standalone PRIVATE standalone ${PROJECT_SOURCE_DIR}/docs/renderers)
  target_link_libraries(fpv_standalone PRIVATE libfpv::fpv SDL3::SDL3-static)
  if(WIN32)
    target_link_options(fpv_standalone PRIVATE -static-libgcc -static-libstdc++ -static)
    set_target_properties(fpv_standalone PROPERTIES WIN32_EXECUTABLE OFF)
  endif()
  # A `diff all` the game imports on its first run (assets/configs)
  set(LIBFPV_STANDALONE_BF_CONFIG "" CACHE FILEPATH "Betaflight diff fpv_standalone starts with")
  if(LIBFPV_STANDALONE_BF_CONFIG)
    get_filename_component(_bf_config ${LIBFPV_STANDALONE_BF_CONFIG} ABSOLUTE BASE_DIR ${PROJECT_SOURCE_DIR})
    if(NOT EXISTS ${_bf_config})
      message(FATAL_ERROR "LIBFPV_STANDALONE_BF_CONFIG: ${_bf_config} not found")
    endif()
    if(EMSCRIPTEN)
      # into the page's virtual file system, beside the persistent data dir
      target_link_options(fpv_standalone PRIVATE "SHELL:--embed-file ${_bf_config}@/bf_config.txt")
      target_compile_definitions(fpv_standalone PRIVATE FPV_STANDALONE_BF_CONFIG="/bf_config.txt")
      set_property(TARGET fpv_standalone APPEND PROPERTY LINK_DEPENDS ${_bf_config})
    else()
      target_compile_definitions(fpv_standalone PRIVATE FPV_STANDALONE_BF_CONFIG="${_bf_config}")
    endif()
  endif()
  if(EMSCRIPTEN)
    # The web page: everything in <build>/web (index.html, index.js,
    # index.wasm), ready for any static web host (docs/WEB.md).
    set_target_properties(fpv_standalone PROPERTIES OUTPUT_NAME index SUFFIX .html
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/web)
    target_link_options(fpv_standalone PRIVATE
      -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 -sALLOW_MEMORY_GROWTH=1 -sENVIRONMENT=web
      "SHELL:--shell-file ${CMAKE_CURRENT_SOURCE_DIR}/standalone/web/shell.html")
    set_property(TARGET fpv_standalone APPEND PROPERTY LINK_DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/standalone/web/shell.html ${PROJECT_SOURCE_DIR}/src/web/libfpv_web.js)
  endif()
endif()

# Windows loads a DLL from the exe's folder first: copy the fresh libfpv.dll
# beside the examples on every build, so a stale copy there never shadows it
# (and a clean build folder runs at all).
get_target_property(_fpv_type fpv TYPE)
if(WIN32 AND _fpv_type STREQUAL "SHARED_LIBRARY")
  foreach(_example fpv_minimal fpv_standalone)
    if(TARGET ${_example})
      add_custom_command(TARGET ${_example} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:fpv> $<TARGET_FILE_DIR:${_example}>)
    endif()
  endforeach()
endif()
