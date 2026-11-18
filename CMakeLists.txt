cmake_minimum_required(VERSION 3.20)

if(APPLE)
  # Universal binary: runs on Apple Silicon (M1) and Intel Macs
  set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "")
  set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "")
endif()

project(TrailStab VERSION 1.0.0 LANGUAGES C CXX)
if(APPLE)
  enable_language(OBJCXX)   # Metal render path (ofx/metal_render.mm)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)

# ---------------------------------------------------------------- OpenFX headers + C++ support lib
FetchContent_Declare(openfx
  GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openfx.git
  GIT_TAG        4249fe1b590a7e65caf4b1e4693b6b53220703f5
  SOURCE_SUBDIR  _do_not_add_)        # only fetch sources, we build what we need
FetchContent_MakeAvailable(openfx)

set(OFX_SUPPORT ${openfx_SOURCE_DIR}/Support)
add_library(ofxsupport STATIC
  ${OFX_SUPPORT}/Library/ofxsCore.cpp
  ${OFX_SUPPORT}/Library/ofxsImageEffect.cpp
  ${OFX_SUPPORT}/Library/ofxsInteract.cpp
  ${OFX_SUPPORT}/Library/ofxsLog.cpp
  ${OFX_SUPPORT}/Library/ofxsMultiThread.cpp
  ${OFX_SUPPORT}/Library/ofxsParams.cpp
  ${OFX_SUPPORT}/Library/ofxsProperty.cpp
  ${OFX_SUPPORT}/Library/ofxsPropertyValidation.cpp)
target_include_directories(ofxsupport PUBLIC
  ${openfx_SOURCE_DIR}/include ${OFX_SUPPORT}/include ${OFX_SUPPORT}/Library)
target_compile_options(ofxsupport PRIVATE -w)

# ---------------------------------------------------------------- OSQP (quadratic program for the camera path)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG        v1.0.0)
FetchContent_MakeAvailable(osqp)

# ---------------------------------------------------------------- portable core (no host dependency)
add_library(trailstab_core STATIC
  core/sidecar.cpp
  core/path_solver.cpp
  core/warp.cpp
  core/analysis.cpp
  core/threadpool.cpp)
target_include_directories(trailstab_core PUBLIC core)
# lets the compiler vectorise the Lucas-Kanade sums (about -20 % analysis time); NOT -ffast-math,
# which would break std::isfinite in the tracker
set_source_files_properties(core/analysis.cpp PROPERTIES COMPILE_OPTIONS "-fassociative-math;-fno-signed-zeros;-fno-trapping-math")
find_package(Threads REQUIRED)
target_link_libraries(trailstab_core PUBLIC Threads::Threads)
target_link_libraries(trailstab_core PUBLIC osqpstatic)

# ---------------------------------------------------------------- helper: build an .ofx.bundle
function(add_ofx_bundle target ofxid)
  add_library(${target} MODULE ${ARGN})
  target_link_libraries(${target} PRIVATE ofxsupport)
  set_target_properties(${target} PROPERTIES PREFIX "" SUFFIX ".ofx")
  if(APPLE)
    set(arch_dir MacOS)
    target_link_options(${target} PRIVATE -Wl,-exported_symbol,_OfxGetPlugin -Wl,-exported_symbol,_OfxGetNumberOfPlugins)
  elseif(WIN32)
    set(arch_dir Win64)
  else()
    set(arch_dir Linux-x86-64)
  endif()
  set(bundle ${CMAKE_BINARY_DIR}/${target}.ofx.bundle)
  # plugin icon: OpenFX hosts look for Contents/Resources/<plugin identifier>.png
  if(EXISTS ${CMAKE_SOURCE_DIR}/resources/Stability.png)
    configure_file(${CMAKE_SOURCE_DIR}/resources/Stability.png ${bundle}/Contents/Resources/${ofxid}.png COPYONLY)
  endif()
  set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${bundle}/Contents/${arch_dir})
  if(APPLE)
    set(OFX_BUNDLE_EXECUTABLE ${target}.ofx)
    set(OFX_BUNDLE_ID de.trailstab.${target})
    configure_file(${CMAKE_SOURCE_DIR}/cmake/Info.plist.in ${bundle}/Contents/Info.plist @ONLY)
  endif()
  install(DIRECTORY ${bundle} DESTINATION /Library/OFX/Plugins USE_SOURCE_PERMISSIONS)
endfunction()

add_ofx_bundle(TrailStab de.trailstab.Stabilizer ofx/TrailStabPlugin.cpp)
target_link_libraries(TrailStab PRIVATE trailstab_core)
target_compile_definitions(TrailStab PRIVATE TRAILSTAB_VERSION="${PROJECT_VERSION}")
target_include_directories(TrailStab PRIVATE ofx)
if(APPLE)
  target_sources(TrailStab PRIVATE ofx/metal_render.mm)
  set_source_files_properties(ofx/metal_render.mm PROPERTIES COMPILE_OPTIONS "-fobjc-arc")
  target_link_libraries(TrailStab PRIVATE "-framework Metal" "-framework Foundation")
else()
  target_sources(TrailStab PRIVATE ofx/metal_render_stub.cpp)
endif()

# ---------------------------------------------------------------- tests / tools
add_executable(trailstab_selftest tools/selftest.cpp)
target_link_libraries(trailstab_selftest PRIVATE trailstab_core)
add_executable(analyze_cli tools/analyze_cli.cpp)
target_link_libraries(analyze_cli PRIVATE trailstab_core)
add_executable(dump_path tools/dump_path.cpp)
target_link_libraries(dump_path PRIVATE trailstab_core)
add_executable(dump_warp tools/dump_warp.cpp)
target_link_libraries(dump_warp PRIVATE trailstab_core)
