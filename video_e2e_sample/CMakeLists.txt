# ##############################################################################
# Copyright (C) Intel Corporation
#
# SPDX-License-Identifier: MIT
# ##############################################################################
cmake_minimum_required(VERSION 3.10.2)
project(video_e2e_sample)

# Default install places 64 bit runtimes in the environment, so we want to do a
# 64 bit build by default.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(BUILDER_ROOT ${CMAKE_SOURCE_DIR}/builder )

ADD_SUBDIRECTORY(sample_common)
link_directories(${CMAKE_SOURCE_DIR}/lib)
list( APPEND LIBS_VARIANT sample_common )

set(TARGET video_e2e_sample)
file(GLOB SOURCES
    src/*.cpp
    human_pose/*.cpp
    vehicle_detect/*.cpp
    multi_object_tracker/*.cpp
)

# Set default build type to RelWithDebInfo if not specified
if(NOT CMAKE_BUILD_TYPE)
  message(
    STATUS "Default CMAKE_BUILD_TYPE not set using Release with Debug Info")
  set(CMAKE_BUILD_TYPE
      "RelWithDebInfo"
      CACHE
        STRING
        "Choose build type from: None Debug Release RelWithDebInfo MinSizeRel"
        FORCE)
endif()

set(OPENCV_INCLUDE $ENV{INTEL_OPENVINO_DIR}/extras/opencv/include)
set(OPENCV_LIB $ENV{INTEL_OPENVINO_DIR}/extras/opencv/lib)
set(DL_SDK_INCLUDE $ENV{INTEL_OPENVINO_DIR}/runtime/include/)
set(DL_SDK_LIB $ENV{INTEL_OPENVINO_DIR}/runtime/lib/intel64/)
set(DL_SAMPLE_INCLUDE $ENV{INTEL_OPENVINO_DIR}/samples/cpp/common/utils/include/)
set(DL_SAMPLE_INCLUDE_V214 $ENV{INTEL_OPENVINO_DIR}/deployment_tools/inference_engine/samples/cpp/common/utils/include/)
set(NGRAPH_INCLUDE $ENV{INTEL_OPENVINO_DIR}/runtime/include/)
set(NGRAPH_LIB $ENV{INTEL_OPENVINO_DIR}/deployment_tools/ngraph/lib/)
set(DL_GPU_INTEL $ENV{INTEL_OPENVINO_DIR}/runtime/include/openvino/runtime/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -m64 -msse4.2  -pipe -fPIC -Wall -Wformat -Wformat-security -Wnon-virtual-dtor -Wno-deprecated-declarations -Wno-unknown-pragmas -Wno-unused  -DUNIX -D__USE_LARGEFILE64 -D_FILE_OFFSET_BITS=64 -DLINUX -DLINUX32 -DLINUX64 -DENABLE_INFERENCE -DMEDIA_VERSION_STR=\"22.3.0\" -g -O3 -std=c++14 -funroll-loops -ftree-vectorize -I${DL_SAMPLE_INCLUDE} -I${DL_SAMPLE_INCLUDE_V214} -I${NGRAPH_INCLUDE} -I${DL_SDK_INCLUDE} -I${DL_SDK_INCLUDE}/ie -I${DL_SDK_INCLUDE}/ie/cpp -I${OPENCV_INCLUDE} -I${DL_GPU_INTEL} -DLIBVA_SUPPORT -DLIBVA_DRM_SUPPORT -DLIBVA_X11_SUPPORT")

set(ENABLE_EXPORTS false)
set(CMAKE_EXE_LINKER_FLAGS "-L${OPENCV_LIB} -L${DL_SDK_LIB} -L${NGRAPH_LIB}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/../bin/)

link_directories(${CMAKE_SOURCE_DIR}/lib)

#link_directories(${OPENCV_LIB} ${MFX_LIB_OPENSOURCE} ${MFX_LIB})
link_directories(${OPENCV_LIB} )

list( APPEND LIBS_VARIANT sample_common )
list( APPEND LIBS_NOVARIANT vpp_plugin )

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -m64 -msse4.2  -pipe -fPIC -Wall -Wformat -Wformat-security -Wnon-virtual-dtor -Wno-deprecated-declarations -Wno-unknown-pragmas -Wno-unused  -DUNIX -D__USE_LARGEFILE64 -D_FILE_OFFSET_BITS=64 -DLINUX -DLINUX32 -DLINUX64  -g -O3 -std=c++14 -funroll-loops -ftree-vectorize -DLIBVA_SUPPORT -DLIBVA_DRM_SUPPORT -DLIBVA_X11_SUPPORT")
add_executable(${TARGET} ${SOURCES})

include_directories (
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/media_sdk_compatibility_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/sample_common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sample_common/include/vm
    #    ${MFX_INCLUDE}/mfx
    #    ${MFX_INCLUDE}/vpp_plugin/include
    #    ${MFX_INCLUDE}/rotate_cpu/include
    /usr/include/libdrm
    $ENV{INTEL_OPENVINO_DIR}/deployment_tools/inference_engine/include
)

if(POLICY CMP0074)
  # ignore warning of VPL_ROOT in find_package search path
  cmake_policy(SET CMP0074 OLD)
endif()

find_package(VPL REQUIRED)
target_link_libraries(${TARGET} VPL::dispatcher)

target_compile_definitions(${TARGET} PRIVATE MFX_ONEVPL LIBVA_SUPPORT)

if(UNIX)
  find_package(PkgConfig REQUIRED)
  # note: pkg-config version for libva is *API* version
  pkg_check_modules(PKG_LIBVA libva>=1.2 libva-drm>=1.2)
  if(PKG_LIBVA_FOUND)
    target_compile_definitions(${TARGET} PUBLIC -DLIBVA_SUPPORT)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    set(THREADS_PREFER_PTHREAD_FLAG TRUE)
    find_package(Threads REQUIRED)
    target_link_libraries(${TARGET} ${PKG_LIBVA_LIBRARIES}
                          ${PKG_THREAD_LIBRARIES})
    target_include_directories(${TARGET} PUBLIC ${PKG_LIBVA_INCLUDE_DIRS})
  else()
    message(WARNING "libva not found: building without libVA support")
  endif()
endif()

pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libavutil
)

target_link_libraries(${TARGET} PkgConfig::LIBAV)
#target_link_libraries(${TARGET} sample_common drm asound mp3lame pthread rt dl)
target_link_libraries(video_e2e_sample avformat avcodec avutil  sample_common va va-drm drm asound mp3lame pthread rt dl opencv_core opencv_video opencv_videoio opencv_imgproc opencv_photo opencv_highgui opencv_imgcodecs openvino OpenCL)
