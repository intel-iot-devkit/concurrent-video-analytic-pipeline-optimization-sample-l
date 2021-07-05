cmake_minimum_required( VERSION 3.6 )

cmake_policy(SET CMP0048 NEW) # version handled by project()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BUILDER_ROOT ${CMAKE_SOURCE_DIR}/builder )

set(OPENCV_INCLUDE $ENV{INTEL_CVSDK_DIR}/opencv/include)
set(OPENCV_LIB $ENV{INTEL_CVSDK_DIR}/opencv/lib)
set(DL_SDK_INCLUDE $ENV{INTEL_CVSDK_DIR}/deployment_tools/inference_engine/include)
set(DL_SDK_LIB $ENV{INTEL_CVSDK_DIR}/deployment_tools/inference_engine/lib/intel64/)
set(DL_SAMPLE_INCLUDE $ENV{INTEL_OPENVINO_DIR}/inference_engine/samples/cpp/common/)
set(NGRAPH_INCLUDE $ENV{INTEL_OPENVINO_DIR}/deployment_tools/ngraph/include/)
set(NGRAPH_LIB $ENV{INTEL_OPENVINO_DIR}/deployment_tools/ngraph/lib/)

set(MFX_INCLUDE $ENV{MFX_HOME}/include)
set(MFX_LIB_OPENSOURCE $ENV{MFX_HOME}/lib)
set(MFX_LIB $ENV{MFX_HOME}/lib/lin_x64)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -m64 -msse4.2  -pipe -fPIC -Wall -Wformat -Wformat-security -Wnon-virtual-dtor -Wno-deprecated-declarations -Wno-unknown-pragmas -Wno-unused  -DUNIX -D__USE_LARGEFILE64 -D_FILE_OFFSET_BITS=64 -DLINUX -DLINUX32 -DLINUX64 -DMFX_API_VERSION=\"1.34\" -DMFX_GIT_COMMIT=\"34c7cbbb\" -DENABLE_INFERENCE -DMEDIA_VERSION_STR=\"20.3.0\" -g -O3 -std=c++11 -funroll-loops -ftree-vectorize -I${DL_SAMPLE_INCLUDE} -I${NGRAPH_INCLUDE} -I${MFX_INCLUDE} -I${DL_SDK_INCLUDE} -I${DL_SDK_INCLUDE}/cpp -I${OPENCV_INCLUDE} -DLIBVA_SUPPORT -DLIBVA_DRM_SUPPORT -DLIBVA_X11_SUPPORT")

set(ENABLE_EXPORTS false)
set(CMAKE_EXE_LINKER_FLAGS "-L${OPENCV_LIB} -L${DL_SDK_LIB} -L${NGRAPH_LIB}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_SOURCE_DIR}/../bin/)

link_directories(${CMAKE_SOURCE_DIR}/lib)

link_directories(${OPENCV_LIB} ${MFX_LIB_OPENSOURCE} ${MFX_LIB})

list( APPEND LIBS_VARIANT sample_common )
list( APPEND LIBS_NOVARIANT vpp_plugin )

include_directories (
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MFX_INCLUDE}/sample_common/include
    ${MFX_INCLUDE}/mfx
    ${MFX_INCLUDE}/vpp_plugin/include
    ${MFX_INCLUDE}/rotate_cpu/include
    /usr/include/libdrm
)

set(DEPENDENCIES itt libmfx dl pthread)

file(GLOB SOURCES
    src/*.cpp
    include/*.h
    include/*.cpp
    human_pose/*.cpp
    vehicle_detect/*.cpp
    multi_object_tracker/*.cpp
)

add_executable(video_e2e_sample ${SOURCES})

target_link_libraries(video_e2e_sample avformat avcodec avutil  vpp_plugin sample_common mfx va va-drm drm pthread rt dl opencv_core opencv_video opencv_videoio opencv_imgproc opencv_photo opencv_highgui opencv_imgcodecs inference_engine inference_engine_legacy OpenCL ngraph)

