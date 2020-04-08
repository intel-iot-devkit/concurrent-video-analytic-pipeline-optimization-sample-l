
set(OPENCV_INCLUDE $ENV{INTEL_CVSDK_DIR}/opencv/include)
set(OPENCV_LIB $ENV{INTEL_CVSDK_DIR}/opencv/lib)
set(DL_SDK_INCLUDE $ENV{INTEL_CVSDK_DIR}/deployment_tools/inference_engine/include)
set(DL_SDK_LIB $ENV{INTEL_CVSDK_DIR}/deployment_tools/inference_engine/lib/intel64/)
set(DL_SAMPLE_INCLUDE $ENV{INTEL_OPENVINO_DIR}/inference_engine/samples/common/)
  
set(MFX_INCLUDE $ENV{MFX_HOME}/include)
set(MFX_LIB_OPENSOURCE $ENV{MFX_HOME}/lib)
set(MFX_LIB $ENV{MFX_HOME}/lib/lin_x64)
#set(GFLAGS_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/gflags/include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2 -g -O3 -std=c++11 -funroll-loops -ftree-vectorize -I${DL_SAMPLE_INCLUDE}  -I${MFX_INCLUDE} -I${DL_SDK_INCLUDE} -I${DL_SDK_INCLUDE}/cpp -I${OPENCV_INCLUDE} -DLIBVA_SUPPORT -DLIBVA_DRM_SUPPORT")
set(ENABLE_EXPORTS false)
set(CMAKE_EXE_LINKER_FLAGS "-L${OPENCV_LIB} -L${DL_SDK_LIB}")

link_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(${OPENCV_LIB} ${MFX_LIB_OPENSOURCE} ${MFX_LIB})

list( APPEND LIBS_VARIANT sample_common )
list( APPEND LIBS_NOVARIANT vpp_plugin )

include_directories (
  ${CMAKE_CURRENT_SOURCE_DIR}/../sample_common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../sample_misc/wayland/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../sample_plugins/vpp_plugin/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../sample_plugins/rotate_cpu/include
  /usr/include/libdrm
)

set(DEPENDENCIES itt libmfx dl pthread)

make_executable( shortname universal "nosafestring" )

target_link_libraries(video_e2e_sample avformat avcodec avutil  vpp_plugin sample_common mfx va va-drm drm pthread rt dl opencv_core opencv_video opencv_videoio opencv_imgproc opencv_photo opencv_highgui opencv_imgcodecs inference_engine)
