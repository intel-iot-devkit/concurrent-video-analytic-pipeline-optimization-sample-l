#!/bin/bash
#This script is used to pack all libva, media-driver and MediaSDK binaries to directory cva_e2e_sample_l.
#Copy whole directory cva_e2e_sample_l to another device and run install_binary.sh to install the binaries

root_path=$PWD
release_folder=$root_path/cva_e2e_sample_l
echo "Release folder is $release_folder"
rm -rf $release_folder
mkdir -p $release_folder/libva
mkdir -p $release_folder/media-driver
mkdir -p $release_folder/MediaSDK
libva_binary_path=$release_folder/libva/

cd libva/va/

libva_file_list="../COPYING drm/va_drm.h \
        x11/va_dri2.h x11/va_dricommon.h \
        glx/va_backend_glx.h glx/va_glx.h \
        libva.la libva-drm.la libva-x11.la libva-glx.la \
        .libs/libva.so.2.1100.0  .libs/libva.lai \
        .libs/libva-drm.so.2.1100.0T .libs/libva-drm.lai  \
        .libs/libva-x11.so.2.1100.0T .libs/libva-x11.lai  \
        .libs/libva-glx.so.2.1100.0T .libs/libva-glx.lai  \
         va.h va_backend.h va_backend_vpp.h va_compat.h \
        va_dec_av1.h va_dec_hevc.h va_dec_jpeg.h va_dec_vp8.h \
        va_dec_vp9.h va_drmcommon.h va_egl.h va_enc_hevc.h \
        va_enc_h264.h va_enc_jpeg.h va_enc_vp8.h va_fei.h \
        va_fei_h264.h va_enc_mpeg2.h va_fei_hevc.h va_enc_vp9.h \
        va_str.h va_tpi.h va_version.h va_vpp.h va_x11.h \
        ../pkgconfig/libva.pc ../pkgconfig/libva-drm.pc ../pkgconfig/libva-x11.pc ../pkgconfig/libva-glx.pc
"
echo "Pack libva header files and libraries"
for i in $libva_file_list
do
    cmd="cp $i $libva_binary_path"
    echo $cmd
    `$cmd`
    if [ $? != 0 ];
    then
        echo "Failed to run command \"$cmd\" "
        exit 1;
    fi
done

cd $root_path


echo "Pack libva-utils binary"
mkdir -p $release_folder/libva-utils

cp libva-utils/vainfo/vainfo $release_folder/libva-utils/


echo "Pack media-driver binary"
mkdir -p $release_folder/media-driver
cp media_build/media_driver/iHD_drv_video.so $release_folder/media-driver/
cp media_build/cmrtlib/linux/libigfxcmrt.so $release_folder/media-driver/
cp media-driver/LICENSE.md $release_folder/media-driver/

echo "Pack MediaSDK binaryies"
msdk_release_dir=$release_folder/MediaSDK/
echo "MediaSDK binaries will be copied into $msdk_release_dir"
mkdir -p $msdk_release_dir/include
mkdir -p $msdk_release_dir/lib/pkgconfig
mkdir -p $msdk_release_dir/lib/mfx
mkdir -p  $msdk_release_dir/share/mfx/samples

cd $root_path/MediaSDK/
cp LICENSE $msdk_release_dir/ 
cp api/include/*.h $msdk_release_dir/include/ 

cd $root_path/MediaSDK/build/
echo "cd $root_path/MediaSDK/__cmake/intel64.make.release"
if [ $? != 0 ]; then 
    echo "Folder $root_patch/MediaSDK/_cmake/intel64.make.release doesn't exist"
    echo "Please make MediaSDK has been build successfully"
    exit 1
fi

cp __bin/release/libmfx.pc  $msdk_release_dir/lib/pkgconfig/mfx.pc
cp __bin/release/libmfxhw64.so  $msdk_release_dir/lib/libmfxhw64.so.1.35 
cp __lib/release/libvpp_plugin.a $msdk_release_dir/share/mfx/samples/

sample_bin="sample_decode sample_encode sample_fei sample_multi_transcode"
for i in $sample_bin;
do
    cp __bin/release/$i $msdk_release_dir/share/mfx/samples/
    echo "cp __bin/release/$i $msdk_release_dir/share/mfx/samples/"
done
cp __bin/release/video_e2e_sample $release_folder/

cp plugins.cfg $msdk_release_dir/share/mfx

plugins_lib="libmfx_hevcd_hw64.so libmfx_hevcd_hw64.so libmfx_h264la_hw64.so libmfx_vp9e_hw64.so libmfx_vp8d_hw64.so libmfx_vp9d_hw64.so libmfx_hevce_hw64.so libmfx_vp9e_hw64.so" 

for i in $plugins_lib;
do
    cp __bin/release/$i $msdk_release_dir/lib/mfx/
    echo "cp __bin/release/$i $msdk_release_dir/share/mfx/"
done

cd $root_path
cp $root_path/script/install_binary.sh  $release_folder/
cp $root_path/script/download_and_copy_models.sh   $release_folder/
cp $root_path/script/run_face_detection_test.sh $release_folder/
cp $root_path/svet_env_setup.sh $release_folder/

mkdir -p $release_folder/par_file
cp par_file/inference/n16_face_detection_1080p.par $release_folder/par_file/
cp par_file/inference/n4_human_pose_1080p.par $release_folder/par_file/
cp par_file/inference/n4_vehicel_detect_1080p.par  $release_folder/par_file/
echo "libva, media-driver, MediaSDK and SVET sample application binaries have been copied to $release_folder"
echo "Done"
