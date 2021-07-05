#!/bin/bash

root_path=$PWD
#download_method="git"
download_method="wget"
built_with_ocv=true
LIBVA_INSTALL_PREFIX="/opt/intel/svet/msdk"
LIBVA_INSTALL_PATH="${LIBVA_INSTALL_PREFIX}/lib"
LIBVA_DRIVERS_PATH="${LIBVA_INSTALL_PATH}/dri"
LIBVA_DRIVER_NAME="iHD"

while getopts b: flag
do
    case "${flag}" in
        b)
            if [[ "no_ocv" == ${OPTARG} ]]
            then
                echo "Build SVET without OpenVINO"
                built_with_ocv=false
            fi
            ;;
    esac
done

if [ ! -d "${INTEL_OPENVINO_DIR}" ] && [ $built_with_ocv = true ];
then echo "Please make sure openvino has been installed and enviroment variables has been set by "
    echo "source intel/openvino/bin/setupvars.sh"
    exit -1;
fi

sudo apt install python

if [[ $download_method == "git" ]]
then
python msdk_pre_install_internal.py | tee svet_download.log
else
python msdk_pre_install.py | tee svet_download.log
fi

git_projects="MediaSDK media-driver libva"
for i in $git_projects;
do
    if [[ ! -d $i ]];
    then
        echo "Directory $i doesn't exist. Please run ./msdk_pre_install.py manually to
    download the code"
    exit -1;
    fi
done

#check md5sum if wget is used to download source code packages

if [[ $download_method == "wget" ]]
then
    echo "Checking the md5sum of downloaded packages"
    declare -A  media_packages_md5sum
    media_packages_md5sum[intel-gmmlib-21.1.1.tar.gz]=100ff57bff1cb19763f729fcd00269d1
    media_packages_md5sum[intel-media-21.1.3.tar.gz]=98f59d7fd2c6c08808acf0ba741216b0
    media_packages_md5sum[intel-mediasdk-21.1.3.tar.gz]=a4747e315448ded38171b245232a5a7c
    media_packages_md5sum[libva.tar.gz]=ba136b5a02ee4c7bf093d09bc1635560
    media_packages_md5sum[libva-utils.tar.gz]=010243cc8217fd455d0ad11f2025d43c

    for c in ${!media_packages_md5sum[@]}; do
        md5_res=`md5sum $c`
        if [[ ${media_packages_md5sum[$c]} ==  ${md5_res%% *} ]]
        then
            echo "$c" "${media_packages_md5sum[$c]}" correct
        else
            echo "$c" "md5sum is wrong " ${md5_res%% *}  vs ${media_packages_md5sum[$c]}
            echo "Pleaes remove libva/media-driver/MediaSDK re-run this script"
            echo "If you still see md5sum not match, please refer to msdk_pre_install.py and download, uncomparess the package manually"
            echo "Or you can modify line 5 in this scirpt and change the download_method to \"git\". Then re-run this script."
            exit
        fi
    done
fi

cd "MediaSDK"
patch -N --no-backup-if-mismatch -p1 < ../patches/0001-MSDK-Enable-AVC-decode-SFC-RGB4.patch
patch -N --no-backup-if-mismatch -p1 < ../patches/0002-sample_common-Add-support-to-multiple-displays.patch

if [ $? != 0 ]; then
    echo "Apply patch failed!"
fi

cd ../

if [[ $download_method == "git" ]]
then
python msdk_pre_install_internal.py -b cfl| tee svet_build.log
else
python msdk_pre_install.py -b cfl | tee svet_build.log
fi

#video_e2e_sample depends on libsample_common.a
if [[ -f MediaSDK/build/__lib/release/libsample_common.a ]];
then
    rm -f bin
    ln -s MediaSDK/build/__bin/release/ bin


    #va_env_set=`grep "LIBVA_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri" ~/.bashrc -c`
    set -x
    export LD_LIBRARY_PATH=$LIBVA_INSTALL_PATH:$LD_LIBRARY_PATH
    export LIBRARY_PATH=$LIBVA_INSTALL_PATH:$LIBRARY_PATH
    export LIBVA_DRIVERS_PATH=$LIBVA_DRIVERS_PATH
    export LIBVA_DRIVER_NAME=$LIBVA_DRIVER_NAME
    export MFX_HOME=$LIBVA_INSTALL_PREFIX
    set +x

    #if (( va_env_set == 0 ))
    #then
        #set -x
        #export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu:/usr/lib
        #export LIBVA_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri
        #export LIBVA_DRIVER_NAME=iHD
        #export MFX_HOME=/opt/intel/mediasdk
        #set +x
        #echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu:/usr/lib' >> ~/.bashrc
        #echo "export LIBVA_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri" >> ~/.bashrc
        #echo "export LIBVA_DRIVER_NAME=iHD" >> ~/.bashrc
        #echo "export MFX_HOME=/opt/intel/mediasdk" >> ~/.bashrc

    #fi

    #if [[ -d "/opt/intel/mediasdk/lib64" ]]
    #then
    #    echo "A old MediaSDK version is found under folder /opt/intel/mediasdk/lib64"
    #    echo "Rename /opt/intel/mediasdk/lib64 as /opt/intel/mediasdk/oldlib64"
    #    mv  /opt/intel/mediasdk/lib64 /opt/intel/mediasdk/oldlib64
    #fi

    #if [[ $LD_LIBRARY_PATH != *"/opt/intel/mediasdk/lib/"* ]]
    #then
    #    export LD_LIBRARY_PATH="/opt/intel/mediasdk/lib/:$LD_LIBRARY_PATH"
    #    echo 'export LD_LIBRARY_PATH="/opt/intel/mediasdk/lib/:$LD_LIBRARY_PATH"' >> ~/.bashrc
    #fi

    if [[ -d $cl_cache_dir ]];
    then
        echo "cl_cache is enabled. \$cl_cache_dir : $cl_cache_dir"
        echo "Clean previous cl kernel cache"
        sudo rm -f $cl_cache_dir/*
    else
        echo "Add enabling cl_cache commands to ~\.bashrc"
        set -x
        mkdir -p ~/cl_cache
        export cl_cache_dir=~/cl_cache
        echo "mkdir -p ~/cl_cache" >> ~/.bashrc
        echo "export cl_cache_dir=~/cl_cache" >> ~/.bashrc
    fi

    #install library for metric_monitor
    sudo cp MediaSDK/build/__bin/release/libcttmetrics.so /usr/lib/

    #Start to build SVET sample application
    #Copy MediaSDK sample headers and libraries that weren't installed by default
    sudo -E mkdir -p $MFX_HOME/include/sample_common
    sudo -E mkdir -p $MFX_HOME/include/vpp_plugin
    sudo -E mkdir -p $MFX_HOME/include/rotate_cpu
    sudo -E cp -rf MediaSDK/samples/sample_common/include/ $MFX_HOME/include/sample_common/
    sudo -E cp -rf MediaSDK/samples/sample_plugins/vpp_plugin/include/ $MFX_HOME/include/vpp_plugin/
    sudo -E cp -rf MediaSDK/samples/sample_plugins/rotate_cpu/include/ $MFX_HOME/include/rotate_cpu/
    sudo -E cp -f MediaSDK/build/__lib/release/libsample_common.a  $LIBVA_INSTALL_PATH/
    sudo -E cp -f MediaSDK/build/__lib/release/libvpp_plugin.a $LIBVA_INSTALL_PATH/

    rm -rf video_e2e_sample/build

    mkdir -p video_e2e_sample/build

    if [ $built_with_ocv != true ]
    then
        echo "Use the CMakefile.txt that excludes inference source code"
        cp script/CMakeLists_woOpenVINO.txt video_e2e_sample/CMakeLists.txt
    fi

    cd video_e2e_sample/build
    cmake ../ | tee /tmp/svet_build.log
    cat /tmp/svet_build.log >> svet_build.log
    make -j4 | tee /tmp/svet_build.log
    cat /tmp/svet_build.log >> svet_build.log
fi

cd $root_path
if [ $built_with_ocv = true ]
then
    #Download openvino models
    echo "Check if openvino models IR files have been downloaded...."
    source ./script/download_and_copy_models.sh
fi

cd $root_path

if [[ ! -d video ]]
then
    mkdir -p video
fi

if [[ ! -f video/1080p.h264 ]]
then
   echo "Download test clip video/1080p.h264"
   cd video/
   wget -O main.zip https://github.com/elainewangprc/testvideo/archive/refs/heads/main.zip
   if [[ ! -f "main.zip" ]]
   then
       echo "Failed to download https://github.com/elainewangprc/testvideo/archive/refs/heads/main.zip"
       echo "Please download it manually"
   else
       unzip main.zip
       cp testvideo-main/*.h264 .
   fi
   cd ../
fi

cd $root_path
if [[ -f bin/video_e2e_sample ]]
then
    echo "************************************************************************"
    echo "SVET sample application building has completed!"

    gen12=`sudo cat /sys/kernel/debug/dri/0/i915_capabilities | grep "gen: 12"`
    if [[ ! -z $gen12 ]];
    then
        ver=`uname -a`
        if [[ $ver != *5.4.102* ]] ;
        then
            echo "For TigerLake CPU, please refer to user guide chapter 1.3 to upgrade the kernel with https://github.com/intel/linux-intel-lts/releases/tag/lts-v5.4.102-yocto-210310T010318Z";
        fi
    fi

    echo "Please run the cmd below to setup running environment:"
    echo " "
    echo "source ./svet_env_setup.sh"
    echo " "
    echo "Then use ./bin/video_e2e_sample for testing"
    echo "IMPORTANT NOTICE: please run 'source ./svet_env_setup.sh' first when you start a new shell (or change user in shell such as run 'su -') to run ./bin/video_e2e_sample"
    echo "************************************************************************"
else
    echo "SVET sample application building failed!"
    echo "Please create an IPS with SVET in title on premiersupport.intel.com and upload ./svet_build.log ./svet_download.log to IPS."
fi

