#!/bin/bash

root_path=$PWD
#download_method="git"
download_method="wget"

if [[ ! -d "${INTEL_OPENVINO_DIR}" ]];
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
    media_packages_md5sum[intel-gmmlib-20.3.2.tar.gz]=583d9fbef52d880238629f2ced50a9be
    media_packages_md5sum[intel-media-20.3.0.tar.gz]=9c9219c09447567254aec00ade5dd3b8
    media_packages_md5sum[intel-mediasdk-20.3.0.tar.gz]=1fd2cb03d15b3c308e6a9bd1735d2845
    media_packages_md5sum[libva.tar.gz]=fdc00aedad4a51e058403f5f24d9abac
    media_packages_md5sum[libva-utils.tar.gz]=a22479f85f7693536c5756ad46192b32 

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

    
    va_env_set=`grep "LIBVA_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri" ~/.bashrc -c`

    if (( va_env_set == 0 )) 
    then
        set -x
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu:/usr/lib
        export LIBVA_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri
        export LIBVA_DRIVER_NAME=iHD
        export MFX_HOME=/opt/intel/mediasdk
        set +x
        echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu:/usr/lib' >> ~/.bashrc
        echo "export LIBVA_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri" >> ~/.bashrc
        echo "export LIBVA_DRIVER_NAME=iHD" >> ~/.bashrc
        echo "export MFX_HOME=/opt/intel/mediasdk" >> ~/.bashrc

    fi

    if [[ -d "/opt/intel/mediasdk/lib64" ]]
    then
        echo "A old MediaSDK version is found under folder /opt/intel/mediasdk/lib64"
        echo "Rename /opt/intel/mediasdk/lib64 as /opt/intel/mediasdk/oldlib64"
        mv  /opt/intel/mediasdk/lib64 /opt/intel/mediasdk/oldlib64
    fi

    if [[ $LD_LIBRARY_PATH != *"/opt/intel/mediasdk/lib/"* ]]
    then
        export LD_LIBRARY_PATH="/opt/intel/mediasdk/lib/:$LD_LIBRARY_PATH"
        echo 'export LD_LIBRARY_PATH="/opt/intel/mediasdk/lib/:$LD_LIBRARY_PATH"' >> ~/.bashrc
    fi

    if [[ -d $cl_cache_dir ]];
    then
        echo "cl_cache is enabled. \$cl_cache_dir : $cl_cache_dir"
    else
        echo "Add enabling cl_cache commands to ~\.bashrc"
        set -x
        mkdir -p ~/cl_cache
        export cl_cache_dir=~/cl_cache
        echo "mkdir -p ~/cl_cache" >> ~/.bashrc
        echo "export cl_cache_dir=~/cl_cache" >> ~/.bashrc
    fi

    #Start to build SVET sample application
    #Copy MediaSDK sample headers and libraries that weren't installed by default
    sudo mkdir -p /opt/intel/mediasdk/include/sample_common
    sudo mkdir -p /opt/intel/mediasdk/include/vpp_plugin
    sudo mkdir -p /opt/intel/mediasdk/include/rotate_cpu
    sudo cp -rf MediaSDK/samples/sample_common/include/ /opt/intel/mediasdk/include/sample_common/
    sudo cp -rf MediaSDK/samples/sample_plugins/vpp_plugin/include/ /opt/intel/mediasdk/include/vpp_plugin/
    sudo cp -rf MediaSDK/samples/sample_plugins/rotate_cpu/include/ /opt/intel/mediasdk/include/rotate_cpu/
    sudo cp -f MediaSDK/build/__lib/release/libsample_common.a  /opt/intel/mediasdk/lib/
    sudo cp -f MediaSDK/build/__lib/release/libvpp_plugin.a /opt/intel/mediasdk/lib/

    mkdir -p video_e2e_sample/build
    cd video_e2e_sample/build
    cmake ../ | tee /tmp/svet_build.log
    cat /tmp/svet_build.log >> svet_build.log
    make -j4 | tee /tmp/svet_build.log
    cat /tmp/svet_build.log >> svet_build.log

    cd $root_path
    if [[ -f bin/video_e2e_sample ]];
    then
        echo "SVET sample application building has completed!"
        echo "Please use ./bin/video_e2e_sample for testing"
    else
        echo "SVET sample application building failed!"
        echo "Please create an IPS with SVET in title on premiersupport.intel.com and upload ./svet_build.log ./svet_download.log to IPS."
    fi
fi

#Download openvino models
echo "Check if openvino models IR files have been downloaded...."
source ./script/download_and_copy_models.sh

