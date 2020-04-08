#!/bin/bash

root_path=$PWD

if [[ ! -d "${INTEL_OPENVINO_DIR}" ]];
then echo "Please make sure openvino has been installed and enviroment variables has been set by "
    echo "source intel/openvino/bin/setupvars.sh"
    exit -1;
fi

./msdk_pre_install.py

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

cd "MediaSDK"
git reset --hard
patch -p1 < ../patches/0001-MSDK-Enable-AVC-decode-SFC-RGB4.patch
patch -p1 < ../patches/0002-sample_common-Add-support-to-multiple-displays.patch

if [ $? != 0 ]; then
    echo "Apply patch failed!"
fi

cd ../
cp -rf video_e2e_sample  MediaSDK/samples/

./msdk_pre_install.py -b cfl

if [[ -f MediaSDK/build/__bin/release/video_e2e_sample ]];
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

    if [[ $LD_LIBRARY_PATH != *"/opt/intel/mediasdk/lib"* ]]
    then
        export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/intel/mediasdk/lib"
        echo 'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/intel/mediasdk/lib"' >> ~/.bashrc
    fi

    echo "VAAS sample application building has completed!"
    echo "Please use ./bin/video_e2e_sample for testing"
fi

#Download openvino models
echo "Check if openvino models IR files have been downloaded...."
source ./script/download_and_copy_models.sh

