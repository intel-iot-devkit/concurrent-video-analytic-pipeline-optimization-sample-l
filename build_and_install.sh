#!/bin/bash

root_path=$PWD
#download_method="git"
download_method="wget"
built_with_ocv=true

install_overwrite=False

while getopts 'b:i:' flag; do
    case "${flag}" in
        b)
        if [[ "no_ocv" == ${OPTARG} ]]
        then
                echo "Build SVET without OpenVINO"
                built_with_ocv=false
        fi
        ;;
        i)
        if [[ "overwrite" == ${OPTARG} ]]
        then
                echo "Overwrite the default Intel media stack library installed."
                install_overwrite=True
                echo 'export SVET_INSTALL_OVERWRITE=True' >> ~/.bashrc
                export SVET_INSTALL_OVERWRITE=True
        fi
        ;;
    esac
done


if [ ! -d "${INTEL_OPENVINO_DIR}" ] && [ $built_with_ocv = true ];
then echo "Please make sure openvino has been installed and enviroment variables has been set by "
    echo "source intel/openvino/bin/setupvars.sh"
    exit -1;
fi

mkdir -p vpl
cd vpl
rm $root_path/svet_build.log
touch $root_path/svet_build.log
python3 ../onevpl_pre_install.py | tee $root_path/svet_build.log

mkdir -p ~/cl_cache
export cl_cache_dir=~/cl_cache

if [[ -d $cl_cache_dir ]];
then
	echo "cl_cache is enabled. \$cl_cache_dir : $cl_cache_dir"
	echo "Clean previous cl kernel cache"
	sudo rm -f $cl_cache_dir/*
else
	echo "Add enabling cl_cache commands to ~\.bashrc"
	mkdir -p ~/cl_cache
	export cl_cache_dir=~/cl_cache
	echo "mkdir -p ~/cl_cache" >> ~/.bashrc
	echo "export cl_cache_dir=~/cl_cache" >> ~/.bashrc
fi

cd $root_path
source ./svet_env_setup.sh
rm -rf video_e2e_sample/build

mkdir -p video_e2e_sample/build

if [ $built_with_ocv != true ]
then
	echo "Use the CMakefile.txt that excludes inference source code"
	cp script/CMakeLists_woOpenVINO.txt video_e2e_sample/CMakeLists.txt
fi

cd video_e2e_sample/build
cmake ../ | tee /tmp/svet_build.log
cat /tmp/svet_build.log >> $root_path/svet_build.log
make -j4 | tee /tmp/svet_build.log
cat /tmp/svet_build.log >> $root_path/svet_build.log

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

    if [[ $install_overwrite == True ]]
    then
        echo "Please run ./bin/video_e2e_sample for testing"
    else
        echo "Please run the cmd below to setup running environment:"
        echo " "
        echo "source ./svet_env_setup.sh"
        echo " "
        echo "Then use ./bin/video_e2e_sample for testing"
        echo "IMPORTANT NOTICE: please run 'source ./svet_env_setup.sh' first when you start a new shell (or change user in shell such as run 'su -') to run ./bin/video_e2e_sample"
        echo "************************************************************************"
    fi
else
    echo "SVET sample application building failed!"
    echo "Please create an IPS with SVET in title on premiersupport.intel.com and upload ./svet_build.log to IPS."
fi
