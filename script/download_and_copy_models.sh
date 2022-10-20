#!/bin/bash

root_path=$PWD
target_dir="model"
mkdir -p $target_dir

if [[ ! -d "${INTEL_OPENVINO_DIR}" ]];
then echo "Please make sure openvino has been installed and enviroment variables has been set by "
    echo "source intel/openvino_2022/setupvars.sh"
    exit -1;
fi

pip install openvino-dev==2021.4.2
export PATH=$PATH=~/.local/bin

model_list="face-detection-retail-0004 human-pose-estimation-0001 vehicle-license-plate-detection-barrier-0106 vehicle-attributes-recognition-barrier-0039 person-detection-retail-0013 person-reidentification-retail-0288"

for i in $model_list
do
    if [ -f $root_path/$target_dir/$i.xml ]
    then
        echo "model/$i exists. No need to download"
    else

        if [ -f "/home/${USER}/intel/$i/FP16/$i.xml" ]
        then
            echo "Copy $i IR files to directory model/"
            cp /home/${USER}/intel/$i/FP16/$i.xml /home/${USER}/intel/$i/FP16/$i.bin $root_path/$target_dir/
        else
            echo " omz_downloader--name $i  --precisions FP16 --num_attempts 10"
            omz_downloader --name $i  --precisions FP16 --num_attempts 10 -o /home/${USER}/

            if [ -f "/home/${USER}/intel/$i/FP16/$i.xml" ]
            then
                echo "$i was downloaded successfully"
                cp /home/${USER}/intel/$i/FP16/$i.xml /home/${USER}/intel/$i/FP16/$i.bin $root_path/$target_dir/
            else
                echo "Failed to download $i model. "
                echo "Please manually run command:omz_downloader --name $i --precisions FP16" 
            fi

        fi
    fi
done
