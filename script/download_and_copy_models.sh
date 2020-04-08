#!/bin/bash

root_path=$PWD
target_dir="model"
mkdir -p $target_dir

if [[ ! -d "${INTEL_OPENVINO_DIR}" ]];
then echo "Please make sure openvino has been installed and enviroment variables has been set by "
    echo "source intel/openvino/bin/setupvars.sh"
    exit -1;
fi
cd ${INTEL_OPENVINO_DIR}
cd deployment_tools/tools/model_downloader

model_list="face-detection-retail-0004 human-pose-estimation-0001 vehicle-license-plate-detection-barrier-0106 vehicle-attributes-recognition-barrier-0039" 

for i in $model_list
do
    if [ -f $root_path/$target_dir/$i.xml ]
    then
        echo "model/$i exists. No need to download"
    else

        if [ -f "${INTEL_OPENVINO_DIR}/deployment_tools/open_model_zoo/tools/downloader/intel/$i/FP16/$i.xml" ]
        then
            echo "Copy $i IR files to directory model/"
            cp ${INTEL_OPENVINO_DIR}/deployment_tools/open_model_zoo/tools/downloader/intel/$i/FP16/$i.xml ${INTEL_OPENVINO_DIR}/deployment_tools/open_model_zoo/tools/downloader/intel/$i/FP16/$i.bin $root_path/$target_dir/
        else
            echo "./downloader.py --name $i  --precisions fp16"
            ./downloader.py --name $i  --precisions FP16

            if [ -f "${INTEL_OPENVINO_DIR}/deployment_tools/open_model_zoo/tools/downloader/intel/$i/FP16/$i.xml" ]
            then
                echo "$i was downloaded successfully"
                cp ${INTEL_OPENVINO_DIR}/deployment_tools/open_model_zoo/tools/downloader/intel/$i/FP16/$i.xml ${INTEL_OPENVINO_DIR}/deployment_tools/open_model_zoo/tools/downloader/intel/$i/FP16/$i.bin $root_path/$target_dir/
            else
                echo "Failed to download $i model. "
                echo "Please manually run command:${INTEL_OPENVINO_DIR}/deployment_tools/tools/model_downloader/downloader.py --name $i --precisions fp16" 
            fi

        fi
    fi
done
