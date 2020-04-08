if [[ ! -d "${INTEL_OPENVINO_DIR}" ]];
then echo "Please make sure openvino has been installed and enviroment variables has been set by "
    echo "source intel/openvino/bin/setupvars.sh"
    exit -1;
fi

if [[ ! -d "${LIBVA_DRIVERS_PATH}" ]];
then echo "Please make sure media-driver has been installed and environment variable LIBVA_DRIVERS_PATH was set correctly in your .bashrc"
    echo "Please use 'su -p' instead of 'su' to switch to root user, otherwise, the environment variables will be reset"
fi

MEDIA_AI_USE_OPENVINO_MODEL=1 ./video_e2e_sample -par par_file/face_detection_1080p_16_channel.par 
