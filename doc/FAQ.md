# Frequently asked questions (SVET sample application)

## Where can I find the descritpion of options used in par file?
See chapter 2.4 in doc/svet_sample_application_user_guide_2020.1.0.pdf
Running the SVET sample applicton with option "-?" can show the usage of options.

## Why does the system need to be switched to text mode before running the sample application
The sample application uses libDRM to render the video directly to display, so it needs to act as master of the DRM display, which isn't allowed when X server is running. 
If the par file doesn't include display session, there is no need to switch to text mode.

## Why it needs "su -p" to switch to root user before running the sample application
To become DRM master, it needs root privileges. With option "-p", it will preserve environment variables, like LIBVA_DRIVERS_PATH, LIBVA_DRIVER_NAME and LD_LIBRARY_PATH. If without "-p", these environment variables will be reset and the sample application will run into problems.

## The loading time of 16-channel face detection demo is too long
Please enable cl_cache by running command "export cl_cache_dir=/tmp/cl_cache" and "mkdir -p /tmp/cl_cache". Then after the first running of 16-channel face detection demo, the compiled OpenCL kernles are cached and the model loading time of next runnings of 16-channel face detection demo will only take about 10 seconds.
More details about cl_cache can be found at https://github.com/intel/compute-runtime/blob/master/opencl/doc/FAQ.md

## Can sources number for "-vpp_comp_only" or "-vpp_comp" be different from number of decoding sessions?  
No. The sources number for "-vpp_comp_only" or "-vpp_comp" must be equal to the numer of decoding sessions. Otherwise, the sample application will fail during pipeline initialization or running. 

## How to limit the fps of whole pipeline to 30?
Add "-fps 30" to every decoding session.

## How to limit the frame number of input to 1000?
Add "-n 1000" to every decoding dessions. However this option won't work if both "-vpp_comp_only" and "-vpp_comp" are set.

## Where can I find tutorials for inference engine?
Please refer to https://docs.openvinotoolkit.org/latest/_docs_IE_DG_Deep_Learning_Inference_Engine_DevGuide.html

## Where can I find information for the models?
Please refer to https://github.com/opencv/open_model_zoo/tree/master/models/intel. The names of models used in sample application are
face-detection-retail-0004, human-pose-estimation-0001, vehicle-attributes-recognition-barrier-0039, vehicle-license-plate-detection-barrier-0106.

## Can I use other OpenVINO version rather than 2019 R3?
Yes, but you have to modify some code due to interfaces changing. And also you need to download the IR files and copy them to ./model manually. Please refer to script/download_and_copy_models.sh for how to download the IR files.
