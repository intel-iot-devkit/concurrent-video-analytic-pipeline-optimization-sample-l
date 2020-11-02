# Frequently asked questions (SVET sample application)

## Where can I find the descritpion of options used in par file?
See chapter 2.4 in doc/svet_sample_application_user_guide_2020.1.0.pdf
Running the SVET sample applicton with option "-?" can show the usage of options.

## Why does the system need to be switched to text console mode before running the sample application
The sample application uses libDRM to render the video directly to display, so it needs to act as master of the DRM display, which isn't allowed when X client is running. If there is any VNC session, please also close it. Because VNC session also starts X client. 
If the par file doesn't include display session, there is no need to switch to text mode.

## Why it needs "su -p" to switch to root user before running the sample application
To become DRM master, it needs root privileges. With option "-p", it will preserve environment variables, like LIBVA_DRIVERS_PATH, LIBVA_DRIVER_NAME and LD_LIBRARY_PATH. If without "-p", these environment variables will be reset and the sample application will run into problems.

## Is it possible to use X11 instead of DRM display?
If user doesn't want to switch to text console mode or switch to root for using DRM display, user can replace "-rdrm-DisplayPort" with "-rx11" in the par file. However, the X11 rendering isn't as effcient as DRM rendering. According to our 16-channel face detection 1080p test on CFL , the time cost of each frame increased by around 6ms. Example [par file](./par_file/inference/n16_face_detection_1080p_x11.par) using X11 as rendering method. 

## Is there any limitation of the order of decoding, encoding and dislay sessions in par file
Yes. The decoding dessions must be descripted firstly. If there is display dession, it must be the last line in par file.

## The loading time of 16-channel face detection demo is too long
Please make sure cl_cache is enabled by command "echo $cl_cache_dir". If this environment isn't set, please enable cl_cache by running command "export cl_cache_dir=/tmp/cl_cache" and "mkdir -p /tmp/cl_cache". Then after the first running of 16-channel face detection demo, the compiled OpenCL kernles are cached and the model loading time of next runnings of 16-channel face detection demo will only take about 10 seconds.
More details about cl_cache can be found at https://github.com/intel/compute-runtime/blob/master/opencl/doc/FAQ.md

## Can sources number for "-vpp_comp_only" or "-vpp_comp" be different from number of decoding sessions?  
No. The sources number for "-vpp_comp_only" or "-vpp_comp" must be equal to the numer of decoding sessions. Otherwise, the sample application will fail during pipeline initialization or running. 

## How to limit the fps of whole pipeline to 30?
Add "-fps 30" to every decoding session.

## "-fps 30" doesn't work with "-fake_sink"
Fake sink session doesn't support "-fps 30". Please add "-fps 30" to every decoding session instead.

Add "-fps 30" to every decoding session.
## How to limit the frame number of input to 1000?
Add "-n 1000" to every decoding dessions. But please not add "-n" to encode, display and fake sink session. These sink sessions will automatically stop when the source session stops. Note, this option won't work if both "-vpp_comp_only" and "-vpp_comp" are set. 

## Where can I find tutorials for inference engine?
Please refer to https://docs.openvinotoolkit.org/latest/_docs_IE_DG_Deep_Learning_Inference_Engine_DevGuide.html

## Why HDDL card usage ratio is low for face detection inference?
It can be caused by the decoded frames aren't fed to inference engine efficiently. The default inference interval of face detection is 6. You try to set the inference interval to a lower valuer when using HDDL as inference target device. For example, with 3 HDDL L2 card, adding "-infer::inverval 1" to 16-channel face detection par file can increase the HDDL usage ratio to 100%.  

## Where can I find information for the models?
Please refer to https://github.com/opencv/open_model_zoo/tree/master/models/intel. The names of models used in sample application are
face-detection-retail-0004, human-pose-estimation-0001, vehicle-attributes-recognition-barrier-0039, vehicle-license-plate-detection-barrier-0106.

## Can I use other OpenVINO version rather than 2020.3 ?
Yes, but you have to modify some code due to interfaces changing. And also you need to download the IR files and copy them to ./model manually. Please refer to script/download_and_copy_models.sh for how to download the IR files.

## When run 4 channel decode plus inference and display on APL, the CPU occupy ratio is very high and fps is low
Please refer to par file par_file/inference/n4_face_detection_rgbp.par. It uses option "-dc::rgbp" that make the SFC outputs RGB4 for display and VPP outputs RGBP for inference input. Then there is no need to use OpenCV for resizing and color conversion which consume much CPU time on APL.
Note, "-dc::rgbp" only works with "-infer::fd". Will support more inference types in the future.
