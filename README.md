# Concurrent Video Analytic Pipeline Optimzation Sample 
Support users to quickly setup and adjust the core concurrent video analysis workload through configuration file to obtain the best performance of video codec, post-processing and inference based on Intel® integrated GPU according to their product requirements.
Users can use the sample application video_e2e_sample to complete runtime performance evaluation or as a reference for debugging core video workload issues.

## Typical workloads
Sample par files can be found in par_files directory. Verfied on i7-8559U. Performance differs on other platforms.
* 16 1080p H264 decoding, scaling, face detection inference, rendering inference results, composition, saving composition results to local H264 file, and display
* 4 1080p H264 decoding, scaling, human pose estimation inference, rendering inference results, composition and display
* 4 1080p H264 decoding, scaling, vehicle and vehicle attributes detection inference, rendering inference results, composition and display
* 16 1080p RTSP H264 stream decoding, scaling, face detection inference, rendering inference results, composition and display.
* 16 1080p H264 decoding, scaling, face detection inference, rendering inference results, composition and display. Plus 16 1080p H264 decoding, composition and showing on second display. 

# Dependencies
The sample application depends on [Intel® Media SDK](https://github.com/Intel-Media-SDK/), [Intel® OpenVINO™](https://software.intel.com/en-us/openvino-toolkit) and [FFmpeg](https://www.ffmpeg.org/)

# FAQ
See [FAQ](./doc/FAQ.md)

# Table of contents

  * [License](#license)
  * [How to contribute](#how-to-contribute)
  * [Documentation](#documentation)
  * [System requirements](#system-requirements)
  * [How to build](#how-to-build)
    * [Build steps](#build-steps)
  * [Known limitations](#know-limitations)

# License
The sample application is licensed under MIT license. See [LICENSE](./video_e2e_sample/LICENSE) for details.

# How to contribute
See [CONTRIBUTING](./doc/CONTRIBUTING.md) for details. Thank you!

# Documentation
See [user guide](./doc/concurrent_video_analytic_sample_application_user_guide.pdf)

# System requirements

**Operating System:**
* Ubuntu 20.04.03

**Software:**
* [OneVPL dispather v2022.0.3] (https://github.com/oneapi-src/oneVPL/archive/refs/tags/v2022.0.3.tar.gz)
* [OneVPL GPU 22.3.2] (https://github.com/oneapi-src/oneVPL-intel-gpu/archive/refs/tags/intel-onevpl-22.3.2.tar.gz)
* [OpenVINO™ 2022.1](https://software.intel.com/en-us/openvino-toolkit)

**Hardware:** 
* Intel® platforms supported by the OneVPL GPU 22.3.2 and OpenVINO 2022.1. 
* For OneVPL GPU, the major platform dependency comes from the back-end media driver. https://github.com/intel/media-driver
* For OpenVINO™, see details from here: https://software.intel.com/en-us/openvino-toolkit/documentation/system-requirements

# How to build

Run build_and_install.sh to install dependent software packages and build sample application video_e2e_sample. 

Please refer to ”Installation Guide“ in [user guide](./doc/concurrent_video_analytic_sample_application_user_guide.pdf) for details.

Below are the main steps

## Install OpenVINO 2022.1, OpenCV and NEO 
Install OpenVINO 2022.1 from https://software.intel.com/en-us/openvino-toolkit

Then install OpenCV:
```sh
cd /opt/intel/openvino_2022/extras/scripts/
sudo -E download_opencv.sh
```

If you see errors, you can also try below commands:
```sh
su
cd /opt/intel/openvino_2022/extras/scripts/
wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2022.1/opencv/openvino_opencv_ubuntu20.tgz
mv openvino_opencv_ubuntu20.tgz ../../
cd ../../
tar zxvf openvino_opencv_ubuntu20.tgz
```

Install NEO manually according to instruction on https://github.com/intel/compute-runtime/releases/tag/22.20.23198

```sh
mkdir neo; cd neo
wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.11222/intel-igc-core_1.0.11222_amd64.deb
wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.11222/intel-igc-opencl_1.0.11222_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/22.20.23198/intel-level-zero-gpu-dbgsym_1.3.23198_amd64.ddeb
wget https://github.com/intel/compute-runtime/releases/download/22.20.23198/intel-level-zero-gpu_1.3.23198_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/22.20.23198/intel-opencl-icd-dbgsym_22.20.23198_amd64.ddeb
wget https://github.com/intel/compute-runtime/releases/download/22.20.23198/intel-opencl-icd_22.20.23198_amd64.deb
wget https://github.com/intel/compute-runtime/releases/download/22.20.23198/libigdgmm12_22.1.2_amd64.deb

sudo dpkg -i *.deb
```
Add user to group video and render:
```sh
usermod -a -G video $USER
usermod -a -G render $USER
```

At last, log out and log in again to make above change take effect.

## Build steps
Get sources with the following git command: 
```sh
git clone https://github.com/intel-iot-devkit/concurrent-video-analytic-pipeline-optimization-sample-l.git cva_sample 
```

```sh
cd cva_sample 
./build_and_install.sh
```
This script will install the dependent software packages by running command "apt install". So it will ask for sudo password. Then it will download libva, libva-util, gmmlib, media-driver, onevpl and onevpl_gpu source code and install these libraries. It might take 10 to 20 minutes depending on the network bandwidth.

After the script finishing, the sample application video_e2e_sample can be found under ./bin.

In order to enable the libva/media-driver/onevpl installed by SVET to coexist with different versions of libva/media-driver/onevpl installed on the same computer, we suggest (we have done so in our build script) that the media SDK environment variables of SVET should only be set in the current bash, not saved to the global system environment.
So please run 'source ./svet_env_setup.sh' first when you start a new shell (or change user in shell such as run 'su -') to run ./bin/video_e2e_sample".

```sh
cd cva_sample
source ./svet_env_setup.sh
./bin/video_e2e_sample -par sanity_test/n1nodsp.par
./bin/video_e2e_sample -par sanity_test/n2fd.par
```
n1nodsp.par is a basic test to check if the media stack work correctly.
n2fd.par runs two video decode and face detection inference without display. It can be used to check if the OpenVINO and NEO installed correctly.
Please refer to "Run sample application" for more use cases in [user guide](./doc/concurrent_video_analytic_sample_application_user_guide.pdf) for details.

# Known limitations

The sample application has been validated on Intel® platforms Tiger Lake U(i7-1185G7E, i5-1135G7E, Celeron 6305E) and Adler Lake (i5-12400) 


