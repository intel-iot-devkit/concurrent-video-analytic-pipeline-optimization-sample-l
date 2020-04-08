We welcome community contributions to SVET sample application. Thank you for your time!

Please note that review and merge might take some time at this point.

SVET sample application is licensed under MIT license. By contributing to the project, you agree to the license and copyright terms therein and release your contribution under these terms.

Steps:
 - In the commit message, explain what bug is fixed or what new feature is added in details.
 - Validate that your changes don't break a build. [build instruction](../README.md#how-to-build)
 - Pass [testing](#testing)
 - Wait while your patchset is reviewed and tested by our internal validation cycle

# Testing

## Requirements

Hardware Requirements: Coffee Lake or Whiskey Lake
Software Requirements: Ubuntu 18.04, MediaSDK 19.4.0 and OpenVINO 2019 R3 

## How to test your changes

### 1. Build the SVET sample applicaton

```sh
./build_and_install.sh
```
Apply the changes to MediaSDK/samples/video_e2e_sample/. Then build video_e2e_sample:
```sh
cp msdk_build.sh MediaSDK
cd MediaSDK
./msdk_build.sh
```

### 2. Run tests to make sure below tests can run without error 

Simple display test:
```sh
./bin/video_e2e_sample -par par_file/par_file_name.par
```
Basic video decoding and inference tests:
n16_1080p_1080p_dp_noinfer.par
n16_1080p_1080p_dp.par
n16_1080p_4k_dp.par
n4_1080p_1080p_dp.par
n4_vehical_detect_1080p.par
n64_d1_1080p_dp.par

RTSP play and saving tests:
rtsp_dump_only.par
n16_1080p_rtsp_simu.pa
n4_1080p_rtsp_simu_dp.par
n16_1080p_rtsp_simu_dump.par

Multiple display tests:
```sh
./bin/video_e2e_sample -par par_file/n16_1080p_1080p_dp_noinfer.par -par par_file/n16_1080p_1080p_dp.par
```
 
