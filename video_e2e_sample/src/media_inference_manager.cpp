/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <unistd.h>
#include <samples/ocv_common.hpp>
#include <samples/common.hpp>
#include "sample_utils.h"
#include "media_inference_manager.h"

#include "human_pose_estimator.hpp"
#include "render_human_pose.hpp"

#define FDUMP 0
#define LESS_P 1

#define HP_INFERENCE_DEV "GPU"

#define FD_IR_FILE_PATH "deployment_tools/open_model_zoo/tools/downloader/intel/face-detection-retail-0004/FP16/"
#define FD_IR_FILE_NAME "face-detection-retail-0004.xml"


#define HP_IR_FILE_PATH "deployment_tools/open_model_zoo/tools/downloader/intel/human-pose-estimation-0001/FP16/"
#define HP_IR_FILE_NAME "human-pose-estimation-0001.xml"

#define VA_IR_FILE_PATH "deployment_tools/open_model_zoo/tools/downloader/intel/vehicle-attributes-recognition-barrier-0039/FP16/"
#define VA_IR_FILE_NAME "vehicle-attributes-recognition-barrier-0039.xml"

//vehicle-license-plate-detection-barrier-0106.xml has bettern performance but can't detect car if the car image size is too small
#define VD_LPD_MODEL
#ifdef VD_LPD_MODEL
#define VD_IR_FILE_PATH "deployment_tools/open_model_zoo/tools/downloader/intel/vehicle-license-plate-detection-barrier-0106/FP16/"
#define VD_IR_FILE_NAME "vehicle-license-plate-detection-barrier-0106.xml"
#else
#define VD_IR_FILE_PATH "deployment_tools/open_model_zoo/tools/downloader/intel/vehicle-detection-adas-0002/FP16/"
#define VD_IR_FILE_NAME "vehicle-detection-adas-0002.xml"
#endif

#define IR_PATH_MAX_LEN 1024

using namespace cv;

MediaInferenceManager::MediaInferenceManager():
    mInferType(0),
    mTargetDevice("GPU"),
    mMaxObjNum(-1)
{
    mInit = false;
}

MediaInferenceManager::~MediaInferenceManager()
{
    delete mHPEstimator;
    mHPEstimator = nullptr;
    delete mObjectDetector;
    mObjectDetector = nullptr;
    delete mVehicleDetector;
    mVehicleDetector = nullptr;
}

int MediaInferenceManager::Init(int dec_w, int dec_h,
        int infer_type, const char *model_dir,
        enum InferDeviceType device, int maxObjNum)
{
    int ret = 0;
    mInferType = infer_type;
    mDecW = dec_w;
    mDecH = dec_h;
    mMaxObjNum = maxObjNum;

    switch(device)
    {
        case InferDeviceGPU:
            mTargetDevice = "GPU";
            break;
        case InferDeviceCPU:
            mTargetDevice = "CPU";
            break;
        case InferDeviceHDDL:
            mTargetDevice = "HDDL";
            break;
        default:
            break;
    }

    switch(mInferType)
    {
        case InferTypeFaceDetection:
            ret = InitFaceDetection(model_dir);
            break;
        case InferTypeHumanPoseEst:
            ret = InitHumanPose(model_dir);
            break;
        case InferTypeVADetect:
            ret = InitVehicleDetect(model_dir);
            break;
        default:
            msdk_printf(MSDK_STRING("ERROR:Unsupported inference type %d\n"), mInferType);
            ret = -1;
            break;
    }
    if (ret == 0)
    {
        mInit = true;
    }
    return ret;
}

int MediaInferenceManager::RunInfer(mfxFrameData *pData, bool inferOffline)
{
    if (!mInit)
    {
        return -1;
    }

    switch(mInferType)
    {
        case InferTypeFaceDetection:
            RunInferFD(pData, inferOffline);
            break;
        case InferTypeHumanPoseEst:
            RunInferHP(pData, inferOffline);
            break;
        case InferTypeVADetect:
            RunInferVDVA(pData, inferOffline);
            break;
        default:
            msdk_printf(MSDK_STRING("ERROR:Unsupported inference type %d\n"), mInferType);
            return -1;
    }
    return 0;
}

int MediaInferenceManager::RenderRepeatLast(mfxFrameData *pData)
{
    if (!mInit)
    {
        return -1;
    }

    switch(mInferType)
    {
        case InferTypeFaceDetection:
            RenderRepeatLastFD(pData);
            break;
        case InferTypeHumanPoseEst:
            RenderRepeatLastHP(pData);
            break;
        case InferTypeVADetect:
            RenderRepeatLastVD(pData);
            break;
        default:
            msdk_printf(MSDK_STRING("ERROR:Unsupported inference type %d\n"), mInferType);
            return -1;
    }
    return 0;
}


int MediaInferenceManager::RenderRepeatLastFD(mfxFrameData *pData)
{
    if (mObjectDetector && ( mFDResults.size() > 0)){
        Mat frameRGB4(mDecH, mDecW, CV_8UC4, (unsigned char *)pData->B);
        mObjectDetector->RenderResults(mFDResults, frameRGB4);
    }
    return 0;
}

int MediaInferenceManager::RenderRepeatLastHP(mfxFrameData *pData)
{
    if (mPoses.size() > 0){
        Mat frameRGB4(mDecH, mDecW, CV_8UC4, (unsigned char *)pData->B);
        renderHumanPose(mPoses, frameRGB4);
    }

    return 0;
}

int MediaInferenceManager::RenderRepeatLastVD(mfxFrameData *pData)
{
    if (mVehicleDetector && ( mVDResults.size() > 0)){
        Mat frameRGB4(mDecH, mDecW, CV_8UC4, (unsigned char *)pData->B);
        mVehicleDetector->RenderVDResults(mVDResults, frameRGB4);
    }

    return 0;
}

int MediaInferenceManager::RunInferHP(mfxFrameData *pData, bool inferOffline)
{
    unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
    Mat frameRGB4(mDecH, mDecW, CV_8UC4, (unsigned char *)pbuf);
    Mat frameScl(mInputH, mInputH, CV_8UC4);
    Mat frame(mInputH, mInputW, CV_8UC3);

    resize(frameRGB4, frameScl, Size(mInputW, mInputH));
    cvtColor(frameScl, frame, COLOR_RGBA2RGB);

    if (mPoses.size() > 0)
    {
        mPoses.clear();
    }
    mPoses = mHPEstimator->estimate(frame);

    if (!inferOffline)
    {
        renderHumanPose(mPoses, frameRGB4);
    }

    return 0;
}

int MediaInferenceManager::RunInferFD(mfxFrameData *pData, bool inferOffline)
{
    unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
    Mat frameRGB4(mDecH, mDecW, CV_8UC4, (unsigned char *)pbuf);
    Mat frameScl(mInputH, mInputH, CV_8UC4);
    Mat frame(mInputH, mInputW, CV_8UC3);

    resize(frameRGB4, frameScl, Size(mInputW, mInputH));
    cvtColor(frameScl, frame, COLOR_RGBA2RGB);

    if (mFDResults.size() > 0)
    {
        mFDResults.clear();
    }
    mObjectDetector->Detect(frame, mFDResults);

    if (!inferOffline)
    {
        /* Bounding box */
        mObjectDetector->RenderResults(mFDResults, frameRGB4);
    }

    return 0;
}

int MediaInferenceManager::RunInferVDVA(mfxFrameData *pData, bool inferOffline)
{
    unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
    Mat frameRGB4(mDecH, mDecW, CV_8UC4, (unsigned char *)pbuf);
    Mat frameScl(mInputH, mInputH, CV_8UC4);
    Mat frame(mInputH, mInputW, CV_8UC3);

    resize(frameRGB4, frameScl, Size(mInputW, mInputH));
    cvtColor(frameScl, frame, COLOR_RGBA2RGB);

    if (mVDResults.size() > 0)
    {
        mVDResults.clear();
    }

    mVehicleDetector->Detect(frame, mVDResults, mMaxObjNum);

    if (!inferOffline)
    {
        mVehicleDetector->RenderVDResults(mVDResults, frameRGB4);
    }

    return 0;
}

int MediaInferenceManager::InitHumanPose(const char *model_dir)
{
    mInputW = 456;
    mInputH = 256;

    char ir_file[IR_PATH_MAX_LEN] = {0};

    if (0 != GetFullIRPath(model_dir, HP_IR_FILE_PATH, HP_IR_FILE_NAME, ir_file))
    {
        return -1;
    }

    mHPEstimator = new HumanPoseEstimator(ir_file, mTargetDevice, false);
    mHPEstimator->Init();

    return 0;
}


int MediaInferenceManager::GetFullIRPath(const char *model_dir, const char *file_path, const char *file_name, char *ir_file)
{
    char *openvino_dir = nullptr;
#if 0
    //only for debug
    if (getenv("MEDIA_AI_USE_OPENVINO_MODEL"))
    {
        openvino_dir = getenv("INTEL_OPENVINO_DIR");
        if (openvino_dir)
        {
            snprintf(ir_file, IR_PATH_MAX_LEN, "%s/%s/%s", openvino_dir, file_path, file_name);
        }
        else
        {
            msdk_printf(MSDK_STRING("ERROR:env MEDIA_AI_USE_OPENVINO_MODEL is set but env INTEL_OPENVINO_DIR isn't set\n"));
            return -1;
        }
    }
    else
#endif
    {
        snprintf(ir_file, IR_PATH_MAX_LEN, "%s/%s", model_dir, file_name);
    }

    if (0 != access(ir_file, R_OK))
    {
        msdk_printf(MSDK_STRING("ERROR:Not able to open IR file %s. Please check if the IR file path is correct\n"), ir_file);
        return -1;
    }
    std::string str_ir_xml(ir_file);
    std::string str_ir_bin = fileNameNoExt(str_ir_xml) + ".bin";
    if (0 != access(str_ir_bin.c_str(), R_OK))
    {
        msdk_printf(MSDK_STRING("ERROR:Not able to open IR file %s. Please check if the IR file path is correct\n"), str_ir_bin.c_str());
        return -1;
    }
    return 0;
}

int MediaInferenceManager::InitVehicleDetect(const char *model_dir)
{
#ifdef VD_LPD_MODEL
    mInputW = 300;
    mInputH = 300;
#else
    mInputW = 672;
    mInputH = 384;
#endif

    char ir_file_vd[IR_PATH_MAX_LEN] = {0};
    char ir_file_va[IR_PATH_MAX_LEN] = {0};
    char *openvino_dir;

    if (0 != GetFullIRPath(model_dir, VD_IR_FILE_PATH, VD_IR_FILE_NAME, ir_file_vd))
    {
        return -1;
    }
    if (0 != GetFullIRPath(model_dir, VA_IR_FILE_PATH, VA_IR_FILE_NAME, ir_file_va))
    {
        return -1;
    }

    mVehicleDetector = new VehicleDetect(false);
    mVehicleDetector->Init(ir_file_vd, ir_file_va, mTargetDevice);
    mVehicleDetector->SetSrcImageSize(mDecW, mDecH);

    return 0;
}


int MediaInferenceManager::InitFaceDetection(const char *model_dir)
{
    char ir_file[IR_PATH_MAX_LEN] = {0};
    char *openvino_dir;

    if (0 != GetFullIRPath(model_dir, FD_IR_FILE_PATH, FD_IR_FILE_NAME, ir_file))
    {
        return -1;
    }

    mInputW = 300;
    mInputH = 300;
    mBatchId = 1;

    mObjectDetector = new ObjectDetect(false);
    mObjectDetector->Init(ir_file, mTargetDevice);
    mObjectDetector->SetSrcImageSize(mDecW, mDecH);

    return 0;
}
