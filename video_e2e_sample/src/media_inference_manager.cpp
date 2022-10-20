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
//#include <samples/ocv_common.hpp>
#include <samples/common.hpp>
#include "sample_utils.h"
#include "media_inference_manager.h"

#include "human_pose_estimator.hpp"
#include "render_human_pose.hpp"
#include "vaapi_allocator.h"
#include "e2e_sample_infer_def.h"

#define FDUMP 0
#define LESS_P 1

#define HP_INFERENCE_DEV "GPU"

#define FD_IR_FILE_PATH "deployment_tools/open_model_zoo/tools/downloader/intel/face-detection-retail-0004/FP16/"
#ifdef USE_MOBILENET_SSD
#define FD_IR_FILE_NAME "mobilenet-ssd.xml"
#else
#define FD_IR_FILE_NAME "face-detection-retail-0004.xml"
//#define FD_IR_FILE_NAME "face-detection-adas-0001.xml"
#endif


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

//Person detection retail
#define PDR_IR_FILE_PATH "deployment_tools/open_model_zoo/tools/downloader/intel/person-detection-retail-0013/FP16/"
#define PDR_IR_FILE_NAME "person-detection-retail-0013.xml"

//Person reidentification retail
#define PREIDR_IR_FILE_PATH "deployment_tools/open_model_zoo/tools/downloader/intel/person-reidentification-retail-0288/FP16/"
#define PREIDR_IR_FILE_NAME "person-reidentification-retail-0288.xml"

#define IR_PATH_MAX_LEN 1024

using namespace cv;

MediaInferenceManager::MediaInferenceManager():
    mInferType(0),
    mTargetDevice("GPU"),
    mMaxObjNum(-1),
    mRemoteBlob(false),
    mVADpy(nullptr),
    mInit(false)
{

}

MediaInferenceManager::~MediaInferenceManager()
{
    delete mHPEstimator;
    mHPEstimator = nullptr;
    delete mObjectDetector;
    mObjectDetector = nullptr;
    delete mVehicleDetector;
    mVehicleDetector = nullptr;
    delete mMultiObjectTracker;
    mMultiObjectTracker = nullptr;
}

int MediaInferenceManager::Init(int dec_w, int dec_h,
        int infer_type, const char *model_dir,
        enum InferDeviceType device, int inferInterval, int maxObjNum,
        bool remoteBlob, VADisplay vaDpy, msdk_char* str_detectResultsavefile)
{
    int ret = 0;
    mInferType = infer_type;
    mDecW = dec_w;
    mDecH = dec_h;
    mMaxObjNum = maxObjNum;
    mInferInterval = inferInterval;
    msdk_opt_read(str_detectResultsavefile, m_strDetectResultSaveFile);

    if (remoteBlob && (!vaDpy))
    {
        return -1;
    }

    mRemoteBlob = remoteBlob;
    mVADpy = vaDpy;

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
        case InferTypeYolo:
            ret = InitFaceDetection(model_dir);
            break;
        case InferTypeHumanPoseEst:
            ret = InitHumanPose(model_dir);
            break;
        case InferTypeVADetect:
            ret = InitVehicleDetect(model_dir);
            break;
        case InferTypeMOTracker:
            ret = InitMultiObjectTracker(model_dir);
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

int MediaInferenceManager::RunInfer(mfxFrameData *pData, mfxFrameData *pData_dec, bool inferOffline, int decSurfPitch, int frameNum)
{
    if (!mInit)
    {
        return -1;
    }

    switch(mInferType)
    {
        case InferTypeFaceDetection:
            RunInferFD(pData, pData_dec, inferOffline, decSurfPitch);
            SaveFDResults(frameNum);
            break;
        default:
            msdk_printf(MSDK_STRING("ERROR:Unsupported inference type %d\n"), mInferType);
            return -1;
    }
    return 0;
}

int MediaInferenceManager::RunInfer(mfxFrameData *pData, bool inferOffline, int frameNum)
{
    if (!mInit)
    {
        return -1;
    }

    switch(mInferType)
    {
        case InferTypeFaceDetection:
        case InferTypeYolo:
            RunInferFD(pData, inferOffline);
            SaveFDResults(frameNum);
            SaveYOLOResults(frameNum);
            break;
        case InferTypeHumanPoseEst:
            RunInferHP(pData, inferOffline);
            SaveHPResults(frameNum);
            break;
        case InferTypeVADetect:
            RunInferVDVA(pData, inferOffline);
            SaveVDResults(frameNum);
            break;
        case InferTypeMOTracker:
            RunInferMOT(pData, inferOffline);
            SaveMOTResults(frameNum);
            break;
        default:
            msdk_printf(MSDK_STRING("ERROR:Unsupported inference type %d\n"), mInferType);
            return -1;
    }
    return 0;
}

int MediaInferenceManager::RenderRepeatLast(mfxFrameData *pData, bool isGrey, int decSurfPitch)
{
    if (!mInit)
    {
        return -1;
    }

    switch(mInferType)
    {
        case InferTypeFaceDetection:
            RenderRepeatLastFD(pData, true, decSurfPitch);
            break;
        default:
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
        case InferTypeYolo:
            RenderRepeatLastFD(pData);
            break;
        case InferTypeHumanPoseEst:
            RenderRepeatLastHP(pData);
            break;
        case InferTypeVADetect:
            RenderRepeatLastVD(pData);
            break;
        case InferTypeMOTracker:
            RenderRepeatLastMOT(pData);
            break;
        default:
            msdk_printf(MSDK_STRING("ERROR:Unsupported inference type %d\n"), mInferType);
            return -1;
    }
    return 0;
}


int MediaInferenceManager::RenderRepeatLastFD(mfxFrameData *pData, bool isGrey, int decSurfPitch)
{

    switch (mInferType)
    {
        case InferTypeYolo:
            if (mObjectDetector && ( mYoloResults.size() > 0)){
                unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
                Mat frameRGB4pad(mDecH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pbuf);
                Rect content(0, 0, mDecW, mDecH);
                Mat frameRGB4 = frameRGB4pad(content);
                mObjectDetector->RenderResults(mYoloResults, frameRGB4);
            }
            break;
        default:
            if (mObjectDetector && ( mFDResults.size() > 0)){
                if (isGrey)
                {
                    if (decSurfPitch < mDecW)
                    {
                        std::cout<<"MediaInferenceManager::RenderRepeatLastFD: Wrong pitch size "<<decSurfPitch<<std::endl;
                        return -1;
                    }
                    Mat frameY(mDecH, decSurfPitch, CV_8UC1, (unsigned char *)pData->Y);
                    mObjectDetector->RenderResults(mFDResults, frameY, true);
                }
                else
                {
                    Mat frameRGB4pad(mDecH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pData->B);
                    Rect content(0, 0, mDecW, mDecH);
                    Mat frameRGB4 = frameRGB4pad(content);

                    mObjectDetector->RenderResults(mFDResults, frameRGB4);
                }
            }
            break;
    }
    return 0;
}

int MediaInferenceManager::RenderRepeatLastHP(mfxFrameData *pData)
{
    if (mPoses.size() > 0){
        unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
        Mat frameRGB4pad(mDecH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pbuf);
        Rect content(0, 0, mDecW, mDecH);
        Mat frameRGB4 = frameRGB4pad(content);
        renderHumanPose(mPoses, frameRGB4);
    }

    return 0;
}

int MediaInferenceManager::RenderRepeatLastVD(mfxFrameData *pData)
{
    if (mVehicleDetector && ( mVDResults.size() > 0)){
        unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
        Mat frameRGB4pad(mDecH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pbuf);
        Rect content(0, 0, mDecW, mDecH);
        Mat frameRGB4 = frameRGB4pad(content);
        mVehicleDetector->RenderVDResults(mVDResults, frameRGB4);
    }

    return 0;
}

int MediaInferenceManager::RenderRepeatLastMOT(mfxFrameData* pData)
{
    if (mMultiObjectTracker) {
        mMultiObjectTracker->RenderRepeatLast(pData);
    }

    return 0;
}

int MediaInferenceManager::RunInferHP(mfxFrameData *pData, bool inferOffline)
{
    if (!mRemoteBlob)
    {
        unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
        Mat frameRGB4pad(mDecH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pbuf);
        Rect content(0, 0, mDecW, mDecH);
        Mat frameRGB4 = frameRGB4pad(content);
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
    }
    else
    {
        VASurfaceID *surface =  ((vaapiMemId*)pData->MemId)->m_surface;
        mPoses = mHPEstimator->estimate(*surface);
    }

    return 0;
}

int MediaInferenceManager::RunInferFD(mfxFrameData *pData, mfxFrameData *pData_dec, bool inferOffline, int decSurfPitch)
{
    int alignedH = MSDK_ALIGN16(mInputH);
    int alignedW = MSDK_ALIGN32(mInputW);

    if (mFDResults.size() > 0)
    {
        mFDResults.clear();
    }

    mObjectDetector->Detect(pData,pData_dec, mFDResults);

    if (!inferOffline)
    {
        if (decSurfPitch < mDecW)
        {
            std::cout<<"MediaInferenceManager::RunInferFD Wrong pitch size "<<decSurfPitch<<std::endl;
        }
        Mat frameY(mDecH, decSurfPitch, CV_8UC1, (unsigned char *)pData_dec->Y);

        /* Bounding box */
        mObjectDetector->RenderResults(mFDResults, frameY, true);
    }

    return 0;
}

int MediaInferenceManager::RunInferFD(mfxFrameData *pData, bool inferOffline)
{
    if (!mRemoteBlob)
    {
        
        unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
        Mat frameRGB4pad(mDecH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pbuf);
        Rect content(0, 0, mDecW, mDecH);
        Mat frameRGB4 = frameRGB4pad(content);
        Mat frame(mInputH, mInputW, CV_8UC3);
        if (mDecH == mInputH && mInputW == mDecW)
        {
            cvtColor(frameRGB4, frame, COLOR_RGBA2RGB);
        }
        else
        {
            Mat frameScl(mInputH, mInputW, CV_8UC4);
            resize(frameRGB4, frameScl, Size(mInputW, mInputH));
            cvtColor(frameScl, frame, COLOR_RGBA2RGB);
        }

        switch(mInferType)
        {
            case InferTypeYolo:
                if (mYoloResults.size() > 0)
                {
                    mYoloResults.clear();
                }
                mObjectDetector->Detect(frame, mYoloResults);

                if (!inferOffline)
                {
                    /* Bounding box */
                    mObjectDetector->RenderResults(mYoloResults, frameRGB4);
                }
                break;
            default:
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
                break;
        }
    }
    else
    {
        VASurfaceID *surface =  ((vaapiMemId*)pData->MemId)->m_surface;
        mObjectDetector->Detect(*surface, mFDResults);
    }

    return 0;
}

int MediaInferenceManager::RunInferVDVA(mfxFrameData *pData, bool inferOffline)
{
    unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;

    Mat frameRGB4pad(mDecH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pbuf);
    Rect content(0, 0, mDecW, mDecH);
    Mat frameRGB4 = frameRGB4pad(content);
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

int MediaInferenceManager::RunInferMOT(mfxFrameData* pData, bool inferOffline)
{
    if (mMultiObjectTracker) {
        mMultiObjectTracker->RunInfer(pData, inferOffline);
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
    mHPEstimator->Init(mRemoteBlob, mVADpy, mDecW, mDecH);

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
        unsigned int len = strnlen(model_dir, IR_PATH_MAX_LEN);
        /* It's IR XML file */
        if ( len > 4 && strncmp(model_dir + len - 4, ".xml", 4) == 0)
        {
            snprintf(ir_file, IR_PATH_MAX_LEN, "%s", model_dir);
        }
        else
        {
            snprintf(ir_file, IR_PATH_MAX_LEN, "%s/%s", model_dir, file_name);
        }
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

    mBatchId = 1;

    mObjectDetector = new ObjectDetect(false);
    mObjectDetector->Init(ir_file, mInferType, mTargetDevice, mRemoteBlob, mVADpy);
    mObjectDetector->GetInputSize(mInputW, mInputH);
    mObjectDetector->SetSrcImageSize(mDecW, mDecH);

    return 0;
}

int MediaInferenceManager::InitMultiObjectTracker(const char* modelDir)
{
    char detIrFile[IR_PATH_MAX_LEN] = { 0 };
    char reidIrFile[IR_PATH_MAX_LEN] = { 0 };

    if (0 != GetFullIRPath(modelDir, PDR_IR_FILE_PATH, PDR_IR_FILE_NAME, detIrFile))
    {
        return -1;
    }

    if (0 != GetFullIRPath(modelDir, PREIDR_IR_FILE_PATH, PREIDR_IR_FILE_NAME, reidIrFile))
    {
        return -1;
    }

    mMultiObjectTracker = new MultiObjectTracker();
    mMultiObjectTracker->Init(detIrFile, reidIrFile, mTargetDevice, mInferInterval, mRemoteBlob, mVADpy);
    mMultiObjectTracker->SetRenderSize(mDecW, mDecH);
    return 0;
}

int savetofile(std::string filename, std::string& str)
{
    try{
        std::ofstream saveFile;
        saveFile.open(filename,std::ios::app);
        saveFile<< str;
        saveFile.close();
    }
    catch(...){
        std::cout<<"Cann't save Detect results to "<< filename<< " !"<<std::endl;
        return -1;
    }
    return 0;
}

int MediaInferenceManager::SaveFDResults(int frameNum)
{
    if(!std::string(m_strDetectResultSaveFile).empty() && mFDResults.size()>0)
    {
        std::string str;
        str.reserve(100);
        str.append(std::to_string(frameNum));
        str.append("\t");
        for (auto &object : mFDResults)
        {
            if (object.confidence < 0.6)
                continue;
            str.append("{ ");
            str.append(std::to_string(object.label));
            str.append("\t");
            str.append(std::to_string(object.confidence));
            str.append("\t");
            str.append(std::to_string(object.location.x));
            str.append(",");
            str.append(std::to_string(object.location.y));
            str.append(",");
            str.append(std::to_string(object.location.width));
            str.append(",");
            str.append(std::to_string(object.location.height));
            str.append(" }\t");
        }
        str.append("\n");
        savetofile(m_strDetectResultSaveFile,str);
    }
    return 0;
}

int MediaInferenceManager::SaveYOLOResults(int frameNum)
{
    if(!std::string(m_strDetectResultSaveFile).empty() && mYoloResults.size()>0)
    {
        std::string str;
        str.reserve(100);
        str.append(std::to_string(frameNum));
        str.append("\t");
        for (auto &object : mYoloResults)
        {
            if (object.confidence < 0.6)
                continue;
            str.append("{ ");
            str.append(std::to_string(object.class_id));
            str.append("\t\t");
            str.append(std::to_string(object.confidence));
            str.append("\t\t");
            str.append(std::to_string(object.xmin));
            str.append(",");
            str.append(std::to_string(object.ymin));
            str.append(",");
            str.append(std::to_string(object.xmax));
            str.append(",");
            str.append(std::to_string(object.ymax));
            str.append(" }\t");
        }
        str.append("\n");
        savetofile(m_strDetectResultSaveFile,str);
    }
    return 0;
}

int MediaInferenceManager::SaveHPResults(int frameNum)
{
    if(!std::string(m_strDetectResultSaveFile).empty())
    {
        std::string str;
        str.reserve(100);
        str.append(std::to_string(frameNum));
        str.append("\t");
        for (auto &pose : mPoses)
        {
            str.append("{ ");
            str.append(std::to_string(pose.score));
            str.append("\t\t");
            for(auto &point : pose.keypoints)
            {
                str.append(std::to_string(point.x));
                str.append(",");
                str.append(std::to_string(point.y));
                str.append("\t");
            }
            str.append(" }\t");
        }
        str.append("\n");
        savetofile(m_strDetectResultSaveFile,str);
    }
    return 0;
}

int MediaInferenceManager::SaveVDResults(int frameNum)
{
    if(!std::string(m_strDetectResultSaveFile).empty())
    {
        std::string str;
        str.reserve(100);
        str.append(std::to_string(frameNum));
        str.append("\t");
        for (auto &object : mVDResults)
        {
            if (object.confidence < 0.6)
                continue;
            str.append("{ ");
            str.append(std::to_string(object.label));
            str.append("\t");
            str.append(std::to_string(object.confidence));
            str.append("\t");
            str.append(std::to_string(object.location.x));
            str.append(",");
            str.append(std::to_string(object.location.y));
            str.append(",");
            str.append(std::to_string(object.location.width));
            str.append(",");
            str.append(std::to_string(object.location.height));
            str.append(" }\t\t");
        }
        str.append("\n");
        savetofile(m_strDetectResultSaveFile,str);
    }
    return 0;
}

int MediaInferenceManager::SaveMOTResults(int frameNum)
{
    if(!std::string(m_strDetectResultSaveFile).empty())
    {
        std::string str;
        str.reserve(100);
        str.append(std::to_string(frameNum));
        str.append("\t MotDetector:[");
        for (const auto& detection : mMultiObjectTracker->GetMotDetectors())
        {
            str.append("{ ");
            str.append(std::to_string(detection.object_id));
            str.append("\t");
            str.append(std::to_string(detection.confidence));
            str.append("\t");
            str.append(std::to_string(detection.rect.x));
            str.append(",");
            str.append(std::to_string(detection.rect.y));
            str.append(",");
            str.append(std::to_string(detection.rect.width));
            str.append(",");
            str.append(std::to_string(detection.rect.height));
            str.append(" }\t");
        }
        str.append("]\t\t mMotTracker:[");
        for (const auto& detection : mMultiObjectTracker->GetMotTrackerDetections())
        {
            str.append("{ ");
            str.append(std::to_string(detection.object_id));
            str.append("\t");
            str.append(std::to_string(detection.confidence));
            str.append("\t");
            str.append(std::to_string(detection.rect.x));
            str.append(",");
            str.append(std::to_string(detection.rect.y));
            str.append(",");
            str.append(std::to_string(detection.rect.width));
            str.append(",");
            str.append(std::to_string(detection.rect.height));
            str.append(" }\t");
        }
        str.append("]\n");
        savetofile(m_strDetectResultSaveFile,str);
    }
    return 0;
}
