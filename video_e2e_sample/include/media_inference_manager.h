/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#pragma once

#include "sample_utils.h"
#include "human_pose_estimator.hpp"
#include "vehicle_detect.hpp"
#include "object_detect.hpp"
#include "e2e_sample_infer_def.h"
#include "multi_object_tracker.hpp"

using namespace human_pose_estimation;

class MediaInferenceManager
{
public:

    MediaInferenceManager();
    ~MediaInferenceManager();

    enum InferDeviceType {InferDeviceGPU, InferDeviceCPU, InferDeviceHDDL };

    int Init(int dec_w, int dec_h, int infer_type, const char *model_dir,
            enum InferDeviceType device, int inferInterval, int maxObjNum, bool remoteBlob, VADisplay vaDpy);
    int GetInferInterval();
    /* If inferOffline is true, the results won't be render to input surface */
    int RunInfer(mfxFrameData *data, bool inferOffline);
    int RunInfer(mfxFrameData *data, mfxFrameData *data_dec, bool inferOffline, int decSurfPitch);
    int RenderRepeatLast(mfxFrameData *data);
    int RenderRepeatLast(mfxFrameData *pData, bool isGrey, int decSurfPitch);

private:
    MediaInferenceManager(MediaInferenceManager const&);
    MediaInferenceManager& operator=(MediaInferenceManager const&);

    int GetFullIRPath(const char *model_dir, const char *file_path,
            const char *file_name, char *ir_file);
    int InitFaceDetection(const char *model_dir);
    int RunInferFD(mfxFrameData *pData, bool inferOffline);
    int RunInferFD(mfxFrameData *data, mfxFrameData *data_dec, bool inferOffline, int decSurfPitch);
    int RenderRepeatLastFD(mfxFrameData *pData, bool isGrey = false, int decSurfPitch = 0);

    int InitHumanPose(const char *model_dir);
    int RunInferHP(mfxFrameData *pData, bool inferOffline);
    int RenderRepeatLastHP(mfxFrameData *pData);

    int InitVehicleDetect(const char *model_dir);
    int RunInferVDVA(mfxFrameData *pData, bool inferOffline);
    int RenderRepeatLastVD(mfxFrameData *pData);

    int InitMultiObjectTracker(const char* model_dir);
    int RunInferMOT(mfxFrameData* pData, bool inferOffline);
    int RenderRepeatLastMOT(mfxFrameData* pData);



    int mInferType;
    int mDecW;
    int mDecH;
    int mInputW;
    int mInputH;
    int mBatchId;
    int mMaxObjNum;
    std::string mTargetDevice;

    int mInferInterval;
    int mInferDevType;
    int mInit;

    VADisplay mVADpy = NULL;
    bool mRemoteBlob;
    /*Face Detection*/
    ObjectDetect *mObjectDetector = nullptr;
    std::vector<ObjectDetectResult> mFDResults;
    std::vector<DetectionObject> mYoloResults;

    /*Human Pose Estimation*/
    HumanPoseEstimator *mHPEstimator = nullptr;
    std::vector<HumanPose> mPoses;

    /*Vehicle and Vehicle attributes detection*/
    VehicleDetect *mVehicleDetector = nullptr;
    std::vector<VehicleDetectResult> mVDResults;

    /*Multi object Tracker*/
    MultiObjectTracker *mMultiObjectTracker = nullptr;
};

