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
#include <string>
#include <map>
#include <utility>
#include <memory>
#include <vector>
#include <iostream>
#include <chrono>
#include <string>
#include <opencv2/core.hpp>
#include "mot_core.hpp"
#include "mot_utils.hpp"
#include "mot_tracker.hpp"
#include "mot_descriptor.hpp"
#include "mot_distance.hpp"
#include "mot_object_detector.hpp"
#include "network_factory.hpp"

#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"

class MultiObjectTracker {
public:
    MultiObjectTracker();

    int Init(const std::string& detModelPath,
        const std::string& reidModelPath,
        const std::string& targetDeviceName = "GPU", int inferInterval = 6,
        bool remoteBlob = false, VADisplay vaDpy = NULL);
    int RunInfer(mfxFrameData* pData, bool inferOffline);
    int RenderRepeatLast(mfxFrameData* pData);
    void SetRenderSize(int width, int height);

    ~MultiObjectTracker() {};

private:
    std::unique_ptr<MotPedestrianTracker> mMotTracker;
    std::unique_ptr<MotObjectDetector> mMotDetector;
    unsigned int mFrameIdx;
    int mRenderW;
    int mRenderH;
    int mInputW;
    int mInputH;
    float mRatioW;
    float mRatioH;
    int mInferInterval;


    std::unique_ptr<MotPedestrianTracker> createMotTracker(const std::string& reidModel,
        const std::string& deviceName, int inferInterval,
        bool shouldKeepTrackingInfo);

    std::unique_ptr<MotObjectDetector> createMotDetector(const std::string& detModel,
        const std::string& deviceName, bool remoteBlob, VADisplay vaDpy);

    void renderResult(cv::Mat& frame);

    void setScaleRatio();



};