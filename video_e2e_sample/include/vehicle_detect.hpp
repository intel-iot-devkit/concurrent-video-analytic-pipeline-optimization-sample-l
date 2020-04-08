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

#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"

#include <string>
#include <vector>

#include <inference_engine.hpp>
#include <opencv2/core/core.hpp>

struct VehicleDetectResult {
    int label;
    float confidence;
    cv::Rect location;
    std::string color;
    std::string type;
};

class VehicleDetect {
public:

    VehicleDetect(bool mEnablePerformanceReport = false);
    void Init(const std::string& detectorModelPath,
            const std::string& VAModelPath,
            const std::string& targetDeviceName);
    void Detect(const cv::Mat& image, std::vector<VehicleDetectResult>& results);
    void SetSrcImageSize(int width, int height);
    void RenderVDResults(std::vector<VehicleDetectResult>& results, cv::Mat& image);
    ~VehicleDetect();

private:

    static std::mutex mInitLock;
    float mDetectThreshold;
    InferenceEngine::InferencePlugin mPlugin;
    InferenceEngine::CNNNetwork mDetectorNetwork;
    InferenceEngine::ExecutableNetwork mVDExecutableNetwork;
    InferenceEngine::InferRequest mDetectorRequest;
    InferenceEngine::CNNNetReader mDetectorNetReader;
    int mDetectorMaxProposalCount;
    int mDetectorObjectSize;
    std::string mDetectorRoiBlobName;
    std::string mDetectorOutputName;
    bool mEnablePerformanceReport;
    cv::Size mSrcImageSize;

    InferenceEngine::CNNNetwork mVANetwork;
    InferenceEngine::ExecutableNetwork mVAExecutableNetwork;
    InferenceEngine::InferRequest mVARequest;
    InferenceEngine::CNNNetReader mVANetReader;
    std::string mVAOutputNameForColor;  // color is the first output
    std::string mVAOutputNameForType;  // type is the second output
    static const std::string mVAColors[];
    static const std::string mVATypes[];
};

