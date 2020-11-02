/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/


// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once
#pragma GCC diagnostic ignored "-Wreorder" 
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"

#include <string>
#include <vector>

#include <inference_engine.hpp>
#include <opencv2/core/core.hpp>
#include <gpu/gpu_context_api_va.hpp>
#include "human_pose.hpp"
#include "network_factory.hpp"

namespace human_pose_estimation {
class HumanPoseEstimator {
public:
    static const size_t keypointsNumber;

    HumanPoseEstimator(const std::string& modelPath,
                       const std::string& targetDeviceName,
                       bool enablePerformanceReport = false);
    int Init(bool remoteBlob, VADisplay va_dpy, int renderW, int renderH);
    std::vector<HumanPose> estimate(const cv::Mat& image);
    std::vector<HumanPose> estimate(VASurfaceID va_surface);
    ~HumanPoseEstimator();

private:

    HumanPoseEstimator(HumanPoseEstimator const&);
    HumanPoseEstimator& operator=(HumanPoseEstimator const&);

    void preprocess(const cv::Mat& image, uint8_t* buffer) const;
    std::vector<HumanPose> postprocess(
            const float* heatMapsData, const int heatMapOffset, const int nHeatMaps,
            const float* pafsData, const int pafOffset, const int nPafs,
            const int featureMapWidth, const int featureMapHeight,
            const cv::Size& imageSize) const;
    std::vector<HumanPose> extractPoses(const std::vector<cv::Mat>& heatMaps,
                                        const std::vector<cv::Mat>& pafs) const;
    void resizeFeatureMaps(std::vector<cv::Mat>& featureMaps) const;
    void correctCoordinates(std::vector<HumanPose>& poses,
                            const cv::Size& featureMapsSize,
                            const cv::Size& imageSize) const;
    bool inputWidthIsChanged(const cv::Size& imageSize);

    VADisplay mVADpy;
    bool mRemoteBlob;
    std::string mInputName;
    int minJointsNumber;
    int stride;
    cv::Vec4i pad;
    cv::Vec3f meanPixel;
    float minPeaksDistance;
    float midPointsScoreThreshold;
    float foundMidPointsRatioThreshold;
    float minSubsetScore;
    cv::Size inputLayerSize;
    int upsampleRatio;
    InferenceEngine::InferRequest mRequest;
    std::string pafsBlobName;
    std::string heatmapsBlobName;
    bool enablePerformanceReport;
    std::string modelPath;
    std::string targetDeviceName;

    NetworkInfo *mNetworkInfo;

    //Width and height of target render image for rendering results
    int mRenderW;
    int mRenderH;

};
}  // namespace human_pose_estimation
