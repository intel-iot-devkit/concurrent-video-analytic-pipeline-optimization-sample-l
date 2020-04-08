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

#include <vector>

#include <opencv2/core/core.hpp>

#include "human_pose.hpp"

namespace human_pose_estimation {
struct Peak {
    Peak(const int id = -1,
         const cv::Point2f& pos = cv::Point2f(),
         const float score = 0.0f);

    int id;
    cv::Point2f pos;
    float score;
};

struct HumanPoseByPeaksIndices {
    explicit HumanPoseByPeaksIndices(const int keypointsNumber);

    std::vector<int> peaksIndices;
    int nJoints;
    float score;
};

struct TwoJointsConnection {
    TwoJointsConnection(const int firstJointIdx,
                        const int secondJointIdx,
                        const float score);

    int firstJointIdx;
    int secondJointIdx;
    float score;
};

void findPeaks(const std::vector<cv::Mat>& heatMaps,
               const float minPeaksDistance,
               std::vector<std::vector<Peak> >& allPeaks,
               int heatMapId);

std::vector<HumanPose> groupPeaksToPoses(
        const std::vector<std::vector<Peak> >& allPeaks,
        const std::vector<cv::Mat>& pafs,
        const size_t keypointsNumber,
        const float midPointsScoreThreshold,
        const float foundMidPointsRatioThreshold,
        const int minJointsNumber,
        const float minSubsetScore);
}  // namespace human_pose_estimation
