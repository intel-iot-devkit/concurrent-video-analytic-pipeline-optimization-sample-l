// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#include "mot_utils.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <vector>
#include <map>
#include <string>
#include <set>
#include <memory>

using namespace InferenceEngine;

namespace {
template <typename StreamType>
void SaveDetectionLogToStream(StreamType& stream,
                              const DetectionLog& log) {
    for (const auto& entry : log) {
        std::vector<TrackedObject> objects(entry.objects.begin(),
                                           entry.objects.end());
        std::sort(objects.begin(), objects.end(),
                  [](const TrackedObject& a,
                     const TrackedObject& b)
                  { return a.object_id < b.object_id; });
        for (const auto& object : objects) {
            auto frame_idx_to_save = entry.frame_idx;
            stream << frame_idx_to_save << ',';
            stream << object.object_id << ','
                << object.rect.x << ',' << object.rect.y << ','
                << object.rect.width << ',' << object.rect.height;
            stream << '\n';
        }
    }
}
}  // anonymous namespace

void DrawPolyline(const std::vector<cv::Point>& polyline,
                  const cv::Scalar& color, cv::Mat* image, int lwd, float ratiow, float ratioh) {
    PT_CHECK(image);
    PT_CHECK(!image->empty());
 //   PT_CHECK_EQ(image->type(), CV_8UC3);
    PT_CHECK_GT(lwd, 0);
    PT_CHECK_LT(lwd, 20);

    for (size_t i = 1; i < polyline.size(); i++) {
        cv::line(*image, cv::Point2f(polyline[i - 1].x * ratiow, polyline[i - 1].y * ratioh), cv::Point2f(polyline[i].x * ratiow, polyline[i].y * ratioh), color, lwd);
        //cv::line(*image, polyline[i - 1], polyline[i], color, lwd);
    }
}

void SaveDetectionLogToTrajFile(const std::string& path,
                                const DetectionLog& log) {
    std::ofstream file(path.c_str());
    PT_CHECK(file.is_open());
    SaveDetectionLogToStream(file, log);
}

void PrintDetectionLog(const DetectionLog& log) {
    SaveDetectionLogToStream(std::cout, log);
}
