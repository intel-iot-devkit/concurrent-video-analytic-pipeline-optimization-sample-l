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

#include "mot_distance.hpp"

#include <vector>

CosDistance::CosDistance(const cv::Size &descriptor_size)
    : descriptor_size_(descriptor_size) {
    PT_CHECK(descriptor_size.area() != 0);
}

float CosDistance::Compute(const cv::Mat &descr1, const cv::Mat &descr2) {
    PT_CHECK(!descr1.empty());
    PT_CHECK(!descr2.empty());
    PT_CHECK(descr1.size() == descriptor_size_);
    PT_CHECK(descr2.size() == descriptor_size_);

    double xy = descr1.dot(descr2);
    double xx = descr1.dot(descr1);
    double yy = descr2.dot(descr2);
    double norm = sqrt(xx * yy) + 1e-6;
    return 0.5f * static_cast<float>(1.0 - xy / norm);
}

std::vector<float> CosDistance::Compute(const std::vector<cv::Mat> &descrs1,
                                        const std::vector<cv::Mat> &descrs2) {
    PT_CHECK(descrs1.size() != 0);
    PT_CHECK(descrs1.size() == descrs2.size());

    std::vector<float> distances(descrs1.size(), 1.f);
    for (size_t i = 0; i < descrs1.size(); i++) {
        distances.at(i) = Compute(descrs1.at(i), descrs2.at(i));
    }

    return distances;
}


float MatchTemplateDistance::Compute(const cv::Mat &descr1,
                                     const cv::Mat &descr2) {
    PT_CHECK(!descr1.empty() && !descr2.empty());
    PT_CHECK_EQ(descr1.size(), descr2.size());
    PT_CHECK_EQ(descr1.type(), descr2.type());
    cv::Mat res;
    cv::matchTemplate(descr1, descr2, res, type_);
    PT_CHECK(res.size() == cv::Size(1, 1));
    float dist = res.at<float>(0, 0);
    return scale_ * dist + offset_;
}

std::vector<float> MatchTemplateDistance::Compute(const std::vector<cv::Mat> &descrs1,
                                                  const std::vector<cv::Mat> &descrs2) {
    std::vector<float> result;
    for (size_t i = 0; i < descrs1.size(); i++) {
        result.push_back(Compute(descrs1[i], descrs2[i]));
    }
    return result;
}
