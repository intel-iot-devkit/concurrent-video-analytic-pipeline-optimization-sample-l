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

#pragma once
#include "mot_core.hpp"
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
//#include <samples/ocv_common.hpp>
#include "network_factory.hpp"
#include "openvino/openvino.hpp"
#include <opencv2/opencv.hpp>

class MotObjectDetector {
public:
    explicit MotObjectDetector(bool enable_performance_report = false);
    int Init(const std::string& det_model_path,
        const std::string& target_device_name = "GPU",
        bool remote_blob = false, VADisplay va_dpy = NULL);
    void SubmitFrame(const cv::Mat& frame, int frame_idx);
    void WaitAndFetchResults();
    void GetInputSize(int& width, int& height);
    const TrackedObjects& GetResults() const;
    void PrintPerformanceCounts(std::string fullDeviceName);
    ~MotObjectDetector();

private:
    void enqueue(const cv::Mat& frame);
    void submitRequest();
    void fetchResults();

    NetworkInfo* network_info_;
    std::string input_name_;
    std::string im_info_name_;

    std::shared_ptr<ov::InferRequest>  infer_request_;

    std::string output_name_;
    int max_detections_count_;
    int object_size_;
    int enqueued_frames_ = 0;
    float frame_width_ = 0;
    float frame_height_ = 0;
    bool results_fetched_ = false;
    int frame_idx_ = -1;
    std::string target_device_;
    bool enable_performance_report_;

    TrackedObjects results_;

    float confidence_threshold_{ 0.5f };
    float increase_scale_x_{ 1.f };
    float increase_scale_y_{ 1.f };
    //	bool is_async_ = false;
};
