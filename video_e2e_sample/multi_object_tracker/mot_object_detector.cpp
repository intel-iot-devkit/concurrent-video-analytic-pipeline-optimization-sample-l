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

#include "mot_object_detector.hpp"

#include <algorithm>
#include <inference_engine.hpp>
#include <ngraph/ngraph.hpp>

using namespace InferenceEngine;

#define SSD_EMPTY_DETECTIONS_INDICATOR -1.0

namespace {
    cv::Rect TruncateToValidRect(const cv::Rect& rect,
                                 const cv::Size& size) {
        auto tl = rect.tl(), br = rect.br();
        tl.x = std::max(0, std::min(size.width - 1, tl.x));
        tl.y = std::max(0, std::min(size.height - 1, tl.y));
        br.x = std::max(0, std::min(size.width, br.x));
        br.y = std::max(0, std::min(size.height, br.y));
        int w = std::max(0, br.x - tl.x);
        int h = std::max(0, br.y - tl.y);
        return cv::Rect(tl.x, tl.y, w, h);
    }

    cv::Rect IncreaseRect(const cv::Rect& r, float coeff_x,
                          float coeff_y)  {
        cv::Point2f tl = r.tl();
        cv::Point2f br = r.br();
        cv::Point2f c = (tl * 0.5f) + (br * 0.5f);
        cv::Point2f diff = c - tl;
        cv::Point2f new_diff{diff.x * coeff_x, diff.y * coeff_y};
        cv::Point2f new_tl = c - new_diff;
        cv::Point2f new_br = c + new_diff;

        cv::Point new_tl_int {static_cast<int>(std::floor(new_tl.x)), static_cast<int>(std::floor(new_tl.y))};
        cv::Point new_br_int {static_cast<int>(std::ceil(new_br.x)), static_cast<int>(std::ceil(new_br.y))};

        return cv::Rect(new_tl_int, new_br_int);
    }
}  // namespace

void MotObjectDetector::submitRequest() {
    if (!infer_request_) return;
    if (!enqueued_frames_) return;
    enqueued_frames_ = 0;
    results_fetched_ = false;
    results_.clear();

    //if (is_async_) {
    //    infer_request_.StartAsync();
    //} else {
        infer_request_.Infer();
//    }
}

const TrackedObjects& MotObjectDetector::GetResults() const {
    return results_;
}

void MotObjectDetector::enqueue(const cv::Mat &frame) {
    frame_width_ = static_cast<float>(frame.cols);
    frame_height_ = static_cast<float>(frame.rows);

    //std::cout << "mFrameWidth:" << frame_width_ << " mFrameHeight:" << frame_height_ << std::endl;

    Blob::Ptr input_blob = infer_request_.GetBlob(input_name_);

    matU8ToBlob<uint8_t>(frame, input_blob);

    if (!im_info_name_.empty()) {
        LockedMemory<void> im_info_mapped = as<MemoryBlob>(infer_request_.GetBlob(im_info_name_))->wmap();
        float* buffer = im_info_mapped.as<float*>();
        buffer[0] = static_cast<float>(input_blob->getTensorDesc().getDims()[2]);
        buffer[1] = static_cast<float>(input_blob->getTensorDesc().getDims()[3]);
        buffer[2] = buffer[4] = static_cast<float>(input_blob->getTensorDesc().getDims()[3]) / frame_width_;
        buffer[3] = buffer[5] = static_cast<float>(input_blob->getTensorDesc().getDims()[2]) / frame_height_;
    }

    enqueued_frames_ = 1;
}

void MotObjectDetector::SubmitFrame(const cv::Mat &frame, int frame_idx) {
    frame_idx_ = frame_idx;
    enqueue(frame);
    submitRequest();
}

int MotObjectDetector::Init(const std::string& modelPath, const std::string& targetDeviceName,
    bool remoteBlob, VADisplay vaDpy) {
    static std::mutex initLock;
   std::lock_guard<std::mutex> lock(initLock);

   NetworkOptions networkOpt;
   if (remoteBlob)
   {
       networkOpt.enableRemoteBlob = remoteBlob;
       networkOpt.vaDpy = vaDpy;
   }

   network_info_ = NetworkFactory::GetNetwork(modelPath, targetDeviceName, networkOpt);
   if (!network_info_)
   {
       std::cout << "NetworkFactory::GetNetwork(" << modelPath << "," << targetDeviceName << ") failed!" << std::endl;
       return -1;
   }
   std::cout << "Loading network " << modelPath << " on device " << targetDeviceName << " is done." << std::endl;

   InferenceEngine::CNNNetwork& det_network = network_info_->mNetwork;

   infer_request_ = network_info_->CreateNewInferRequest();

    InputsDataMap inputInfo(det_network.getInputsInfo());
    if (1 == inputInfo.size() || 2 == inputInfo.size()) {
        for (const std::pair<std::string, InputInfo::Ptr>& input : inputInfo) {
            InputInfo::Ptr input_info = input.second;
            InferenceEngine::SizeVector blob_size = input_info->getTensorDesc().getDims();
            if (4 == blob_size.size()) {
               input_name_ = input.first;
            }
            else if (SizeVector{ 1, 6 } == blob_size) {
                im_info_name_ = input.first;
            }
            else {
                std::cout << "Unknown input for Person Detection network" <<std::endl;
                return -1;
            }
            //mInputW = blobSize[3];
            //mInputH = blobSize[2];
        }
        if (input_name_.empty()) {
            std::cout << "No image input for Person Detection network found" <<std::endl;
            return -1;
        }
    }
    else {
        std::cout << "Person Detection network should have one or two inputs" <<std::endl;
        return -1;
    }

    OutputsDataMap outputInfo(det_network.getOutputsInfo());
    if (outputInfo.size() != 1) {
        std::cout << "Person Detection network should have only one output" <<std::endl;
        return -1;
    }
    DataPtr& output = outputInfo.begin()->second;
    output_name_ = outputInfo.begin()->first;

    const SizeVector output_dims = output->getTensorDesc().getDims();
    if (output_dims.size() != 4) {
        std::cout << "Person Detection network output dimensions not compatible shoulld be 4, but was " +
            std::to_string(output_dims.size()) <<std::endl;
        return -1;
    }
    max_detections_count_ = output_dims[2];
    object_size_ = output_dims[3];

    if (object_size_ != 7) {
        std::cout << "Person Detection network output layer should have 7 as a last dimension" <<std::endl;
        return -1;
    }
    return 0;
}

MotObjectDetector::MotObjectDetector(bool enablePerformanceReport)
    :frame_idx_(0),
    enable_performance_report_(enablePerformanceReport)
{
}

MotObjectDetector::~MotObjectDetector() {
    if (network_info_) {
        NetworkFactory::PutNetwork(network_info_);
    }
}

void MotObjectDetector::wait() {
    if (!infer_request_) return;
    infer_request_.Wait(InferenceEngine::IInferRequest::WaitMode::RESULT_READY);
}

void MotObjectDetector::fetchResults() {
    results_.clear();
    if (results_fetched_) return;
    results_fetched_ = true;
    LockedMemory<const void> output_mapped = as<MemoryBlob>(infer_request_.GetBlob(output_name_))->rmap();
    const float *data = output_mapped.as<float *>();

    for (int detId = 0; detId < max_detections_count_; ++detId) {
        const int start_pos = detId * object_size_;

        const float batch_id = data[start_pos];
        if (start_pos == SSD_EMPTY_DETECTIONS_INDICATOR) {
            break;
        }

        const float score = std::min(std::max(0.0f, data[start_pos + 2]), 1.0f);
        const float x0 =
            std::min(std::max(0.0f, data[start_pos + 3]), 1.0f) * frame_width_;
        const float y0 =
            std::min(std::max(0.0f, data[start_pos + 4]), 1.0f) * frame_height_;
        const float x1 =
            std::min(std::max(0.0f, data[start_pos + 5]), 1.0f) * frame_width_;
        const float y1 =
            std::min(std::max(0.0f, data[start_pos + 6]), 1.0f) * frame_height_;

        TrackedObject object;
        object.confidence = score;
        object.rect = cv::Rect(cv::Point(static_cast<int>(round(static_cast<double>(x0))),
                                         static_cast<int>(round(static_cast<double>(y0)))),
                               cv::Point(static_cast<int>(round(static_cast<double>(x1))),
                                         static_cast<int>(round(static_cast<double>(y1)))));

        object.rect = TruncateToValidRect(IncreaseRect(object.rect,  increase_scale_x_, increase_scale_y_),
                                          cv::Size(static_cast<int>(frame_width_), static_cast<int>(frame_height_)));
        object.frame_idx = frame_idx_;

        if (object.confidence > confidence_threshold_ && object.rect.area() > 0) {
            results_.emplace_back(object);
        }
    }
}

void MotObjectDetector::GetInputSize(int& width, int& height) {
    width = frame_width_;
    height = frame_height_;
}

void MotObjectDetector::WaitAndFetchResults() {
    wait();
    fetchResults();
}

void MotObjectDetector::PrintPerformanceCounts(std::string fullDeviceName) {
    std::cout << "Performance counts for object detector" << std::endl << std::endl;
    ::printPerformanceCounts(infer_request_, std::cout, fullDeviceName, false);
}