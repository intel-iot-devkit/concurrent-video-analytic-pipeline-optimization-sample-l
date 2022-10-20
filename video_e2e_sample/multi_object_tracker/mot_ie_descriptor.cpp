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

#include "mot_descriptor.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>

#include <samples/ocv_common.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <inference_engine.hpp>

using namespace InferenceEngine;

MotIeDescriptor::MotIeDescriptor(const std::string& model_path, const std::string& target_device_name) {

    NetworkOptions network_opt;
    //if (remote_blob)
    //{
    //    network_opt.enableRemoteBlob = remote_blob;
    //    network_opt.vaDpy = va_dpy;
    //}
    network_info_ = NetworkFactory::GetNetwork(model_path, target_device_name, network_opt);
    if (!network_info_)
    {
        std::cout << "NetworkFactory::GetNetwork(" << model_path << "," << target_device_name << ") failed!" << std::endl;
        THROW_IE_EXCEPTION << "NetworkFactory::GetNetwork(" << model_path << "," << target_device_name << ") failed!";
    }
    infer_request_ = network_info_->CreateNewInferRequest2();
    input_name_ = network_info_->input_tensor_name;
    output_name_ = network_info_->output_tensor_name;
    max_detections_count_ = network_info_->m_max_detections_count;
    object_size_ = network_info_->m_object_size;
    ov::Shape outputdims = network_info_->GetoutputDims();

    result_size_ = outputdims[0]*outputdims[1];

}

MotIeDescriptor::~MotIeDescriptor() {
    if (network_info_) {
        NetworkFactory::PutNetwork(network_info_);
    }

}
void MotIeDescriptor::Compute(const cv::Mat& frame, cv::Mat* vector) {
    std::vector<cv::Mat> output;
    Compute({ frame }, &output);
    *vector = output[0];
}

void MotIeDescriptor::Compute(const std::vector<cv::Mat>& images, std::vector<cv::Mat>* vectors)  {
    if (images.empty()) {
        return;
    }
    vectors->clear();
    cv::Size outp_shape = cv::Size();
    auto results_fetcher = [vectors, outp_shape](const InferenceEngine::BlobMap& outputs, size_t batch_size) {
        for (auto&& item : outputs) {
            InferenceEngine::Blob::Ptr blob = nullptr;
            if ((blob = item.second) != nullptr) {
                InferenceEngine::SizeVector ie_output_dims = blob->getTensorDesc().getDims();
                std::vector<int> blob_sizes(ie_output_dims.size(), 0);
                for (size_t i = 0; i < blob_sizes.size(); ++i) {
                    blob_sizes[i] = ie_output_dims[i];
                }
                LockedMemory<const void> blobMapped = as<MemoryBlob>(blob)->rmap();
                cv::Mat out_blob(blob_sizes, CV_32F, blobMapped.as<float*>());
                for (size_t b = 0; b < batch_size; b++) {
                    cv::Mat blob_wrapper(out_blob.size[1], 1, CV_32F,
                        reinterpret_cast<void*>((out_blob.ptr<float>(0) + b * out_blob.size[1])));
                    vectors->emplace_back();
                    if (outp_shape != cv::Size())
                        blob_wrapper = blob_wrapper.reshape(1, { outp_shape.height, outp_shape.width });
                    blob_wrapper.copyTo(vectors->back());
                }
            } else {
                THROW_IE_EXCEPTION << "MotIeDescriptor::Compute() Invalid blob '" << item.first << "'";
            }
        }
    };
    InferBatch(images, results_fetcher);
}

void MotIeDescriptor::InferBatch(
    const std::vector<cv::Mat>& frames,
    const std::function<void(const InferenceEngine::BlobMap&, size_t)>& fetch_results) const {
    const size_t batch_size = pre_alloc_input_blob_->getTensorDesc().getDims()[0];

    size_t num_imgs = frames.size();
    for (size_t batch_i = 0; batch_i < num_imgs; batch_i += batch_size) {
        const size_t current_batch_size = std::min(batch_size, num_imgs - batch_i);
        for (size_t b = 0; b < current_batch_size; b++) {
            matU8ToBlob<uint8_t>(frames[batch_i + b], pre_alloc_input_blob_, b);
        }

        infer_request_->infer();
    }
}

void MotIeDescriptor::Infer(const cv::Mat& frame,
    const std::function<void(const InferenceEngine::BlobMap&, size_t)>& fetch_results) const {
    InferBatch({ frame }, fetch_results);
}
