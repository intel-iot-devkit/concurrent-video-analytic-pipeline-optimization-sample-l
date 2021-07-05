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
    std::cout << "Loading network " << model_path << " on device " << target_device_name << " is done." << std::endl;

    InferenceEngine::CNNNetwork& cnn_network = network_info_->mNetwork;

    InferenceEngine::InputsDataMap inputs_data_map;
    inputs_data_map = cnn_network.getInputsInfo();
    if (inputs_data_map.size() != 1) {
        THROW_IE_EXCEPTION << "Network should have only one input";
    }

    SizeVector input_dims = inputs_data_map.begin()->second->getTensorDesc().getDims();
    pre_alloc_input_blob_ = make_shared_blob<uint8_t>(TensorDesc(Precision::U8, input_dims, Layout::NCHW));
    pre_alloc_input_blob_->allocate();
    BlobMap inputs_blob_map;
    inputs_blob_map[inputs_data_map.begin()->first] = pre_alloc_input_blob_;

    outputs_data_map_ = cnn_network.getOutputsInfo();

    for (auto&& item : outputs_data_map_) {
        SizeVector output_dims = item.second->getTensorDesc().getDims();
        auto output_layout = item.second->getTensorDesc().getLayout();
        item.second->setPrecision(Precision::FP32);
        TBlob<float>::Ptr output =
            make_shared_blob<float>(TensorDesc(Precision::FP32, output_dims, output_layout));
        output->allocate();
        outputs_blob_map_[item.first] = output;
    }


    infer_request_ = network_info_->CreateNewInferRequest();

    infer_request_.SetInput(inputs_blob_map);
    infer_request_.SetOutput(outputs_blob_map_);
    if (outputs_blob_map_.size() != 1) {
        THROW_IE_EXCEPTION << "Demo supports topologies only with 1 output";
    }
    OutputsDataMap::iterator it = outputs_data_map_.begin();
    result_size_ = 0;
    if ((it != outputs_data_map_.end())) {
        InferenceEngine::SizeVector dims = outputs_data_map_.begin()->second->getTensorDesc().getDims();
        result_size_ = std::accumulate(std::next(dims.begin(), 1), dims.end(), 1, std::multiplies<int>());
    }
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

        infer_request_.Infer();

        fetch_results(outputs_blob_map_, current_batch_size);
    }
}

void MotIeDescriptor::Infer(const cv::Mat& frame,
    const std::function<void(const InferenceEngine::BlobMap&, size_t)>& fetch_results) const {
    InferBatch({ frame }, fetch_results);
}