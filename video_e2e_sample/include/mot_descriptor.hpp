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

#include <vector>
#include <string>
#include <memory>
#include <inference_engine.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"

#include "network_factory.hpp"
#include "mot_utils.hpp"
#include "openvino/openvino.hpp"
#include <opencv2/opencv.hpp>

///
/// \brief The IImageDescriptor class declares base class for image
/// descriptor.
///
class IImageDescriptor {
public:
    ///
    /// \brief Descriptor size getter.
    /// \return Descriptor size.
    ///
    virtual cv::Size size() const = 0;

    ///
    /// \brief Computes image descriptor.
    /// \param[in] mat Color image.
    /// \param[out] descr Computed descriptor.
    ///
    virtual void Compute(const cv::Mat &mat, cv::Mat *descr) = 0;

    ///
    /// \brief Computes image descriptors in batches.
    /// \param[in] mats Images of interest.
    /// \param[out] descrs Matrices to store the computed descriptors.
    ///
    virtual void Compute(const std::vector<cv::Mat> &mats,
                         std::vector<cv::Mat> *descrs) = 0;

    ///
    /// \brief Prints performance counts for CNN-based descriptors
    ///
    virtual void PrintPerformanceCounts(std::string full_device_name) const {}

    virtual ~IImageDescriptor() {}
};


///
/// \brief Uses resized image as descriptor.
///
class ResizedImageDescriptor : public IImageDescriptor {
public:
    ///
    /// \brief Constructor.
    /// \param[in] descr_size Size of the descriptor (resized image).
    /// \param[in] interpolation Interpolation algorithm.
    ///
    explicit ResizedImageDescriptor(const cv::Size &descr_size,
                                    const cv::InterpolationFlags interpolation)
        : descr_size_(descr_size), interpolation_(interpolation) {
            PT_CHECK_GT(descr_size.width, 0);
            PT_CHECK_GT(descr_size.height, 0);
        }

    ///
    /// \brief Returns descriptor size.
    /// \return Number of elements in the descriptor.
    ///
    cv::Size size() const override { return descr_size_; }

    ///
    /// \brief Computes image descriptor.
    /// \param[in] mat Frame containing the image of interest.
    /// \param[out] descr Matrix to store the computed descriptor.
    ///
    void Compute(const cv::Mat &mat, cv::Mat *descr) override {
        PT_CHECK(descr != nullptr);
        PT_CHECK(!mat.empty());
        cv::resize(mat, *descr, descr_size_, 0, 0, interpolation_);
    }

    ///
    /// \brief Computes images descriptors.
    /// \param[in] mats Frames containing images of interest.
    /// \param[out] descrs Matrices to store the computed descriptors.
    //
    void Compute(const std::vector<cv::Mat> &mats,
                 std::vector<cv::Mat> *descrs) override  {
        PT_CHECK(descrs != nullptr);
        descrs->resize(mats.size());
        for (size_t i = 0; i < mats.size(); i++)  {
            Compute(mats[i], &(descrs[i]));
        }
    }

private:
    cv::Size descr_size_;

    cv::InterpolationFlags interpolation_;
};


class MotIeDescriptor : public IImageDescriptor {
private:
    int result_size_;

    NetworkInfo* network_info_;


    /** @brief Inference Engine device */
    std::string deviceName_;
    /** @brief Pointer to the pre-allocated input blob */
    mutable InferenceEngine::Blob::Ptr pre_alloc_input_blob_;

    std::shared_ptr<ov::InferRequest>  infer_request_;
    std::string input_name_;
    std::string output_name_;
    int max_detections_count_;
    int object_size_;

    /**
     * @brief Run network
     *
     * @param frame Input image
     * @param results_fetcher Callback to fetch inference results
     */
    void Infer(const cv::Mat& frame,
        const std::function<void(const InferenceEngine::BlobMap&, size_t)>& results_fetcher) const;

    /**
     * @brief Run network in batch mode
     *
     * @param frames Vector of input images
     * @param results_fetcher Callback to fetch inference results
     */
    void InferBatch(const std::vector<cv::Mat>& frames,
        const std::function<void(const InferenceEngine::BlobMap&, size_t)>& results_fetcher) const;



public:
    explicit MotIeDescriptor(const std::string& model_path,
        const std::string& target_device_name = "GPU");
    ~MotIeDescriptor();
    ///
    /// \brief Descriptor size getter.
    /// \return Descriptor size.
    ///
    cv::Size size() const override {
        return cv::Size(1, result_size_);
    }

    void Compute(const cv::Mat& mat, cv::Mat* descr) override;

    void Compute(const std::vector<cv::Mat>& mats,
        std::vector<cv::Mat>* descrs) override;



};

