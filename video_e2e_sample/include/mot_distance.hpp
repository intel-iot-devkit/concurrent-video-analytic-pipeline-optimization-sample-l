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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <vector>

#include "logging.hpp"

#include <inference_engine.hpp>

///
/// \brief The IDescriptorDistance class declares an interface for distance
/// computation between reidentification descriptors.
///
class IDescriptorDistance {
public:
    ///
    /// \brief Computes distance between two descriptors.
    /// \param[in] descr1 First descriptor.
    /// \param[in] descr2 Second descriptor.
    /// \return Distance between two descriptors.
    ///
    virtual float Compute(const cv::Mat &descr1, const cv::Mat &descr2) = 0;

    ///
    /// \brief Computes distances between two descriptors in batches.
    /// \param[in] descrs1 Batch of first descriptors.
    /// \param[in] descrs2 Batch of second descriptors.
    /// \return Distances between descriptors.
    ///
    virtual std::vector<float> Compute(const std::vector<cv::Mat> &descrs1,
                                       const std::vector<cv::Mat> &descrs2) = 0;

    virtual ~IDescriptorDistance() {}
};

///
/// \brief The CosDistance class allows computing cosine distance between two
/// reidentification descriptors.
///
class CosDistance : public IDescriptorDistance {
public:
    ///
    /// \brief CosDistance constructor.
    /// \param[in] descriptor_size Descriptor size.
    ///
    explicit CosDistance(const cv::Size &descriptor_size);

    ///
    /// \brief Computes distance between two descriptors.
    /// \param descr1 First descriptor.
    /// \param descr2 Second descriptor.
    /// \return Distance between two descriptors.
    ///
    float Compute(const cv::Mat &descr1, const cv::Mat &descr2) override;

    ///
    /// \brief Computes distances between two descriptors in batches.
    /// \param[in] descrs1 Batch of first descriptors.
    /// \param[in] descrs2 Batch of second descriptors.
    /// \return Distances between descriptors.
    ///
    std::vector<float> Compute(
        const std::vector<cv::Mat> &descrs1,
        const std::vector<cv::Mat> &descrs2) override;

private:
    cv::Size descriptor_size_;
};



///
/// \brief Computes distance between images
///        using MatchTemplate function from OpenCV library
///        and its cross-correlation computation method in particular.
///
class MatchTemplateDistance : public IDescriptorDistance {
public:
    ///
    /// \brief Constructs the distance object.
    ///
    /// \param[in] type Method of MatchTemplate function computation.
    /// \param[in] scale Scale parameter for the distance.
    ///            Final distance is computed as:
    ///            scale * distance + offset.
    /// \param[in] offset Offset parameter for the distance.
    ///            Final distance is computed as:
    ///            scale * distance + offset.
    ///
    MatchTemplateDistance(int type = cv::TemplateMatchModes::TM_CCORR_NORMED,
                          float scale = -1, float offset = 1)
        : type_(type), scale_(scale), offset_(offset) {}
    ///
    /// \brief Computes distance between image descriptors.
    /// \param[in] descr1 First image descriptor.
    /// \param[in] descr2 Second image descriptor.
    /// \return Distance between image descriptors.
    ///
    float Compute(const cv::Mat &descr1, const cv::Mat &descr2) override;
    ///
    /// \brief Computes distances between two descriptors in batches.
    /// \param[in] descrs1 Batch of first descriptors.
    /// \param[in] descrs2 Batch of second descriptors.
    /// \return Distances between descriptors.
    ///
    std::vector<float> Compute(const std::vector<cv::Mat> &descrs1,
                               const std::vector<cv::Mat> &descrs2) override;
    virtual ~MatchTemplateDistance() {}

private:
    int type_;      ///< Method of MatchTemplate function computation.
    float scale_;   ///< Scale parameter for the distance. Final distance is
                    /// computed as: scale * distance + offset.
    float offset_;  ///< Offset parameter for the distance. Final distance is
                    /// computed as: scale * distance + offset.
};

