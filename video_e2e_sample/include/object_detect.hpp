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

#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"

#include <string>
#include <vector>
#include <queue>

#include "openvino/openvino.hpp"
#include <inference_engine.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <gpu/gpu_context_api_va.hpp>
#include <ngraph/ngraph.hpp>

#include "sample_utils.h"
#include "network_factory.hpp"

//#define USE_MOBILENET_SSD

#define INFER_FD_INPUT_W 300
#define INFER_FD_INPUT_H 300
struct ObjectDetectResult {
    int label;
    float confidence;
    cv::Rect location;
    std::string color;
    std::string type;
};

struct DetectionObject {
    int xmin, ymin, xmax, ymax, class_id;
    float confidence;

    DetectionObject(double x, double y, double h, double w, int class_id, float confidence, float h_scale, float w_scale) {
        this->xmin = static_cast<int>((x - w / 2) * w_scale);
        this->ymin = static_cast<int>((y - h / 2) * h_scale);
        this->xmax = static_cast<int>(this->xmin + w * w_scale);
        this->ymax = static_cast<int>(this->ymin + h * h_scale);
        this->class_id = class_id;
        this->confidence = confidence;
    }

    bool operator <(const DetectionObject &s2) const {
        return this->confidence < s2.confidence;
    }
    bool operator >(const DetectionObject &s2) const {
        return this->confidence > s2.confidence;
    }
};

class YoloParams {
    template <typename T>
    void computeAnchors(const std::vector<T> & mask) {
        std::vector<float> maskedAnchors(num * 2);
        for (int i = 0; i < num; ++i) {
            maskedAnchors[i * 2] = anchors[mask[i] * 2];
            maskedAnchors[i * 2 + 1] = anchors[mask[i] * 2 + 1];
        }
        anchors = maskedAnchors;
    }

public:
    int num = 0, classes = 0, coords = 0;
    std::vector<float> anchors = {10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0,
                                  156.0, 198.0, 373.0, 326.0};

    YoloParams() {}

    YoloParams(const std::shared_ptr<ngraph::op::RegionYolo> regionYolo) {
        coords = regionYolo->get_num_coords();
        classes = regionYolo->get_num_classes();
        anchors = regionYolo->get_anchors();
        auto mask = regionYolo->get_mask();
        num = mask.size();

        computeAnchors(mask);
    }

    YoloParams(size_t classes,
            int coords,
            const std::vector<float>& anchors,
            const std::vector<int64_t>& masks,
            size_t outputWidth,
            size_t outputHeight) : classes(classes),

    coords(coords) {
        num = masks.size();

        if (num) {
            this->anchors.resize(num * 2);

            for (int i = 0; i < num; ++i) {
                this->anchors[i * 2] = anchors[masks[i] * 2];
                this->anchors[i * 2 + 1] = anchors[masks[i] * 2 + 1];
            }
        } else {
            this->anchors = anchors;
            num = anchors.size() / 2;
        }
    }
};


class ObjectDetect {
public:

    ObjectDetect(bool mEnablePerformanceReport = false);
    int Init(const std::string& detectorModelPath,
            const int inferType,
            const std::string& targetDeviceName = "GPU",
            bool remoteBlob = false, VADisplay vaDpy = NULL);
    void Detect(const cv::Mat& image, std::vector<DetectionObject>& results);
    void Detect(const cv::Mat& image, std::vector<ObjectDetectResult>& results);
    void Detect(VASurfaceID surface, std::vector<ObjectDetectResult>& results);
    void SetSrcImageSize(int width, int height);
    void GetInputSize(int &width, int &height);
    void RenderResults(std::vector<ObjectDetectResult>& results, cv::Mat& image, bool isGrey = false);
    void RenderResults(std::vector<DetectionObject>& results, cv::Mat& image);

    int Detect(mfxFrameData *pData, mfxFrameData *pData_dec, std::vector<ObjectDetectResult>& results);
    ~ObjectDetect();

private:
    //Nonassinable
    ObjectDetect(ObjectDetect const&);
    ObjectDetect& operator=(ObjectDetect const&);

    float mDetectThreshold;
    bool mRemoteBlob;
    std::string mInputName;
    VADisplay mVaDpy;

    int preprocessYolo(ov::CompiledModel &model);
    void CopyDetectResults(const float *detections, std::vector<ObjectDetectResult>& results);
    int CopyImageData(unsigned char *dst, char unsigned *src, unsigned int width, unsigned int height, unsigned int pitch);

    void CopyDetectResults(std::vector<DetectionObject>& results);
    NetworkInfo *mNetworkInfo;

    InferenceEngine::InferRequest mDetectorRequest;
    std::shared_ptr<ov::InferRequest>  mDetectorRequest1;
    std::shared_ptr<ov::CompiledModel> mCompiled_model;
    ov::CompiledModel  mCompiled_model1;
    cv::VideoWriter writer;
    std::queue<ov::InferRequest> mReqQueue;
    std::string mInput_tensor_name;
    std::string mOutput_tensor_name;

    int mDetectorMaxProposalCount;
    int mDetectorObjectSize;
    std::string mDetectorRoiBlobName;
    std::string mDetectorOutputName;
    bool mEnablePerformanceReport;
    cv::Size mSrcImageSize;

    unsigned int mInputW;
    unsigned int mInputH;
    unsigned char *mImgData;

    std::map<std::string, YoloParams> yoloParams;
    std::vector<std::string> labels;

    int mInferType;
};

