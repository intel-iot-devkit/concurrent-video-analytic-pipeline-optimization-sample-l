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

#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"

#include <algorithm>
#include <string>
#include <vector>
#include <mutex>

#include <opencv2/imgproc/imgproc.hpp>
//#include <samples/ocv_common.hpp>
#include <samples/common.hpp>

#include "human_pose_estimator.hpp"
#include <gpu/gpu_context_api_va.hpp>
#include <gpu/gpu_context_api_ocl.hpp>
#include  <cldnn/cldnn_config.hpp>
#include "peak.hpp"
#include "openvino/openvino.hpp"
#include <opencv2/opencv.hpp>

using namespace InferenceEngine;

namespace human_pose_estimation {
const size_t HumanPoseEstimator::keypointsNumber = 18;

HumanPoseEstimator::HumanPoseEstimator(const std::string& modelpath,
                                       const std::string& targetDev,
                                       bool enablePerformanceReport)
    : minJointsNumber(3),
      stride(8),
      pad(cv::Vec4i::all(0)),
      meanPixel(cv::Vec3f::all(128)),
      minPeaksDistance(3.0f),
      midPointsScoreThreshold(0.05f),
      foundMidPointsRatioThreshold(0.8f),
      minSubsetScore(0.2f),
      inputLayerSize(-1, -1),
      upsampleRatio(4),
      enablePerformanceReport(enablePerformanceReport),
      targetDeviceName(targetDev),
      modelPath(modelpath),
      mRemoteBlob(false),
      mNetworkInfo(nullptr),
      mRenderW(456),
      mRenderH(256)
{
}

int HumanPoseEstimator::Init(bool remoteBlob, VADisplay va_dpy, int renderW, int renderH)
{
    static std::mutex initLock;
    std::lock_guard<std::mutex> lock(initLock);

    mRemoteBlob = remoteBlob;
    if (remoteBlob && va_dpy == nullptr)
    {
        return -1;
    }

    if (renderW < 0 || renderW > 8192 || renderH < 0 || renderH > 8192)
    {
        std::cout<<"The size of rendering image is wrong "<<renderW<<"x"<<renderH<<std::endl;
        return -1;
    }

    mRenderW = renderW;
    mRenderH = renderH;

    NetworkOptions opt;
    if (remoteBlob)
    {
        mRemoteBlob = remoteBlob;
        opt.enableRemoteBlob = remoteBlob;
        opt.vaDpy = va_dpy;
    }

    mNetworkInfo = NetworkFactory::GetNetwork(modelPath, targetDeviceName, opt);

    if (!mNetworkInfo)
    {
        std::cout<<"NetworkFactory::GetNetwork("<<modelPath<<","<<targetDeviceName<<") failed!"<<std::endl;
        return -1;
    }

    inputLayerSize = cv::Size(456, 256);
    mRequest1 =  mNetworkInfo->CreateNewInferRequest2();
    mInputName = mNetworkInfo->input_tensor_name;
    return 0;
}


std::vector<HumanPose> HumanPoseEstimator::estimate(VASurfaceID va_surface) {
    std::vector<HumanPose> poses;
    if (!mNetworkInfo)
    {
        return poses;
    }

    //va_surface = alloc_va_surface(mVaDpy,  mInputW, mInputH);

    auto nv12_blob = mNetworkInfo->mVAContext->create_tensor_nv12(256, 456, va_surface);
    auto infer_request = mRequest1;// mNetworkInfo->mCompiled_model.create_infer_request();

    auto model = mNetworkInfo->mModel;
    auto input0 = model->get_parameters().at(0);
    auto input1 = model->get_parameters().at(1);

    auto shape = input0->get_shape();
    auto width = shape[1];
    auto height = shape[2];

    mRequest1->set_tensor(input0->get_friendly_name(), nv12_blob.first);
    mRequest1->set_tensor(input1->get_friendly_name(), nv12_blob.second);
    mRequest1->infer();


    ov::Tensor pafs = mRequest1->get_tensor("Mconv7_stage2_L1");
    ov::Tensor heatmap = mRequest1->get_tensor("Mconv7_stage2_L2");

    cv::Size imageSize(456, 256);
    poses  = postprocess(
            heatmap.data<float>(),
            32 * 57,
            keypointsNumber,
            pafs.data<float>(),
            32 * 57,
            38,
            57, 32, imageSize);

    return poses;
}

std::vector<HumanPose> HumanPoseEstimator::estimate(const cv::Mat& image) {
    CV_Assert(image.type() == CV_8UC3);

    cv::Size imageSize = image.size();
    //The input size isn't allowed to change
    ov::Tensor input_tensor = mRequest1->get_tensor(mInputName);

    static const ov::Layout layout{"NHWC"};
    const ov::Shape& shape = input_tensor.get_shape();
    cv::Size size{int(shape[ov::layout::width_idx(layout)]), int(shape[ov::layout::height_idx(layout)])};

    cv::resize(image, cv::Mat{size, CV_8UC3, input_tensor.data()}, size);

    mRequest1->infer();

    ov::Tensor pafs = mRequest1->get_tensor("Mconv7_stage2_L1");
    ov::Tensor heatmap = mRequest1->get_tensor("Mconv7_stage2_L2");

    std::vector<HumanPose> poses  = postprocess(
            heatmap.data<float>(),
            32 * 57,
            keypointsNumber,
            pafs.data<float>(),
            32 * 57,
            38,
            57, 32, imageSize);
    return poses;
}

void HumanPoseEstimator::preprocess(const cv::Mat& image, uint8_t* buffer) const {
    cv::Mat resizedImage;
    double scale = inputLayerSize.height / static_cast<double>(image.rows);
    cv::resize(image, resizedImage, cv::Size(), scale, scale, cv::INTER_CUBIC);
    cv::Mat paddedImage;
    cv::copyMakeBorder(resizedImage, paddedImage, pad(0), pad(2), pad(1), pad(3),
                       cv::BORDER_CONSTANT, meanPixel);
    std::vector<cv::Mat> planes(3);
    for (size_t pId = 0; pId < planes.size(); pId++) {
        planes[pId] = cv::Mat(inputLayerSize, CV_8UC1, buffer + pId * inputLayerSize.area());
    }
    cv::split(paddedImage, planes);
}

std::vector<HumanPose> HumanPoseEstimator::postprocess(
        const float* heatMapsData, const int heatMapOffset, const int nHeatMaps,
        const float* pafsData, const int pafOffset, const int nPafs,
        const int featureMapWidth, const int featureMapHeight,
        const cv::Size& imageSize) const {
    std::vector<cv::Mat> heatMaps(nHeatMaps);
    for (size_t i = 0; i < heatMaps.size(); i++) {
        heatMaps[i] = cv::Mat(featureMapHeight, featureMapWidth, CV_32FC1,
                              reinterpret_cast<void*>(
                                  const_cast<float*>(
                                      heatMapsData + i * heatMapOffset)));
    }
    resizeFeatureMaps(heatMaps);

    std::vector<cv::Mat> pafs(nPafs);
    for (size_t i = 0; i < pafs.size(); i++) {
        pafs[i] = cv::Mat(featureMapHeight, featureMapWidth, CV_32FC1,
                          reinterpret_cast<void*>(
                              const_cast<float*>(
                                  pafsData + i * pafOffset)));
    }
    resizeFeatureMaps(pafs);

    std::vector<HumanPose> poses = extractPoses(heatMaps, pafs);
    correctCoordinates(poses, heatMaps[0].size(), imageSize);
    return poses;
}

class FindPeaksBody: public cv::ParallelLoopBody {
public:
    FindPeaksBody(const std::vector<cv::Mat>& heatMaps, float minPeaksDistance,
                  std::vector<std::vector<Peak> >& peaksFromHeatMap)
        : heatMaps(heatMaps),
          minPeaksDistance(minPeaksDistance),
          peaksFromHeatMap(peaksFromHeatMap) {}

    virtual void operator()(const cv::Range& range) const {
        for (int i = range.start; i < range.end; i++) {
            findPeaks(heatMaps, minPeaksDistance, peaksFromHeatMap, i);
        }
    }

private:
    const std::vector<cv::Mat>& heatMaps;
    float minPeaksDistance;
    std::vector<std::vector<Peak> >& peaksFromHeatMap;
};

std::vector<HumanPose> HumanPoseEstimator::extractPoses(
        const std::vector<cv::Mat>& heatMaps,
        const std::vector<cv::Mat>& pafs) const {
    std::vector<std::vector<Peak> > peaksFromHeatMap(heatMaps.size());
    FindPeaksBody findPeaksBody(heatMaps, minPeaksDistance, peaksFromHeatMap);
    cv::parallel_for_(cv::Range(0, static_cast<int>(heatMaps.size())),
                      findPeaksBody);
    int peaksBefore = 0;
    for (size_t heatmapId = 1; heatmapId < heatMaps.size(); heatmapId++) {
        peaksBefore += static_cast<int>(peaksFromHeatMap[heatmapId - 1].size());
        for (auto& peak : peaksFromHeatMap[heatmapId]) {
            peak.id += peaksBefore;
        }
    }
    std::vector<HumanPose> poses = groupPeaksToPoses(
                peaksFromHeatMap, pafs, keypointsNumber, midPointsScoreThreshold,
                foundMidPointsRatioThreshold, minJointsNumber, minSubsetScore);
    return poses;
}

void HumanPoseEstimator::resizeFeatureMaps(std::vector<cv::Mat>& featureMaps) const {
    for (auto& featureMap : featureMaps) {
        cv::resize(featureMap, featureMap, cv::Size(),
                   upsampleRatio, upsampleRatio, cv::INTER_CUBIC);
    }
}

void HumanPoseEstimator::correctCoordinates(std::vector<HumanPose>& poses,
                                            const cv::Size& featureMapsSize,
                                            const cv::Size& imageSize) const {
    CV_Assert(stride % upsampleRatio == 0);

    cv::Size fullFeatureMapSize = featureMapsSize * stride / upsampleRatio;

    float scaleX = mRenderW /
            static_cast<float>(fullFeatureMapSize.width - pad(1) - pad(3));
    float scaleY = mRenderH /
            static_cast<float>(fullFeatureMapSize.height - pad(0) - pad(2));

    for (auto& pose : poses) {
        for (auto& keypoint : pose.keypoints) {
            if (keypoint != cv::Point2f(-1, -1)) {
                keypoint.x *= stride / upsampleRatio;
                keypoint.x -= pad(1);
                keypoint.x *= scaleX;

                keypoint.y *= stride / upsampleRatio;
                keypoint.y -= pad(0);
                keypoint.y *= scaleY;
            }
        }
    }
}

bool HumanPoseEstimator::inputWidthIsChanged(const cv::Size& imageSize) {
    double scale = static_cast<double>(inputLayerSize.height) / static_cast<double>(imageSize.height);
    cv::Size scaledSize(static_cast<int>(cvRound(imageSize.width * scale)),
                        static_cast<int>(cvRound(imageSize.height * scale)));
    cv::Size scaledImageSize(std::max(scaledSize.width, inputLayerSize.height),
                             inputLayerSize.height);
    int minHeight = std::min(scaledImageSize.height, scaledSize.height);
    scaledImageSize.width = static_cast<int>(std::ceil(
                scaledImageSize.width / static_cast<float>(stride))) * stride;
    pad(0) = static_cast<int>(std::floor((scaledImageSize.height - minHeight) / 2.0));
    pad(1) = static_cast<int>(std::floor((scaledImageSize.width - scaledSize.width) / 2.0));
    pad(2) = scaledImageSize.height - minHeight - pad(0);
    pad(3) = scaledImageSize.width - scaledSize.width - pad(1);
    if (scaledSize.width == (inputLayerSize.width - pad(1) - pad(3))) {
        return false;
    }

    inputLayerSize.width = scaledImageSize.width;
    return true;
}

HumanPoseEstimator::~HumanPoseEstimator() {
    NetworkFactory::PutNetwork(mNetworkInfo);
#if 0
    if (enablePerformanceReport) {
        std::cout << "Performance counts for " << modelPath << std::endl << std::endl;
        printPerformanceCounts(request.GetPerformanceCounts(), std::cout, false);
    }
#endif
}
}  // namespace human_pose_estimation
