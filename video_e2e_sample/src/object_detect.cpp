/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#include <algorithm>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>

#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/imgproc/imgproc.hpp>
#include <samples/ocv_common.hpp>
#include <samples/common.hpp>

#include "object_detect.hpp"

using namespace InferenceEngine;
using namespace cv;

ObjectDetect::ObjectDetect(bool enablePerformanceReport)
    :mDetectThreshold(0.5f),
    mEnablePerformanceReport(enablePerformanceReport)
{
    return;
}

void ObjectDetect::Init(const std::string& detectorModelPath,
        const std::string& targetDeviceName)
{
    static std::mutex initLock;
    std::lock_guard<std::mutex> lock(initLock);
    mPlugin = InferenceEngine::PluginDispatcher()
            .getPluginByDevice(targetDeviceName);
    if (mEnablePerformanceReport)
    {
        mPlugin.SetConfig({{InferenceEngine::PluginConfigParams::KEY_PERF_COUNT,
                           InferenceEngine::PluginConfigParams::YES}});
    }

    //Load Vehicle Detector Network
    mDetectorNetReader.ReadNetwork(detectorModelPath);
    std::string binFileName = fileNameNoExt(detectorModelPath) + ".bin";
    mDetectorNetReader.ReadWeights(binFileName);
    mDetectorNetwork = mDetectorNetReader.getNetwork();
    mDetectorNetwork.setBatchSize(1);
    InferenceEngine::InputInfo::Ptr inputInfo = mDetectorNetwork.getInputsInfo().begin()->second;
    inputInfo->setPrecision(Precision::U8);
    inputInfo->getInputData()->setLayout(Layout::NCHW);

    InferenceEngine::OutputsDataMap outputInfo = mDetectorNetwork.getOutputsInfo();
    auto outputBlobsIt = outputInfo.begin();
    mDetectorRoiBlobName = outputBlobsIt->first;

    DataPtr& output = outputInfo.begin()->second;
    const SizeVector outputDims = output->getTensorDesc().getDims();
    mDetectorOutputName = outputInfo.begin()->first;
    mDetectorMaxProposalCount = outputDims[2];
    mDetectorObjectSize = outputDims[3];
    output->setPrecision(Precision::FP32);
    output->setLayout(Layout::NCHW);

    mExecutableNetwork = mPlugin.LoadNetwork(mDetectorNetwork, {});
    mDetectorRequest = mExecutableNetwork.CreateInferRequest();
}

void ObjectDetect::SetSrcImageSize(int width, int height)
{
    mSrcImageSize.height = height;
    mSrcImageSize.width = width;
}

void ObjectDetect::Detect(const cv::Mat& image, std::vector<ObjectDetectResult>& results)
{
    InferenceEngine::Blob::Ptr input = mDetectorRequest.GetBlob(mDetectorNetwork.getInputsInfo().begin()->first);
    matU8ToBlob<uint8_t>(image, input);
    mDetectorRequest.Infer();

    const float *detections = mDetectorRequest.GetBlob(mDetectorOutputName)->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();
    for (int i = 0; i < mDetectorMaxProposalCount; i++)
    {
        float image_id = detections[i * mDetectorObjectSize + 0];  // in case of batch
        ObjectDetectResult r;
        r.label = static_cast<int>(detections[i * mDetectorObjectSize + 1]);
        r.confidence = detections[i * mDetectorObjectSize + 2];
        if (r.confidence <= mDetectThreshold)
        {
            continue;
        }

        if (image_id < 0)
        {  // indicates end of detections
            break;
        }
        r.location.x = static_cast<int>(detections[i * mDetectorObjectSize + 3] * mSrcImageSize.width);
        r.location.y = static_cast<int>(detections[i * mDetectorObjectSize + 4] * mSrcImageSize.height);
        r.location.width = static_cast<int>(detections[i * mDetectorObjectSize + 5] * mSrcImageSize.width - r.location.x);
        r.location.height = static_cast<int>(detections[i * mDetectorObjectSize + 6] * mSrcImageSize.height - r.location.y);

        /* std::cout << "[" << i << "," << r.label << "] element, prob = " << r.confidence <<
            "    (" << r.location.x << "," << r.location.y << ")-(" << r.location.width << ","
            << r.location.height << ")"
            << (r.confidence  ) << std::endl; */

        results.push_back(r);
    }
    return ;
}


void ObjectDetect::RenderResults(std::vector<ObjectDetectResult>& results, cv::Mat& image)
{
    char confidence[8];
    for (unsigned int i = 0; i < results.size(); i++) {
        cv::Rect &location = results[i].location;
        rectangle(image, results[i].location, Scalar(0, 230, 0), 1, 4);

        /* background color */
        rectangle(image, Point(location.x - 1, location.y - 8), Point(location.x + location.width, location.y - 1), Scalar(0, 230, 0), -1, LINE_8, 0);
        snprintf(confidence, 8, "%.2f", results[i].confidence);

        /* text */
        putText(image, confidence, Point(location.x + 1, location.y - 1), FONT_HERSHEY_TRIPLEX, .3, Scalar(70,70,70));
    }
    return;
}

ObjectDetect::~ObjectDetect()
{
    if (mEnablePerformanceReport)
    {
        std::cout << "Performance counts for object detection:" << std::endl << std::endl;
        printPerformanceCounts(mDetectorRequest.GetPerformanceCounts(), std::cout, "GPU", false);
    }
}
