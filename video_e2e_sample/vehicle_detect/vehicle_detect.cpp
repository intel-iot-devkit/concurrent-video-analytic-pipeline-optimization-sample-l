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

#include "vehicle_detect.hpp"

using namespace InferenceEngine;

VehicleDetect::VehicleDetect(bool enablePerformanceReport)
    :mDetectThreshold(0.65f),
    mDetectorMaxProposalCount(0),
    mDetectorObjectSize(0),
    mEnablePerformanceReport(enablePerformanceReport),
    mVDNetworkInfo(nullptr),
    mVANetworkInfo(nullptr)
{
    return;
}

int VehicleDetect::Init(const std::string& detectorModelPath,
        const std::string& vehicleAttribsModelPath,
        const std::string& targetDeviceName)
{
    static std::mutex initLock;
    std::lock_guard<std::mutex> lock(initLock);


    NetworkOptions optVD, optVA;

    //Vehicle detector
    mVDNetworkInfo = NetworkFactory::GetNetwork(detectorModelPath, targetDeviceName, optVD);

    if (!mVDNetworkInfo)
    {
        std::cout<<"NetworkFactory::GetNetwork("<<detectorModelPath<<","<<targetDeviceName<<") failed!"<<std::endl;
        return -1;
    }

    std::cout<<"Loading network "<<detectorModelPath<<" on device "<<targetDeviceName<<" is done."<<std::endl;
    InferenceEngine::CNNNetwork &detectorVDNetwork = mVDNetworkInfo->mNetwork;

    //Vehicle attributes detector
    mVANetworkInfo = NetworkFactory::GetNetwork(vehicleAttribsModelPath, targetDeviceName, optVA);

    if (!mVANetworkInfo)
    {
        std::cout<<"NetworkFactory::GetNetwork("<<detectorModelPath<<","<<targetDeviceName<<") failed!"<<std::endl;
        return -1;
    }

    std::cout<<"Loading network "<<detectorModelPath<<" on device "<<targetDeviceName<<" is done."<<std::endl;
    InferenceEngine::CNNNetwork &detectorVANetwork = mVANetworkInfo->mNetwork;

    auto inputIt = detectorVDNetwork.getInputsInfo().begin();
    if (inputIt == detectorVDNetwork.getInputsInfo().end())
    {
        return -1;   
    }
    InferenceEngine::InputInfo::Ptr inputInfo = inputIt->second;
    mVDDetectorInputName = inputIt->first;
    inputInfo->setPrecision(Precision::U8);

    InferenceEngine::OutputsDataMap outputInfo = detectorVDNetwork.getOutputsInfo();
    auto outputBlobsIt = outputInfo.begin();
    if (outputBlobsIt == outputInfo.end())
    {
        return -1;
    }
    mDetectorRoiBlobName = outputBlobsIt->first;

    DataPtr& output = outputBlobsIt->second;
    const SizeVector outputDims = output->getTensorDesc().getDims();
    mDetectorOutputName = outputBlobsIt->first;
    mDetectorMaxProposalCount = outputDims[2];
    mDetectorObjectSize = outputDims[3];
    output->setPrecision(Precision::FP32);
    output->setLayout(Layout::NCHW);

    mVDDetectorRequest =  mVDNetworkInfo->CreateNewInferRequest();

    auto VAInputP = detectorVANetwork.getInputsInfo().begin();
    if (VAInputP == detectorVANetwork.getInputsInfo().end())
    {
        return -1;
    }
    inputInfo = VAInputP->second;
    inputInfo->setPrecision(Precision::U8);
    inputInfo->getInputData()->setLayout(Layout::NCHW);

    mVADetectorInputName = VAInputP->first;
    outputInfo = detectorVANetwork.getOutputsInfo();
    outputBlobsIt = outputInfo.begin();
    if (outputBlobsIt == outputInfo.end())
    {
        return -1;
    }

    mVAOutputNameForColor = (outputBlobsIt++)->second->getName();  // color is the first output
    if (outputBlobsIt == outputInfo.end())
    {
        return -1;
    }
    mVAOutputNameForType = (outputBlobsIt)->second->getName();  // type is the second output

    mVARequest =  mVANetworkInfo->CreateNewInferRequest();
    return 0;
}

const std::string VehicleDetect::mVAColors[] =
{
    "white", "gray", "yellow", "red", "green", "blue", "black"
};
const std::string VehicleDetect::mVATypes[] =
{
    "car", "bus", "truck", "van"
};

void VehicleDetect::SetSrcImageSize(int width, int height)
{
    mSrcImageSize.height = height;
    mSrcImageSize.width = width;
}

void VehicleDetect::Detect(const cv::Mat& image, std::vector<VehicleDetectResult>& results, int maxObjNum)
{
  
    InferenceEngine::Blob::Ptr input = mVDDetectorRequest.GetBlob(mVDDetectorInputName);
    matU8ToBlob<uint8_t>(image, input);
    mVDDetectorRequest.Infer();

    const float *detections = mVDDetectorRequest.GetBlob(mDetectorOutputName)->buffer().as<float *>();
    if (maxObjNum < 0 || maxObjNum > mDetectorMaxProposalCount)
    {
        maxObjNum = mDetectorMaxProposalCount; 
    }

    for (int i = 0; i < mDetectorMaxProposalCount; i++)
    {
        float image_id = detections[i * mDetectorObjectSize + 0];  // in case of batch
        VehicleDetectResult r;
        r.label = static_cast<int>(detections[i * mDetectorObjectSize + 1]);
        r.confidence = detections[i * mDetectorObjectSize + 2];
        if (r.confidence <= mDetectThreshold || r.label != 1)
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
        if (results.size() >= (unsigned int)maxObjNum)
        {
            break;
        }
    }

    InferenceEngine::Blob::Ptr VAInput = mVARequest.GetBlob(mVADetectorInputName);
    for (unsigned int i = 0; i < results.size(); i++)
    {
        //image's size can be different from source image
        auto clip = results[i].location & cv::Rect(0, 0, mSrcImageSize.width, mSrcImageSize.height);
        clip.x = clip.x * image.cols / mSrcImageSize.width;
        clip.width = clip.width * image.cols / mSrcImageSize.width;
        clip.y = clip.y * image.rows / mSrcImageSize.height;
        clip.height = clip.height * image.rows / mSrcImageSize.height;
        cv::Mat vehicle = image(clip);
        matU8ToBlob<uint8_t>(vehicle, VAInput);
        mVARequest.Infer();

        auto colorsValues = mVARequest.GetBlob(mVAOutputNameForColor)->buffer().as<float*>();
        // 4 possible types for each vehicle and we should select the one with the maximum probability
        auto typesValues  = mVARequest.GetBlob(mVAOutputNameForType)->buffer().as<float*>();

        const auto color_id = std::max_element(colorsValues, colorsValues + 7) - colorsValues;
        const auto type_id =  std::max_element(typesValues,  typesValues  + 4) - typesValues;
        results[i].color = mVAColors[color_id];
        results[i].type = mVATypes[type_id];
        //std::cout<<"Car attribute: "<<results[i].color<<" "<<results[i].type<<"\n";
    }
    return ;
}


void VehicleDetect::RenderVDResults(std::vector<VehicleDetectResult>& results, cv::Mat& image)
{
    for (unsigned int i = 0; i < results.size(); i++) {
        cv::rectangle(image, results[i].location, cv::Scalar(0, 255, 0), 2);
        std::stringstream va_str;
        va_str<<results[i].color<<" "<<results[i].type;
        cv::putText(image, va_str.str(), cv::Point(results[i].location.x, results[i].location.y + 20),
                cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 255, 0));
    }
    return;
}

VehicleDetect::~VehicleDetect()
{
    NetworkFactory::PutNetwork(mVDNetworkInfo);
    NetworkFactory::PutNetwork(mVANetworkInfo);
#if 0
    if (mEnablePerformanceReport)
    {
        std::cout << "Performance counts for vehicle detection:" << std::endl << std::endl;
        printPerformanceCounts(mDetectorRequest.GetPerformanceCounts(), std::cout, "GPU", false);
    }
#endif
}
