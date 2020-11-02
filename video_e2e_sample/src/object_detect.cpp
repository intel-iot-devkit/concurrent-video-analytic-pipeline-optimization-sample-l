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

#include <cldnn/cldnn_config.hpp>

#include "object_detect.hpp"
#include "sample_defs.h"

using namespace InferenceEngine;
using namespace cv;


ObjectDetect::ObjectDetect(bool enablePerformanceReport)
    :mDetectThreshold(0.5f),
    mRemoteBlob(false),
    mNetworkInfo(nullptr),
    mEnablePerformanceReport(enablePerformanceReport),
    mImgData(nullptr)
{
    return;
}

int ObjectDetect::Init(const std::string& detectorModelPath,
        const std::string& targetDeviceName, bool remoteBlob, VADisplay vaDpy)
{
    static std::mutex initLock;
    std::lock_guard<std::mutex> lock(initLock);

    NetworkOptions opt;
    if (remoteBlob)
    {
        mRemoteBlob = remoteBlob;
        opt.enableRemoteBlob = remoteBlob;
        opt.vaDpy = vaDpy;
    }
    mNetworkInfo = NetworkFactory::GetNetwork(detectorModelPath, targetDeviceName, opt);

    if (!mNetworkInfo)
    {
        std::cout<<"NetworkFactory::GetNetwork("<<detectorModelPath<<","<<targetDeviceName<<") failed!"<<std::endl;
        return -1;
    }

    std::cout<<"Loading network "<<detectorModelPath<<" on device "<<targetDeviceName<<" is done."<<std::endl;
    InferenceEngine::CNNNetwork &detectorNetwork = mNetworkInfo->mNetwork;

    auto inputP = detectorNetwork.getInputsInfo().begin();
    if (inputP == detectorNetwork.getInputsInfo().end())
    {
        return -1;
    } 
    InferenceEngine::InputInfo::Ptr inputInfo = inputP->second;
    mInputName = inputInfo->name();
    InferenceEngine::SizeVector blobSize = inputInfo->getTensorDesc().getDims();

    /*Both MobileNet SSD and face detectin mobile input are 300x300 */
    mInputW = blobSize[3];
    mInputH = blobSize[2];

    /* All supported models, input size is between 300x300 and 1920x1080 */
    if (mInputH < 300 || mInputH > 1080 || mInputW < 300 || mInputW > 1920)
    {
        return -1;
    }

    InferenceEngine::OutputsDataMap outputInfo = detectorNetwork.getOutputsInfo();
    auto outputBlobsIt = outputInfo.begin();
    if (outputBlobsIt == outputInfo.end())
    {
        return -1;
    }
    mDetectorRoiBlobName = outputBlobsIt->first;
    DataPtr& output = outputInfo.begin()->second;

    const SizeVector outputDims = output->getTensorDesc().getDims();
    mDetectorOutputName = outputInfo.begin()->first;

    mDetectorMaxProposalCount = outputDims[2];
    mDetectorObjectSize = outputDims[3];
   
    mDetectorRequest = mNetworkInfo->CreateNewInferRequest();

    return 0;
}

void ObjectDetect::SetSrcImageSize(int width, int height)
{
    mSrcImageSize.height = height;
    mSrcImageSize.width = width;
}

void ObjectDetect::Detect(const cv::Mat& image, std::vector<ObjectDetectResult>& results)
{
    InferenceEngine::Blob::Ptr input = mDetectorRequest.GetBlob(mInputName);
    matU8ToBlob<uint8_t>(image, input);
    mDetectorRequest.Infer();

    const float *detections = mDetectorRequest.GetBlob(mDetectorOutputName)->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();

    CopyDetectResults(detections, results);
    return ;
}

int ObjectDetect::CopyImageData(unsigned char *dst, unsigned char *src, unsigned int width, unsigned int height, unsigned int pitch)
{
    for (unsigned int i = 0; i < height; i++)
    {
        memcpy(dst, src, width);
        dst += width;
        src += pitch;
    }
    return 0;
}

int ObjectDetect::Detect(mfxFrameData *pData, mfxFrameData *pData_dec, std::vector<ObjectDetectResult>& results)
{
    int alignedH = MSDK_ALIGN16(mInputH); 
    int alignedW = MSDK_ALIGN32(mInputW); 

    /*Memory to store the input of inference */
    if (!mImgData)
    {
        mImgData = (unsigned char *)malloc(mInputW * mInputH * 3);
        if (!mImgData)
        {
            return -1;
        }
    }

    CopyImageData(mImgData, pData->B, mInputW, mInputH, alignedW);
    CopyImageData(mImgData +  mInputW * mInputH, pData->G, mInputW, mInputH, alignedW);
    CopyImageData(mImgData +  mInputW * mInputH * 2, pData->R, mInputW, mInputH, alignedW);
    cv::Mat frame(mInputW, mInputH, CV_8UC3,  (char *)mImgData);
    InferenceEngine::TensorDesc tDesc(InferenceEngine::Precision::U8,
                                      {1, 3, mInputH, mInputW},
                                      InferenceEngine::Layout::NHWC);

    auto blob = InferenceEngine::make_shared_blob<uint8_t>(tDesc, frame.data);
    mDetectorRequest.SetBlob(mInputName, blob);
    mDetectorRequest.Infer();

    const float *detections = mDetectorRequest.GetBlob(mDetectorOutputName)->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();
    CopyDetectResults(detections, results);

    return 0;
}


void ObjectDetect::Detect(VASurfaceID va_surface, std::vector<ObjectDetectResult>& results)
{
    /*
    auto inputIt = mDetectorNetwork.getInputsInfo().begin();
    if (inputIt == mDetectorNetwork.getInputsInfo().end())
    {
        return;
    }*/
    //mInputW x mInputH is the resolution of va_surface which must be equal to input of inference
    if (!mNetworkInfo)
    {
        return;
    }
    auto nv12_blob = gpu::make_shared_blob_nv12(mInputW,
                                                mInputH,
                                                mNetworkInfo->mSharedVAContext,
                                                va_surface
                                                );
    mDetectorRequest.SetBlob(mInputName, nv12_blob);

    mDetectorRequest.Infer();

    const float *detections = mDetectorRequest.GetBlob(mDetectorOutputName)->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();
    CopyDetectResults(detections, results);
    return ;
}

void ObjectDetect::CopyDetectResults(const float *detections, std::vector<ObjectDetectResult>& results)
{
    for (int i = 0; i < mDetectorMaxProposalCount; i++)
    {
        float image_id = detections[i * mDetectorObjectSize + 0];  // in case of batch
        ObjectDetectResult r;
        r.label = static_cast<int>(detections[i * mDetectorObjectSize + 1]);
#ifdef USE_MOBILENET_SSD
        //Only saved the result of person
        if (r.label != 15)
        {
            continue;
        }
#endif
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

}

void ObjectDetect::RenderResults(std::vector<ObjectDetectResult>& results, cv::Mat& image, bool isGrey)
{
    char confidence[8];
    for (unsigned int i = 0; i < results.size(); i++) {
        cv::Rect &location = results[i].location;

        if(isGrey)
        {
            rectangle(image, results[i].location, Scalar(230, 230, 230), 1, 4);
        }
        else
        {
            rectangle(image, results[i].location, Scalar(0, 230, 0), 1, 4);

            /* background color */
            rectangle(image, Point(location.x - 1, location.y - 8), Point(location.x + location.width, location.y - 1), Scalar(0, 230, 0), -1, LINE_8, 0);
            snprintf(confidence, 8, "%.2f", results[i].confidence);

            /* text */
            putText(image, confidence, Point(location.x + 1, location.y - 1), FONT_HERSHEY_TRIPLEX, .3, Scalar(70,70,70));
        }
    }
    return;
}

void ObjectDetect::GetInputSize(int &width, int &height)
{
    width = mInputW;
    height = mInputH;
}

ObjectDetect::~ObjectDetect()
{
    free(mImgData);
    mImgData = nullptr;

    NetworkFactory::PutNetwork(mNetworkInfo);
    if (mEnablePerformanceReport)
    {
        std::cout << "Performance counts for object detection:" << std::endl << std::endl;
        printPerformanceCounts(mDetectorRequest.GetPerformanceCounts(), std::cout, "GPU", false);
    }
}
