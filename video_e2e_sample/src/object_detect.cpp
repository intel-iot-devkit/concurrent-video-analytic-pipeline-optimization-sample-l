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
#include "e2e_sample_infer_def.h"

using namespace InferenceEngine;
using namespace cv;

static int EntryIndex(int side, int lcoords, int lclasses, int location, int entry) {
    int n = location / (side * side);
    int loc = location % (side * side);
    return n * side * side * (lcoords + lclasses + 1) + entry * side * side + loc;
}

double IntersectionOverUnion(const DetectionObject &box_1, const DetectionObject &box_2) {
    double width_of_overlap_area = fmin(box_1.xmax, box_2.xmax) - fmax(box_1.xmin, box_2.xmin);
    double height_of_overlap_area = fmin(box_1.ymax, box_2.ymax) - fmax(box_1.ymin, box_2.ymin);
    double area_of_overlap;
    if (width_of_overlap_area < 0 || height_of_overlap_area < 0)
        area_of_overlap = 0;
    else
        area_of_overlap = width_of_overlap_area * height_of_overlap_area;
    double box_1_area = (box_1.ymax - box_1.ymin)  * (box_1.xmax - box_1.xmin);
    double box_2_area = (box_2.ymax - box_2.ymin)  * (box_2.xmax - box_2.xmin);
    double area_of_union = box_1_area + box_2_area - area_of_overlap;
    return area_of_overlap / area_of_union;
}

void ParseYOLOV3Output(const YoloParams &params, const std::string & output_name,
                       const Blob::Ptr &blob, const unsigned long resized_im_h,
                       const unsigned long resized_im_w, const unsigned long original_im_h,
                       const unsigned long original_im_w,
                       const double threshold, std::vector<DetectionObject> &objects) {

    const int out_blob_h = static_cast<int>(blob->getTensorDesc().getDims()[2]);
    const int out_blob_w = static_cast<int>(blob->getTensorDesc().getDims()[3]);
    if (out_blob_h != out_blob_w)
        throw std::runtime_error("Invalid size of output " + output_name +
        " It should be in NCHW layout and H should be equal to W. Current H = " + std::to_string(out_blob_h) +
        ", current W = " + std::to_string(out_blob_h));

    auto side = out_blob_h;
    auto side_square = side * side;
    LockedMemory<const void> blobMapped = as<MemoryBlob>(blob)->rmap();
    const float *output_blob = blobMapped.as<float *>();
    // --------------------------- Parsing YOLO Region output -------------------------------------
    for (int i = 0; i < side_square; ++i) {
        int row = i / side;
        int col = i % side;
        for (int n = 0; n < params.num; ++n) {
            int obj_index = EntryIndex(side, params.coords, params.classes, n * side * side + i, params.coords);
            int box_index = EntryIndex(side, params.coords, params.classes, n * side * side + i, 0);
            float scale = output_blob[obj_index];
            if (scale < threshold)
                continue;
            double x = (col + output_blob[box_index + 0 * side_square]) / side * resized_im_w;
            double y = (row + output_blob[box_index + 1 * side_square]) / side * resized_im_h;
            double height = std::exp(output_blob[box_index + 3 * side_square]) * params.anchors[2 * n + 1];
            double width = std::exp(output_blob[box_index + 2 * side_square]) * params.anchors[2 * n];
            for (int j = 0; j < params.classes; ++j) {
                int class_index = EntryIndex(side, params.coords, params.classes, n * side_square + i, params.coords + 1 + j);
                float prob = scale * output_blob[class_index];
                if (prob < threshold)
                    continue;
                DetectionObject obj(x, y, height, width, j, prob,
                        static_cast<float>(original_im_h) / static_cast<float>(resized_im_h),
                        static_cast<float>(original_im_w) / static_cast<float>(resized_im_w));
                objects.push_back(obj);
            }
        }
    }
}

ObjectDetect::ObjectDetect(bool enablePerformanceReport)
    :mDetectThreshold(0.6f),
    mRemoteBlob(false),
    mNetworkInfo(nullptr),
    mEnablePerformanceReport(enablePerformanceReport),
    mImgData(nullptr),
    mInferType(0)
{
    return;
}

int ObjectDetect::Init(const std::string& detectorModelPath, const int inferType,
        const std::string& targetDeviceName, bool remoteBlob, VADisplay vaDpy)
{
    static std::mutex initLock;
    std::lock_guard<std::mutex> lock(initLock);

    mInferType = inferType;
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

    switch (mInferType)
    {
        case InferTypeYolo:
            mYoloOutputInfo = detectorNetwork.getOutputsInfo();
            if (auto ngraphFunction = detectorNetwork.getFunction()) {
                for (const auto op : ngraphFunction->get_ops()) {
                    auto outputLayer = mYoloOutputInfo.find(op->get_friendly_name());
                    if (outputLayer != mYoloOutputInfo.end()) {
                        auto regionYolo = std::dynamic_pointer_cast<ngraph::op::RegionYolo>(op);
                        if (!regionYolo) {
                            throw std::runtime_error("Invalid output type: " +
                                    std::string(regionYolo->get_type_info().name) + ". RegionYolo expected");
                        }
                        yoloParams[outputLayer->first] = YoloParams(regionYolo);
                    }
                }
            }
            else {
                throw std::runtime_error("Can't get ngraph::Function. Make sure the provided model is in IR version 10 or greater.");
            }
            break;
        default:    
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
            break;
    }
   
    mDetectorRequest = mNetworkInfo->CreateNewInferRequest();

    return 0;
}

void ObjectDetect::SetSrcImageSize(int width, int height)
{
    mSrcImageSize.height = height;
    mSrcImageSize.width = width;
}

void ObjectDetect::Detect(const cv::Mat& image, std::vector<DetectionObject>& results)
{
    InferenceEngine::Blob::Ptr input = mDetectorRequest.GetBlob(mInputName);
    matU8ToBlob<uint8_t>(image, input);
    mDetectorRequest.Infer();
    CopyDetectResults(results);
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


void ObjectDetect::Detect(const cv::Mat& image, std::vector<ObjectDetectResult>& results)
{
    InferenceEngine::Blob::Ptr input = mDetectorRequest.GetBlob(mInputName);
    matU8ToBlob<uint8_t>(image, input);
    mDetectorRequest.Infer();

    const float *detections = mDetectorRequest.GetBlob(mDetectorOutputName)->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();

    CopyDetectResults(detections, results);
    return ;
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

void ObjectDetect::CopyDetectResults(std::vector<DetectionObject>& results)
{
    for (auto &output : mYoloOutputInfo) {
        auto output_name = output.first;
        Blob::Ptr blob = mDetectorRequest.GetBlob(output_name);
        ParseYOLOV3Output(yoloParams[output_name], output_name, blob, mInputH, mInputW, mSrcImageSize.height, mSrcImageSize.width, mDetectThreshold, results);
    }
    // Filtering overlapping boxes
    std::sort(results.begin(), results.end(), std::greater<DetectionObject>());
    for (size_t i = 0; i < results.size(); ++i) {
        if (results[i].confidence < mDetectThreshold)
            continue;
        for (size_t j = i + 1; j < results.size(); ++j)
            if (IntersectionOverUnion(results[i], results[j]) >= 0.4)
                results[j].confidence = 0;
    }
}


void ObjectDetect::RenderResults(std::vector<DetectionObject>& results, cv::Mat& image)
{
    bool verbose = false;
    // Drawing boxes
    for (auto &object : results) {
        if (object.confidence < mDetectThreshold)
            continue;
        auto label = object.class_id;
        float confidence = object.confidence;
        if (verbose) {
            std::cout << "[" << label << "] element, prob = " << confidence <<
                "    (" << object.xmin << "," << object.ymin << ")-(" << object.xmax << "," << object.ymax << ")"
                << ((confidence > mDetectThreshold) ? " WILL BE RENDERED!" : "") << std::endl;
        }
        /** Drawing only results when >confidence_threshold probability **/
        std::ostringstream conf;
        conf << ":" << std::fixed << std::setprecision(3) << confidence;
        cv::putText(image,
                (!labels.empty() ? labels[label] : std::string("label #") + std::to_string(label)) + conf.str(),
                cv::Point2f(static_cast<float>(object.xmin), static_cast<float>(object.ymin - 5)), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                cv::Scalar(0, 0, 255));
        cv::rectangle(image, cv::Point2f(static_cast<float>(object.xmin), static_cast<float>(object.ymin)),
                cv::Point2f(static_cast<float>(object.xmax), static_cast<float>(object.ymax)), cv::Scalar(0, 0, 255));
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
