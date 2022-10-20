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
#include <opencv2/opencv.hpp>
//#include <samples/ocv_common.hpp>
#include <samples/common.hpp>

#include <cldnn/cldnn_config.hpp>
#include "openvino/openvino.hpp"
#include "object_detect.hpp"
#include "sample_defs.h"
#include "e2e_sample_infer_def.h"

using namespace InferenceEngine;
using namespace cv;
using namespace ov;

std::vector<float> defaultAnchors[] = {
    // YOLOv1v2
    {0.57273f, 0.677385f, 1.87446f, 2.06253f, 3.33843f, 5.47434f, 7.88282f, 3.52778f, 9.77052f, 9.16828f},
    // YOLOv3
    {10.0f,
     13.0f,
     16.0f,
     30.0f,
     33.0f,
     23.0f,
     30.0f,
     61.0f,
     62.0f,
     45.0f,
     59.0f,
     119.0f,
     116.0f,
     90.0f,
     156.0f,
     198.0f,
     373.0f,
     326.0f},
    // YOLOv4
    {12.0f,
     16.0f,
     19.0f,
     36.0f,
     40.0f,
     28.0f,
     36.0f,
     75.0f,
     76.0f,
     55.0f,
     72.0f,
     146.0f,
     142.0f,
     110.0f,
     192.0f,
     243.0f,
     459.0f,
     401.0f},
    // YOLOv4_Tiny
    {10.0f, 14.0f, 23.0f, 27.0f, 37.0f, 58.0f, 81.0f, 82.0f, 135.0f, 169.0f, 344.0f, 319.0f},
    // YOLOF
    {16.0f, 16.0f, 32.0f, 32.0f, 64.0f, 64.0f, 128.0f, 128.0f, 256.0f, 256.0f, 512.0f, 512.0f}};

const std::vector<int64_t> defaultMasks[] = {
    // YOLOv1v2
    {},
    // YOLOv3
    {},
    // YOLOv4
    {0, 1, 2, 3, 4, 5, 6, 7, 8},
    // YOLOv4_Tiny
    {1, 2, 3, 3, 4, 5},
    // YOLOF
    {0, 1, 2, 3, 4, 5}};



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


void parseYOLOOutput(ov::Tensor tensor,
                    const YoloParams &yoloParams, const unsigned long resized_im_h,
                    const unsigned long resized_im_w, const unsigned long original_im_h,
                    const unsigned long original_im_w,
                    const double threshold, std::vector<DetectionObject> &objects) {

    const int height = static_cast<int>(tensor.get_shape()[2]);
    const int width = static_cast<int>(tensor.get_shape()[3]);
    if (height != width)
        throw std::runtime_error("Invalid size of output. It should be in NCHW layout and H should be equal to W. Current H = " + std::to_string(height) +
        ", current W = " + std::to_string(height));

    auto num = yoloParams.num;
    auto coords = yoloParams.coords;
    auto classes = yoloParams.classes;

    auto anchors = yoloParams.anchors;

    auto side = height;
    auto side_square = side * side;
    const float* data = tensor.data<float>();
    // --------------------------- Parsing YOLO Region output -------------------------------------
    for (int i = 0; i < side_square; ++i) {
        int row = i / side;
        int col = i % side;
        for (int n = 0; n < num; ++n) {
            int obj_index = EntryIndex(side, coords, classes, n * side * side + i, coords);
            int box_index = EntryIndex(side, coords, classes, n * side * side + i, 0);
            float scale = data[obj_index];
            if (scale < threshold)
                continue;
            double x = (col + data[box_index + 0 * side_square]) / side * resized_im_w;
            double y = (row + data[box_index + 1 * side_square]) / side * resized_im_h;
            double height = std::exp(data[box_index + 3 * side_square]) * anchors[2 * n + 1];
            double width = std::exp(data[box_index + 2 * side_square]) * anchors[2 * n];
            for (int j = 0; j < classes; ++j) {
                int class_index = EntryIndex(side, coords, classes, n * side_square + i, coords + 1 + j);
                float prob = scale * data[class_index];
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
    mVaDpy = vaDpy;
    
    mNetworkInfo = NetworkFactory::GetNetwork(detectorModelPath, targetDeviceName, opt);

    if (!mNetworkInfo)
    {
        std::cout<<"Loading network "<<detectorModelPath<<" failed"<<std::endl;
        return -1;
    }

    mDetectorRequest1 = mNetworkInfo->CreateNewInferRequest2();

    mDetectorObjectSize = mNetworkInfo->m_object_size;
    mDetectorMaxProposalCount= mNetworkInfo->m_max_detections_count;
    mInput_tensor_name = mNetworkInfo->input_tensor_name;
    mOutput_tensor_name = mNetworkInfo->output_tensor_name;

    const ov::Shape& inputShape = mNetworkInfo->mCompiled_model.input().get_shape();
    if (inputShape.size() != 4)
    {
        std::cout<<"Wrong input shape size : "<<inputShape.size()<<std::endl;
        return -1;
    }

    ov::Layout modelLayout = ov::layout::get_layout(mNetworkInfo->mCompiled_model.input());
    mInputH = inputShape[ov::layout::height_idx(modelLayout)];
    mInputW = inputShape[ov::layout::width_idx(modelLayout)];

    switch (mInferType)
    {
        case InferTypeYolo:
            preprocessYolo(mNetworkInfo->mCompiled_model);
            break;
        default:
            break;
    }
    return 0;
}

int ObjectDetect::preprocessYolo(ov::CompiledModel &model)
{
    ov::Layout yoloRegionLayout = {"NCHW"};
    auto outputs = model.outputs();
    int num =  3;
    bool isObjConf = 1;
    int i = 0;

    std::vector<std::string> inputsNames;
    std::vector<std::string> outputsNames;
    std::map<std::string, ov::Shape> outShapes;

    for (auto &out : outputs)	
    {
        outputsNames.push_back(out.get_any_name());
        outShapes[out.get_any_name()] = out.get_shape();
    }
    enum YoloVersion { YOLO_V1V2, YOLO_V3, YOLO_V4, YOLO_V4_TINY, YOLOF };
    YoloVersion yoloVersion = YOLO_V3;

    switch (outputsNames.size()) {
        case 1:
            yoloVersion = YOLOF;
            break;
        case 2:
            yoloVersion = YOLO_V4_TINY;
            break;
        case 3:
            yoloVersion = YOLO_V4;
            break;
    }

    auto chosenMasks = defaultMasks[yoloVersion];

    std::sort(outputsNames.begin(),
            outputsNames.end(),
            [&outShapes, &yoloRegionLayout](const std::string& x, const std::string& y) {
            return outShapes[x][ov::layout::height_idx(yoloRegionLayout)] >
            outShapes[y][ov::layout::height_idx(yoloRegionLayout)];
            });

    for (const auto& name : outputsNames) {
        const auto& shape = outShapes[name];
        if (shape[ov::layout::channels_idx(yoloRegionLayout)] % num != 0) {
            std::cout<< shape[ov::layout::channels_idx(yoloRegionLayout)] <<" : "<<num<<std::endl;
            std::cout<<std::string("Output tenosor ") + name + " has wrong 2nd dimension"<<std::endl;
            return -1;
        }
        yoloParams.emplace(name,
                YoloParams(shape[ov::layout::channels_idx(yoloRegionLayout)] / num - 4 - (isObjConf ? 1 : 0),
                    4,
                    defaultAnchors[yoloVersion],
                    std::vector<int64_t>(chosenMasks.begin() + i * num, chosenMasks.begin() + (i + 1) * num),
                    shape[ov::layout::width_idx(yoloRegionLayout)],
                    shape[ov::layout::height_idx(yoloRegionLayout)]));
        i++;
    }

    return 0;
}


void ObjectDetect::SetSrcImageSize(int width, int height)
{
    mSrcImageSize.height = height;
    mSrcImageSize.width = width;
}

void ObjectDetect::Detect(const cv::Mat& image, std::vector<DetectionObject>& results)
{
    ov::Tensor inputTensor = mDetectorRequest1->get_tensor(mInput_tensor_name);
    static const ov::Layout layout{"NHWC"};
    const ov::Shape& shape = inputTensor.get_shape();
    cv::Size size{int(shape[ov::layout::width_idx(layout)]), int(shape[ov::layout::height_idx(layout)])};
    cv::Mat rgbImg = cv::Mat{size, CV_8UC3, inputTensor.data()};
    cv::resize(image, rgbImg, size);

    mDetectorRequest1->infer();

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


/*
static inline void resize2tensor(const cv::Mat& mat, const ov::Tensor& tensor) {
    static const ov::Layout layout{"NHWC"};
    const ov::Shape& shape = tensor.get_shape();
    cv::Size size{int(shape[ov::layout::width_idx(layout)]), int(shape[ov::layout::height_idx(layout)])};
    assert(tensor.get_element_type() == ov::element::u8);
    assert(shape.size() == 4);
    assert(shape[ov::layout::batch_idx(layout)] == 1);
    assert(shape[ov::layout::channels_idx(layout)] == 3);
    cv::resize(mat, cv::Mat{size, CV_8UC3, tensor.data()}, size);
}
*/

void ObjectDetect::Detect(const cv::Mat& image, std::vector<ObjectDetectResult>& results)
{
    ov::Tensor inputTensor = mDetectorRequest1->get_tensor(mInput_tensor_name);
    static const ov::Layout layout{"NHWC"};
    const ov::Shape& shape = inputTensor.get_shape(); 
    cv::Size size{int(shape[ov::layout::width_idx(layout)]), int(shape[ov::layout::height_idx(layout)])};
    cv::resize(image, cv::Mat{size, CV_8UC3, inputTensor.data()}, size);

    mDetectorRequest1->infer();

    const float *detections = mDetectorRequest1->get_tensor(mOutput_tensor_name).data<float>();
    CopyDetectResults(detections, results);

    return ;
}

void ObjectDetect::Detect(VASurfaceID va_surface, std::vector<ObjectDetectResult>& results)
{
    if (!mNetworkInfo)
    {
        return;
    }

    auto nv12_blob = mNetworkInfo->mVAContext->create_tensor_nv12(mInputH, mInputW, va_surface);

    auto model = mNetworkInfo->mModel;
    auto input0 = model->get_parameters().at(0);
    auto input1 = model->get_parameters().at(1);

    auto shape = input0->get_shape();
    auto width = shape[1];
    auto height = shape[2];

    mDetectorRequest1->set_tensor(input0->get_friendly_name(), nv12_blob.first);
    mDetectorRequest1->set_tensor(input1->get_friendly_name(), nv12_blob.second);
    mDetectorRequest1->infer();

    const float *detections = mDetectorRequest1->get_tensor(mOutput_tensor_name).data<float>();
    CopyDetectResults(detections, results);

    //std::cout<<"infer results size " <<results.size()<<std::endl;

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

    // Parsing outputs
    for (auto &idxParams : yoloParams) {
        parseYOLOOutput(mDetectorRequest1->get_tensor(idxParams.first), idxParams.second, mInputH, mInputW, mSrcImageSize.height, mSrcImageSize.width, mDetectThreshold, results);
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
        //printPerformanceCounts(mDetectorRequest.GetPerformanceCounts(), std::cout, "GPU", false);
    }
}
