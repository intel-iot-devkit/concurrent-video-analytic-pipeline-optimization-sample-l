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

#include "multi_object_tracker.hpp"

using namespace cv;

MultiObjectTracker::MultiObjectTracker()
    :mFrameIdx(0),
    mRenderW(480),
    mRenderH(270),
    mInputW(256),
    mInputH(128),
    mInferInterval(6)
{
    setScaleRatio();
}

void MultiObjectTracker::SetRenderSize(int width, int height) {
    mRenderW = width;
    mRenderH = height;
    setScaleRatio();
}


int MultiObjectTracker::Init(const std::string& detModelPath, const std::string& reidModelPath,
    const std::string& targetDeviceName, int inferInterval, bool remoteBlob, VADisplay vaDpy)
{
    mMotDetector = createMotDetector(detModelPath, targetDeviceName, remoteBlob, vaDpy);
    mMotTracker = createMotTracker(reidModelPath, targetDeviceName, inferInterval, false);

    return 0;
}

int MultiObjectTracker::RunInfer(mfxFrameData* pData, bool inferOffline) {
    unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
    Mat frameRGB4pad(mRenderH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pbuf);
    Rect content(0, 0, mRenderW, mRenderH);
    Mat frameRGB4 = frameRGB4pad(content);
    Mat frameScl(mInputH, mInputW, CV_8UC4);
    Mat frame(mInputH, mInputW, CV_8UC3);
    if (mRenderH == mInputH && mInputW == mRenderW)
    {
        cvtColor(frameRGB4, frame, COLOR_RGBA2RGB);
    }
    else
    {
        resize(frameRGB4, frameScl, Size(mInputW, mInputH));
        cvtColor(frameScl, frame, COLOR_RGBA2RGB);
    }

    mMotDetector->SubmitFrame(frame, mFrameIdx);
    mMotDetector->WaitAndFetchResults();
    mMotDetector->GetInputSize(mInputW, mInputH);
    setScaleRatio();
    TrackedObjects detections = mMotDetector->GetResults();

    int frameTimeInMs = mInferInterval * 1000 / 30; // Normally the fps is 30

    uint64_t cur_timestamp = static_cast<uint64_t>(frameTimeInMs * mFrameIdx);
    mMotTracker->Process(frame, detections, cur_timestamp);

    renderResult(frameRGB4);

    mFrameIdx++;

    return 0;
}

void MultiObjectTracker::renderResult(cv::Mat& frame) {

    // // Drawing colored "worms" (tracks).
    frame = mMotTracker->DrawActiveTracks(frame, mRatioW, mRatioH);

    // Drawing all detected objects on a frame by BLUE COLOR
    for (const auto& detection : mMotDetector->GetResults()) {

        Rect rect(detection.rect.x * mRatioW, detection.rect.y * mRatioH, detection.rect.width * mRatioW, detection.rect.height * mRatioH);
        cv::rectangle(frame, rect, cv::Scalar(255, 0, 0), 2);
    }

    // Drawing tracked detections only by RED color and print ID and detection
    // confidence level.
    for (const auto& detection : mMotTracker->TrackedDetections()) {

        Rect rect(detection.rect.x * mRatioW, detection.rect.y * mRatioH, detection.rect.width * mRatioW, detection.rect.height * mRatioH);
        cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 2);

        std::string text = std::to_string(detection.object_id) +
            " conf: " + std::to_string(detection.confidence);
        cv::putText(frame, text, rect.tl(), cv::FONT_HERSHEY_PLAIN,
            1.0, (0, 0, 255), 2);

    }
}

int MultiObjectTracker::RenderRepeatLast(mfxFrameData* pData) {
    unsigned char *pbuf = (pData->B < pData->R) ? pData->B : pData->R;
    Mat frameRGB4pad(mRenderH, pData->Pitch / 4, CV_8UC4, (unsigned char *)pbuf);
    Rect content(0, 0, mRenderW, mRenderH);
    Mat frameRGB4 = frameRGB4pad(content);

    renderResult(frameRGB4);
    return 0;
}

std::unique_ptr<MotObjectDetector> MultiObjectTracker::createMotDetector(const std::string& detModel,
    const std::string& deviceName, bool remoteBlob, VADisplay vaDpy) {
    std::unique_ptr<MotObjectDetector> detector(new MotObjectDetector());
    detector->Init(detModel, deviceName, remoteBlob, vaDpy);
    return detector;
}

std::unique_ptr<MotPedestrianTracker> MultiObjectTracker::createMotTracker(const std::string& reidModel,
    const std::string& deviceName, int inferInterval,
    bool shouldKeepTrackingInfo) {
    TrackerParams params;
    if (shouldKeepTrackingInfo) {
        params.drop_forgotten_tracks = false;
        params.max_num_objects_in_track = -1;
    }
    if (inferInterval > 0) {
        params.forget_delay = params.forget_delay / inferInterval;
        mInferInterval = inferInterval;
    }

    std::unique_ptr<MotPedestrianTracker> tracker(new MotPedestrianTracker(params));

    // Load reid-model.
    std::shared_ptr<IImageDescriptor> descriptor_fast =
        std::make_shared<ResizedImageDescriptor>(
            cv::Size(16, 32), cv::InterpolationFlags::INTER_LINEAR);
    std::shared_ptr<IDescriptorDistance> distance_fast =
        std::make_shared<MatchTemplateDistance>();

    tracker->set_descriptor_fast(descriptor_fast);
    tracker->set_distance_fast(distance_fast);

    if (!reidModel.empty()) {
        std::shared_ptr<IImageDescriptor> descriptor_strong = nullptr;
        if ((descriptor_strong = std::make_shared<MotIeDescriptor>(reidModel, deviceName)) != nullptr) {
            std::shared_ptr<IDescriptorDistance> distance_strong =
                std::make_shared<CosDistance>(descriptor_strong->size());

                tracker->set_descriptor_strong(descriptor_strong);
                tracker->set_distance_strong(distance_strong);
        }
        else {
            THROW_IE_EXCEPTION << "[SAMPLES] internal error - invalid descriptor";
        }

    }
    else {
        std::cout << "WARNING: Reid model "
            << "was not specified. "
            << "Only fast reidentification approach will be used." << std::endl;
    }

    return tracker;
}

void MultiObjectTracker::setScaleRatio() {
    if (mInputW != 0 && mInputH != 0) {
        mRatioW = mRenderW * 1.0f / mInputW;
        mRatioH = mRenderH * 1.0f / mInputH;
    }
}

TrackedObjects MultiObjectTracker::GetMotTrackerDetections() {
    return mMotTracker->TrackedDetections();
}

const TrackedObjects& MultiObjectTracker::GetMotDetectors() {
    return mMotDetector->GetResults();
}
