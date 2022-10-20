/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#include "av_mp4_muxer.h"
//#include <sys/time.h>

AVMP4Muxer* AVMP4Muxer::m_instancePtr = nullptr;
std::mutex AVMP4Muxer::m_mutex_instance;

AVMP4Muxer::AVMP4Muxer() :
    m_outFmt(nullptr),
    m_outFmtCtx(nullptr),
    m_h264BsfltCtx(nullptr),
    m_avCodecContext(nullptr),
    m_outFileName(nullptr),
    m_vFPS(0),
    m_outStreamV(nullptr),
    m_outStreamA(nullptr),
    m_avPacketV(nullptr),
    m_avPacketA(nullptr),
    m_outStreamIndexV(0),
    m_outStreamIndexA(0),
    m_videoFrameIndex(0),
    m_audioFrameIndex(0),
    m_bVideoInited(false),
    m_bAudioInited(false),
    m_bIsFinished(false)
{
}

AVMP4Muxer* AVMP4Muxer::GetInstance()
{
    std::lock_guard<std::mutex> lock(m_mutex_instance);
    if (m_instancePtr == nullptr)
    {
        m_instancePtr = new AVMP4Muxer();
    }
    return m_instancePtr;
}

void AVMP4Muxer::UpdateVideoFPS(int framerate)
{
    //This logic is because that for multiple channels with different framerate video source input, the lowest framerate of the video source will take the effect.
    if (m_vFPS <= 0 || m_vFPS > framerate)
    {
        m_vFPS = (framerate + 2) / 5 * 5; // set the fps to  the multiple of 5, for example, 59fps will be up around to 60.
    }
}
int AVMP4Muxer::InitAudio(const char* mp4file)
{
    int ret = 0;
    m_audioFrameIndex = 0;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_outFmtCtx == nullptr)
    {
        m_outFileName = mp4file;
        if ((ret = avformat_alloc_output_context2(&m_outFmtCtx, NULL, NULL, m_outFileName)) < 0)
        {
            printf("Error occurred in AVMP4Muxer::InitAudio when alloc output context for mp4: %s\n",
                av_err2str(ret));
            return -1;
        }
        m_outFmt = m_outFmtCtx->oformat;
    }
    if (m_outFmt->audio_codec != AV_CODEC_ID_NONE)
    {
        AVCodec* codec = avcodec_find_decoder(AVCodecID::AV_CODEC_ID_MP3);
        m_outStreamA = avformat_new_stream(m_outFmtCtx, codec);
        if (m_outStreamA == nullptr)
        {
            printf("Error occurred in AVMP4Muxer::InitAudio, avformat new audio output stream failed! \n");
            return -1;
        }
        m_outStreamIndexA = m_outStreamA->index;

        m_outStreamA->codecpar->codec_type = AVMediaType::AVMEDIA_TYPE_AUDIO;
        m_outStreamA->codecpar->codec_id = AVCodecID::AV_CODEC_ID_MP3;
        m_outStreamA->codecpar->codec_tag = 0;
        m_outStreamA->codecpar->format = AV_SAMPLE_FMT_FLTP;
        m_outStreamA->codecpar->bit_rate = TranscodingSample::LT_VCD_Audio_MP3_Bitrate;
        m_outStreamA->codecpar->channel_layout = 3;
        m_outStreamA->codecpar->channels = TranscodingSample::LT_VCD_Audio_Channel;
        m_outStreamA->codecpar->sample_rate = TranscodingSample::LT_VCD_Audio_MP3_Sample_rate;
        m_outStreamA->codecpar->block_align = 0;
        m_outStreamA->codecpar->frame_size = 4092;
        AVCodecContext* aCodecContext = avcodec_alloc_context3(codec);
        avcodec_parameters_to_context(aCodecContext, m_outStreamA->codecpar);
        avcodec_free_context(&aCodecContext);

    }
    m_bAudioInited = true;
    if (m_bVideoInited)
    {
        return writeHeader();
    }
    return ret;
}

int AVMP4Muxer::InitVideo(const char* mp4file, int width, int height)
{
    int ret = 0;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_videoFrameIndex = 0;
    if (m_outFmtCtx == nullptr)
    {
        m_outFileName = mp4file;
        if ((ret = avformat_alloc_output_context2(&m_outFmtCtx, NULL, NULL, m_outFileName)) < 0)
        {
             printf("Error occurred in AVMP4Muxer::InitVideo, When alloc output context for mp4: %s\n",
                    av_err2str(ret));
             return ret;
        }
        m_outFmt = m_outFmtCtx->oformat;
    }
    if (m_outFmt->video_codec != AV_CODEC_ID_NONE)
    {
        AVCodec* codec = avcodec_find_decoder(AVCodecID::AV_CODEC_ID_H264);
        m_outStreamV = avformat_new_stream(m_outFmtCtx, codec);
        if (m_outStreamV == nullptr)
        {
            printf("Error occurred in AVMP4Muxer::InitVideo, When new avformat output stream \n");
            return -1;
        }
        m_outStreamIndexV = m_outStreamV->index;

        m_outStreamV->codecpar->codec_type = AVMediaType::AVMEDIA_TYPE_VIDEO;
        m_outStreamV->codecpar->codec_id = AVCodecID::AV_CODEC_ID_H264;
        m_outStreamV->codecpar->codec_tag = 0;
        m_outStreamV->codecpar->format = AV_PIX_FMT_NONE;
        m_outStreamV->codecpar->bits_per_raw_sample = 8;
        m_outStreamV->codecpar->profile = FF_PROFILE_H264_HIGH;
        m_outStreamV->codecpar->level = 32;
        m_outStreamV->codecpar->width = width;
        m_outStreamV->codecpar->height = height;
        // codecpar->sample_aspect_ratio = new ;
        m_outStreamV->codecpar->field_order = AVFieldOrder::AV_FIELD_PROGRESSIVE;
        m_outStreamV->codecpar->color_range = AVColorRange::AVCOL_RANGE_UNSPECIFIED;
        m_outStreamV->codecpar->color_primaries = AVColorPrimaries::AVCOL_PRI_UNSPECIFIED;
        m_outStreamV->codecpar->color_trc = AVColorTransferCharacteristic::AVCOL_TRC_UNSPECIFIED;
        m_outStreamV->codecpar->color_space = AVColorSpace::AVCOL_SPC_UNSPECIFIED;
        m_outStreamV->codecpar->chroma_location = AVChromaLocation::AVCHROMA_LOC_LEFT;
        m_outStreamV->codecpar->video_delay = 2;
        m_avCodecContext = avcodec_alloc_context3(codec);
        avcodec_parameters_to_context(m_avCodecContext, m_outStreamV->codecpar);
    }

    m_bVideoInited = true;
    
    if (m_bAudioInited)
    {
       return writeHeader();
    }
    return ret;
}

int AVMP4Muxer::writeHeader()
{
    int ret = 0;
    if (m_outFmtCtx == nullptr || m_outFmt == nullptr || !m_bAudioInited || !m_bVideoInited)
    {
        return 0;
    }
//    av_dump_format(m_outFmtCtx, 1, m_outFileName, 1);
    if (!(m_outFmt->flags & AVFMT_NOFILE))
    {
        if ((ret = avio_open(&m_outFmtCtx->pb, m_outFileName, AVIO_FLAG_WRITE)) < 0)
        {
            printf("Error occurred in AVMP4Muxer::writeHeader, When avio open for mp4 file: %s: %s\n", m_outFileName,
                av_err2str(ret));
            return ret;
        }
    }
    if ((ret = avformat_write_header(m_outFmtCtx, NULL)) < 0)
    {
        printf("Error occurred in AVMP4Muxer::writeHeader, When write header for mp4: %s\n",
            av_err2str(ret));
        return ret;
    }
//    m_h264BsfltCtx = av_bitstream_filter_init("h264_mp4toannexb");
    return ret;
}

int AVMP4Muxer::SendEncodedVideo(unsigned char* data, int size)
{
    int ret = 0;
    if (m_outFmtCtx == nullptr || m_bIsFinished)
    {
//       printf("The MP4 muxer is not inited, or the encoded is finished. \n");
        return 0;
    }
    m_avPacketV = av_packet_alloc();
    if (m_avPacketV == nullptr)
    {
        printf("Error occurred in AVMP4Muxer::writeHeader, Alloc video avPacket failed. \n");
        return -1;
    }
    av_init_packet(m_avPacketV);
    unsigned char* tempData = (unsigned char*)malloc(sizeof(unsigned char) * size);
    if (tempData == nullptr)
    {
        printf("Error occurred in AVMP4Muxer::SendEncodedVideo,malloc tempData failed. \n");
        return 0;
    }
 //   unsigned char* tmpeData = (unsigned char*)av_malloc(size);
    memcpy(tempData, data, size);
    if ((ret = av_packet_from_data(m_avPacketV, tempData, size)) < 0)
    {
        printf("Error occurred in AVMP4Muxer::SendEncodedVideo,av_packet_from_data failed. \n");
        free(tempData);
        tempData = nullptr;
        av_packet_free(&m_avPacketV);
        return 0;
    }
    m_avPacketV->stream_index = m_outStreamIndexV;

    AVRational time_base = m_outStreamV->time_base;
    int fps = 30;
    if (m_vFPS > 0)
    {
        fps = m_vFPS;
    }
    long int calc_duration = (double)AV_TIME_BASE / fps; // default
    m_avPacketV->pts = (double)(m_videoFrameIndex * calc_duration) / (double)(av_q2d(time_base) * AV_TIME_BASE);
    m_avPacketV->dts = m_avPacketV->pts;
    m_avPacketV->duration = (double)calc_duration / ((double)av_q2d(time_base) * AV_TIME_BASE);
//    struct timeval tv;
//    gettimeofday(&tv, NULL);
//    printf("timestame %ld:%ld, video data size: %d ,video time_base: %d ,  video pkt->pts: %ld, video pkt->duration: %ld \n",
//        tv.tv_sec, tv.tv_usec, size, time_base.den, m_avPacketV->pts, m_avPacketV->duration);
//    av_bitstream_filter_filter(m_h264BsfltCtx, m_avCodecContext, NULL, &(m_avPacketV->data), &(m_avPacketV->size), m_avPacketV->data, m_avPacketV->size, 0);
    m_videoFrameIndex++;
    std::lock_guard<std::mutex> lock(m_mutex);
    if ((ret = av_interleaved_write_frame(m_outFmtCtx, m_avPacketV)) < 0)
    {
        printf("Error occurred in AVMP4Muxer::SendEncodedVideo, When write interleave write video frame for mp4: %s\n",
            av_err2str(ret));
    }

    av_packet_free(&m_avPacketV);
     m_avPacketV = nullptr;
    if (TranscodingSample::GlobalValue::AppStop)
    {
        Finish();
    }
    return 0;
}

int AVMP4Muxer::SendEncodedAudio(unsigned char* data, int size, int frameCount)
{
    int ret = 0;
    if (m_outFmtCtx == nullptr || m_bIsFinished)
    {
        //       printf("Error! The MP4 muxer is not inited, or the encoded is finished.\n");
        return 0;
    }
    m_avPacketA = av_packet_alloc();
    if (m_avPacketA == nullptr)
    {
        printf("Error occurred in AVMP4Muxer::SendEncodedAudio, Alloc audio avPacket failed. \n");
        return 0;
    }
    av_init_packet(m_avPacketA);
    unsigned char* tempData = (unsigned char*)malloc(sizeof(unsigned char) * size);
    if (tempData == nullptr)
    {
        printf("Error occurred in AVMP4Muxer::SendEncodedAudio,malloc tempData failed. \n");
        return 0;
    }
    memcpy(tempData, data, size);
    if ((ret = av_packet_from_data(m_avPacketA, tempData, size)) < 0)
    {
        free(tempData);
        tempData = nullptr;
        av_packet_free(&m_avPacketA);
        return 0;
    }

    m_avPacketA->stream_index = m_outStreamIndexA;

    AVRational time_base = m_outStreamA->time_base; //TODO
    long int calc_duration = frameCount * (double)AV_TIME_BASE / m_outStreamA->codecpar->sample_rate;
    m_avPacketA->pts = (double)(m_audioFrameIndex * calc_duration) / (double)(av_q2d(time_base) * AV_TIME_BASE);
    m_avPacketA->dts = m_avPacketA->pts;
    m_avPacketA->duration = (double)calc_duration / ((double)av_q2d(time_base) * AV_TIME_BASE);
//    struct timeval tv;
//    gettimeofday(&tv, NULL);
//    printf("timestame %ld:%ld,  audio data size: %d ,audio time_base: %d ,  audio pkt->pts: %ld, audio pkt->duration: %ld \n",
//        tv.tv_sec, tv.tv_usec, size, time_base.den, m_avPacketA->pts, m_avPacketA->duration);
    m_audioFrameIndex++;
    std::lock_guard<std::mutex> lock(m_mutex);
    if ((ret = av_interleaved_write_frame(m_outFmtCtx, m_avPacketA)) < 0)
    {
        printf("Error occured in AVMP4Muxer::SendEncodedAudio, When write interleaved audio frame for mp4: %s\n",
            av_err2str(ret));
    }

    av_packet_free(&m_avPacketA);
    if (TranscodingSample::GlobalValue::AppStop)
    {
        Finish();
    }
    return 0;
}
void AVMP4Muxer::Finish()
{
    int ret = 0;
    std::lock_guard<std::mutex> guard(m_mutex_finish);
    if (m_bIsFinished)
    {
//      printf("Mp4 mux already has finished! \n");
        return;
    }
    if (m_outFmtCtx == nullptr)
    {
        printf("Error occured in AVMP4Muxer::Finish, m_outFmtCtx is null when finish the MP4 mux! \n");
        return;
    }
    if ((ret = av_write_trailer(m_outFmtCtx)) < 0)
    {
        printf("Error occured in AVMP4Muxer::Finish, When write trailer of mp4: %s \n",
            av_err2str(ret));
    }
    if (m_h264BsfltCtx != nullptr)
    {
        av_bitstream_filter_close(m_h264BsfltCtx);
        m_h264BsfltCtx = nullptr;
    }
    if (m_avCodecContext != nullptr)
    {
        avcodec_free_context(&m_avCodecContext);
        m_avCodecContext = nullptr;
    }
    if (m_avPacketV != nullptr)
    {
        av_packet_free(&m_avPacketV);
        m_avPacketV = nullptr;
    }
    if (m_avPacketA != nullptr)
    {
        av_packet_free(&m_avPacketA);
        m_avPacketA = nullptr;
    }

    if (m_outFmtCtx != nullptr && !(m_outFmtCtx->flags & AVFMT_NOFILE))
    {
        if ((ret = avio_closep(&m_outFmtCtx->pb)) < 0)
        {
            printf("Error occurred in AVMP4Muxer::Finish, When close av io for mp4: %s\n",
                av_err2str(ret));
        }
    }
    avformat_free_context(m_outFmtCtx);
    m_outFmtCtx = nullptr;
    m_bIsFinished = true;
    printf("MP4 encode finished! \n");
}

AVMP4Muxer::~AVMP4Muxer()
{
    if (m_outFmt != nullptr)
    {
        delete m_outFmt;
        m_outFmt = nullptr;
    }
    if (m_outFmtCtx != nullptr)
    {
        delete m_outFmtCtx;
        m_outFmtCtx = nullptr;
    }
    if (m_outStreamV != nullptr)
    {
        delete m_outStreamV;
        m_outStreamV = nullptr;
    }
    if (m_outStreamA != nullptr)
    {
        delete m_outStreamA;
        m_outStreamA = nullptr;
    }
    if (m_outFileName != nullptr)
    {
        m_outFileName = nullptr;
    }
    if (m_avPacketV != nullptr)
    {
        m_avPacketV = nullptr;
    }
    if (m_avPacketA != nullptr)
    {
        m_avPacketA = nullptr;
    }
    if (m_h264BsfltCtx != nullptr)
    {
        delete m_h264BsfltCtx;
        m_h264BsfltCtx = nullptr;
    }
}