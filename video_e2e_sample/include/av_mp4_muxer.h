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
//Linux
#ifdef __cplusplus
extern "C"
{
#endif
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/file.h>
#include <libavcodec/avcodec.h>
#ifdef __cplusplus
};
#endif
#include <mutex>
#include "global_value.h"

#undef av_err2str
#define av_err2str(errnum) av_make_error_string((char*)__builtin_alloca(AV_ERROR_MAX_STRING_SIZE), AV_ERROR_MAX_STRING_SIZE, errnum)

class AVMP4Muxer
{
public:
    static AVMP4Muxer* GetInstance();
    int InitAudio(const char* m_outFileName);
    int InitVideo(const char* m_outFileName, int width, int height);
    void UpdateVideoFPS(int frameRate);
    int SendEncodedVideo(unsigned char* data, int size);
    int SendEncodedAudio(unsigned char* data, int size, int frameCount);
    void Finish();
    ~AVMP4Muxer();

private:
    AVOutputFormat* m_outFmt;
    AVFormatContext* m_outFmtCtx;
    AVBitStreamFilterContext* m_h264BsfltCtx;
    AVCodecContext* m_avCodecContext;
    const char* m_outFileName;
    int m_vFPS;

    AVStream* m_outStreamV;
    AVStream* m_outStreamA;

    AVPacket* m_avPacketV;
    AVPacket* m_avPacketA;

    int m_outStreamIndexV;
    int m_outStreamIndexA;

    uint64_t m_videoFrameIndex;
    uint64_t m_audioFrameIndex;

    bool m_bVideoInited;
    bool m_bAudioInited;
    bool m_bIsFinished;

    AVMP4Muxer();
    AVMP4Muxer(const AVMP4Muxer &obj);
    AVMP4Muxer& operator=(const AVMP4Muxer &obj);
    int writeHeader();
    static AVMP4Muxer* m_instancePtr;
    static std::mutex m_mutex_instance;
    std::mutex m_mutex;
    std::mutex m_mutex_finish;
};