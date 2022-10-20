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

#include <stdio.h>
#include <thread>
#include <alsa/asoundlib.h>
#include <sys/time.h>
#include "alsa_vcd_device.h"
#include "lame_mp3_encoder.h"
#include "av_mp4_muxer.h"
#include "global_value.h"
#include "vpl/mfxcommon.h"

struct AudioOpreation
{
    bool playback;
    bool mp3Encode;
    bool mp3Save;
    bool mp4Mux;
};

class AlsaAudioStreamReader
{
public:
    AlsaAudioStreamReader(const char* captureHwName, const char* playbackHwName, const char* strMP4File, const char* strMP3File);
    ~AlsaAudioStreamReader();

    void     Stop();
    mfxStatus  Init();
    void StartAudioThread();

protected:

    std::thread* m_audio_thread;

private:
    AlsaAudioStreamReader(AlsaAudioStreamReader const&);
    AlsaAudioStreamReader& operator=(AlsaAudioStreamReader const&);

    static void AlsaPcmProcesser(AlsaAudioStreamReader* ctx);

    void audioProcess();
    int pcm_write(unsigned char* data, int count);
    int pcm_read(unsigned char* data, int rcount);
    void close();

    AlsaVcdDevice* m_pAlsaCapture;
    AlsaVcdDevice* m_pAlsaPlayback;
    LameMp3Encoder* m_pLameMp3Encoder;
    AVMP4Muxer* m_pAVMP4Muxer;

    AlsaAudioParam m_captureParam;
    AlsaAudioParam m_playbackParam;

    const char* m_mp3FileName;
    const char* m_mp4FileName;
    FILE* m_mp3File;
    AudioOpreation m_audioOperation;

    bool m_forceStop;
};
