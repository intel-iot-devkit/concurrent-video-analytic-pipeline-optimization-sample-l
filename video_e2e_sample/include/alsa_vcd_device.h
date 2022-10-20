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
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <signal.h>
#include <string>
#include <sys/time.h>

struct AlsaAudioParam
{
    snd_pcm_uframes_t chunk_size;//The frame count of the period
    snd_pcm_uframes_t buffer_size;
    int bits_per_sample;
    int bits_per_frame; //channel count * m_bitsPerSample
    int chunk_byte;// the byte count of the period() chunk_byte = chunk_size * m_alsaAudioParam.bits_per_sample * channel / 8
};

class AlsaVcdDevice
{
public:
    AlsaVcdDevice(std::string alsaHWName, snd_pcm_stream_t stream);
    ~AlsaVcdDevice();

    int Open();
    int Close();
    int SetHWParams(snd_pcm_access_t access, snd_pcm_format_t format, unsigned int channel, unsigned int rate);
    int SetHWParams(snd_pcm_access_t access, snd_pcm_format_t format, unsigned int channel, unsigned int rate, snd_pcm_uframes_t periodSizeNear, snd_pcm_uframes_t bufferSizeNear);
    AlsaAudioParam GetAudioParam();
    int ProcessData(unsigned char* buffer, snd_pcm_uframes_t size);
    void ProcessWait();

    void PrintError(int result);
    int XRun();

private:
    AlsaVcdDevice(const AlsaVcdDevice& obj);
    AlsaVcdDevice& operator=(const AlsaVcdDevice& obj);

    std::string m_alsaHWName;
    snd_pcm_stream_t m_stream;

    snd_pcm_hw_params_t* m_hwParams;
    snd_pcm_access_t m_access; // SND_PCM_ACCESS_RW_INTERLEAVED
    unsigned int m_rate;
    unsigned int m_channel;
    snd_pcm_t* m_audioHandle;

    snd_pcm_format_t m_pcmFormat; //SND_PCM_FORMAT_S16_LE

    AlsaAudioParam m_alsaAudioParam;

    int m_significantBitsPerSample;
};