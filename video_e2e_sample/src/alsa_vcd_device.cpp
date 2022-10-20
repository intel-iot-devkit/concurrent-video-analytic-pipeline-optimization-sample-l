/******************************************************************************\
Copyright (c) 2005-2021, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#include "alsa_vcd_device.h"

AlsaVcdDevice::AlsaVcdDevice(std::string alsaHWName, snd_pcm_stream_t stream)
{
    m_alsaHWName = alsaHWName;
    m_stream = stream;
    m_audioHandle = nullptr;
    m_hwParams = nullptr;
}

AlsaVcdDevice::~AlsaVcdDevice()
{
    m_audioHandle = nullptr;
    m_hwParams = nullptr;
}

int AlsaVcdDevice::Open()
{
    int err = 0;
    if (m_audioHandle != nullptr)
    {
        printf("Error occured in AlsaVcdDevice::Open, %s already has been opened \n", m_alsaHWName.c_str());
        return 0;
    }
    if ((err = snd_pcm_open(&m_audioHandle, m_alsaHWName.c_str(), m_stream, 0)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::Open, Cannot open audio device %s (%s)\n",
            m_alsaHWName.c_str(),
            snd_strerror(err));
        return err;
    }
    return 0;
}

int AlsaVcdDevice::Close()
{
    int err = 0;
    if (m_audioHandle != nullptr)
    {
        if ((err = snd_pcm_close(m_audioHandle)) < 0)
        {
            printf("Error occured in AlsaVcdDevice::Close, Cannot close audio device %s (%s)\n",
                m_alsaHWName.c_str(),
                snd_strerror(err));
            return err;
        }
    }
    m_audioHandle = 0;
    return 1;
}

int AlsaVcdDevice::SetHWParams(snd_pcm_access_t access, snd_pcm_format_t format, unsigned int channel, unsigned int rate)
{
    return SetHWParams(access, format, channel, rate, 0, 0);
}

int AlsaVcdDevice::SetHWParams(snd_pcm_access_t access, snd_pcm_format_t format, unsigned int channel, unsigned int rate, snd_pcm_uframes_t periodSizeNear, snd_pcm_uframes_t bufferSizeNear)
{
    int err = 0;
    if (m_audioHandle == nullptr)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, %s has not been opened yet, please open the audio device first \n", m_alsaHWName.c_str());
        return -1;
    }
    if ((err = snd_pcm_hw_params_malloc(&m_hwParams)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot allocate hardware parameter structure (%s)\n",
            snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_hw_params_any(m_audioHandle, m_hwParams)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot initialize hardware parameter structure (%s)\n",
            snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_hw_params_set_access(m_audioHandle, m_hwParams, access)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set access type (%s)\n",
            snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_hw_params_set_format(m_audioHandle, m_hwParams, format)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set sample format (%s)\n",
            snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_hw_params_set_rate_near(m_audioHandle, m_hwParams, &rate, 0)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set sample rate (%s)\n",
            snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_hw_params_set_channels(m_audioHandle, m_hwParams, channel)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set channel count (%s)\n",
            snd_strerror(err));
        return err;
    }

    m_access = access;
    m_pcmFormat = format;
    m_channel = channel;
    m_rate = rate;

    if (periodSizeNear != 0 || bufferSizeNear != 0)
    {
        if (periodSizeNear == 0)
        {
            periodSizeNear = bufferSizeNear / 4;
        }
        else if (bufferSizeNear == 0)
        {
            bufferSizeNear = periodSizeNear * 4;
        }

        if ((err = snd_pcm_hw_params_set_period_size_near(m_audioHandle, m_hwParams, &periodSizeNear, 0)) < 0)
        {
            printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set period size near (%s)\n", snd_strerror(err));
            return err;
        }
        if ((err = snd_pcm_hw_params_set_buffer_size_near(m_audioHandle, m_hwParams, &bufferSizeNear)) < 0)
        {
            printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set buffer size near (%s)\n", snd_strerror(err));
            return err;
        }
    }
    else
    {
        unsigned int bufferTime;
        unsigned int periodTime;
        if ((err = snd_pcm_hw_params_get_buffer_time_max(m_hwParams, &bufferTime, 0)) < 0)
        {
            printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot GET buffer time (%s)\n",
                snd_strerror(err));
            return err;
        }
        if (bufferTime > 500000)
            bufferTime = 500000;

        periodTime = bufferTime / 4;
        if ((err = snd_pcm_hw_params_set_period_time_near(m_audioHandle, m_hwParams, &periodTime, 0)) < 0)
        {
            printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set period time (%s)\n",
                snd_strerror(err));
            return err;
        }
        if ((err = snd_pcm_hw_params_set_buffer_time_near(m_audioHandle, m_hwParams, &bufferTime, 0)) < 0)
        {
            printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set buffer time (%s)\n",
                snd_strerror(err));
            return err;
        }
    }


    if ((err = snd_pcm_hw_params(m_audioHandle, m_hwParams)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot set parameters (%s)\n", snd_strerror(err));
        return err;
    }

    m_alsaAudioParam.bits_per_sample = snd_pcm_format_physical_width(format);
    m_significantBitsPerSample = snd_pcm_format_width(format);
    m_alsaAudioParam.bits_per_frame = m_alsaAudioParam.bits_per_sample * channel;
    snd_pcm_hw_params_get_period_size(m_hwParams, &m_alsaAudioParam.chunk_size, 0);//? period time?
    snd_pcm_hw_params_get_buffer_size(m_hwParams, &m_alsaAudioParam.buffer_size);
    m_alsaAudioParam.chunk_byte = m_alsaAudioParam.chunk_size * m_alsaAudioParam.bits_per_sample * channel / 8;
    if (m_alsaAudioParam.chunk_size == m_alsaAudioParam.buffer_size)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Can't use period equal to buffer size (%lu == %lu)",
            m_alsaAudioParam.chunk_size, m_alsaAudioParam.buffer_size);
        return -1;
    }

    snd_pcm_hw_params_free(m_hwParams);
    if ((err = snd_pcm_prepare(m_audioHandle)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::SetHWParams, Cannot prepare audio interface for use (%s)\n",
            snd_strerror(err));
        return err;
    }
    return err;
}


AlsaAudioParam AlsaVcdDevice::GetAudioParam()
{
    return m_alsaAudioParam;
}

int AlsaVcdDevice::ProcessData(unsigned char* buffer, snd_pcm_uframes_t size)
{
    snd_pcm_sframes_t frame = 0;
    if (m_stream == SND_PCM_STREAM_CAPTURE)
    {
        frame = snd_pcm_readi(m_audioHandle, buffer, size);
    }
    else
    {
        frame = snd_pcm_writei(m_audioHandle, buffer, size);
    }
    return frame;
}

void AlsaVcdDevice::ProcessWait()
{
    snd_pcm_wait(m_audioHandle, 100);
}

void AlsaVcdDevice::PrintError(int result)
{
    printf("Error Alsa: %s \n", snd_strerror(result));
}
int AlsaVcdDevice::XRun()
{
    snd_pcm_status_t* status;
    int res;

    snd_pcm_status_alloca(&status);
    if ((res = snd_pcm_status(m_audioHandle, status)) < 0)
    {
        printf("Error occured in AlsaVcdDevice::XRun, Status error: %s \n", snd_strerror(res));
        return -1;
    }
    if (snd_pcm_status_get_state(status) == SND_PCM_STATE_XRUN)
    {
        if ((res = snd_pcm_prepare(m_audioHandle)) < 0)
        {
            printf("Error occured in AlsaVcdDevice::XRun, Prepare error: %s \n", snd_strerror(res));
        }
        return 0;
    }
    if (snd_pcm_status_get_state(status) == SND_PCM_STATE_DRAINING)
    {
        if (m_stream == SND_PCM_STREAM_CAPTURE)
        {
            if ((res = snd_pcm_prepare(m_audioHandle)) < 0)
            {
                printf("Error occured in AlsaVcdDevice::XRun, (DRAINING): prepare error: %s \n", snd_strerror(res));
                return -1;
            }
            return 0;
        }
    }

    printf("Error occured in AlsaVcdDevice::XRun, read/write error, State = %s \n", snd_pcm_state_name(snd_pcm_status_get_state(status)));
    return -1;
}