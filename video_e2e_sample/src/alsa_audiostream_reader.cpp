/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#include "alsa_audiostream_reader.h"


AlsaAudioStreamReader::AlsaAudioStreamReader(const char* captureHwName, const char* playbackHwName, const char* strMP4File, const char* strMP3File) :
    m_audio_thread(nullptr),
    m_pAlsaCapture(nullptr),
    m_pAlsaPlayback(nullptr),
    m_pLameMp3Encoder(nullptr),
    m_pAVMP4Muxer(nullptr),
    m_mp3FileName(nullptr),
    m_mp4FileName(nullptr),
    m_mp3File(nullptr),
    m_forceStop(false)
{
    m_audioOperation = { false };

    if (captureHwName != nullptr && captureHwName[0] != '\0')
    {
        m_pAlsaCapture = new AlsaVcdDevice(captureHwName, SND_PCM_STREAM_CAPTURE);
    }
    if (playbackHwName != nullptr && playbackHwName[0] != '\0')
    {
        m_audioOperation.playback = true;
        m_pAlsaPlayback = new AlsaVcdDevice(playbackHwName, SND_PCM_STREAM_PLAYBACK);
    }
    if (strMP3File != nullptr && strMP3File[0] != '\0')
    {
        m_audioOperation.mp3Encode = true;
        m_audioOperation.mp3Save = true;
        m_mp3FileName = strMP3File;
    }
    if (strMP4File != nullptr && strMP4File[0] != '\0')
    {
        m_audioOperation.mp3Encode = true;
        m_audioOperation.mp4Mux = true;
        m_mp4FileName = strMP4File;
    }
    if (m_audioOperation.mp3Encode)
    {
        m_pLameMp3Encoder = new LameMp3Encoder();
    }
    if (m_audioOperation.mp4Mux)
    {
        m_pAVMP4Muxer = AVMP4Muxer::GetInstance();
    }
}

AlsaAudioStreamReader::~AlsaAudioStreamReader()
{
    if (m_pAlsaCapture != nullptr)
    {
        delete m_pAlsaCapture;
    }
    m_pAlsaCapture = nullptr;

    if (m_pAlsaPlayback != nullptr)
    {
        delete m_pAlsaPlayback;
    }
    m_pAlsaPlayback = nullptr;

    if (m_audio_thread != nullptr)
    {
        delete m_audio_thread;
    }
    m_audio_thread = nullptr;

    if (m_pLameMp3Encoder != nullptr)
    {
        delete m_pLameMp3Encoder;
    }
    m_pLameMp3Encoder = nullptr;

    if (m_mp3FileName != nullptr)
    {
        m_mp3FileName = nullptr;
    }

    if (m_mp4FileName != nullptr)
    {
        m_mp4FileName = nullptr;
    }

    if (m_mp3File != nullptr)
    {
        delete m_mp3File;
    }
    m_mp3File = nullptr;
    if (m_pAVMP4Muxer != nullptr)
    {
        m_pAVMP4Muxer = nullptr;
    }
}

mfxStatus AlsaAudioStreamReader::Init()
{
    int ret = 0;
    if (m_pAlsaCapture == nullptr)
    {
        printf("Error! Alsa Capture is not inited in AlsaAudioStreamReader::Init\n");
        return MFX_ERR_NOT_INITIALIZED;
    }
    ret = m_pAlsaCapture->Open();
    if (ret < 0)
    {
        return MFX_ERR_NOT_INITIALIZED;
    }
    ret = m_pAlsaCapture->SetHWParams(SND_PCM_ACCESS_RW_INTERLEAVED, SND_PCM_FORMAT_S16_LE, TranscodingSample::LT_VCD_Audio_Channel, TranscodingSample::LT_VCD_Audio_PCM_Sample_rate);
    if (ret < 0)
    {
        return MFX_ERR_NOT_INITIALIZED;
    }
    m_captureParam = m_pAlsaCapture->GetAudioParam();

    if (m_pAlsaPlayback != nullptr)
    {
        ret = m_pAlsaPlayback->Open();
        if(ret < 0)
        {
            return MFX_ERR_NOT_INITIALIZED;
        }
        ret = m_pAlsaPlayback->SetHWParams(SND_PCM_ACCESS_RW_INTERLEAVED, SND_PCM_FORMAT_S16_LE, TranscodingSample::LT_VCD_Audio_Channel, TranscodingSample::LT_VCD_Audio_PCM_Sample_rate, m_captureParam.chunk_size, m_captureParam.buffer_size);
        if (ret < 0)
        {
            return MFX_ERR_NOT_INITIALIZED;
        }
        m_playbackParam = m_pAlsaPlayback->GetAudioParam();
    }

    if (m_audioOperation.mp3Encode && m_pLameMp3Encoder != nullptr)
    {
        ret = m_pLameMp3Encoder->Init(TranscodingSample::LT_VCD_Audio_PCM_Sample_rate, TranscodingSample::LT_VCD_Audio_MP3_Sample_rate, TranscodingSample::LT_VCD_Audio_Channel, TranscodingSample::LT_VCD_Audio_MP3_Bitrate, -1);
        if (ret < 0)
        {
            return MFX_ERR_NOT_INITIALIZED;
        }
        if (m_audioOperation.mp3Save)
        {
            m_mp3File = fopen(m_mp3FileName, "wb");
        }
    }
    if (m_audioOperation.mp4Mux && m_pAVMP4Muxer != nullptr)
    {
       ret =  m_pAVMP4Muxer->InitAudio(m_mp4FileName);
       if (ret < 0)
       {
           return MFX_ERR_NOT_INITIALIZED;
       }
    }
    return MFX_ERR_NONE;
}
void AlsaAudioStreamReader::Stop()
{
    m_forceStop = true;
}

void AlsaAudioStreamReader::close()
{
    if (m_pAVMP4Muxer != nullptr)
    {
        m_pAVMP4Muxer->Finish();
    }
    if (m_mp3File != nullptr)
    {
        fclose(m_mp3File);
    }
    if (m_pAlsaCapture != nullptr)
    {
        m_pAlsaCapture->Close();
    }
    if (m_pAlsaPlayback != nullptr)
    {
        m_pAlsaPlayback->Close();
    }
    if (m_pLameMp3Encoder != nullptr)
    {
        m_pLameMp3Encoder->Close();
    }
}

void AlsaAudioStreamReader::audioProcess()
{
    unsigned char* audiobuf = (unsigned char*)malloc(sizeof(unsigned char) * m_captureParam.buffer_size);
    int frameCount = m_captureParam.buffer_size / (m_captureParam.bits_per_frame / 8);
    while (!m_forceStop && !TranscodingSample::GlobalValue::AppStop)
    {
        int readed = pcm_read(audiobuf, frameCount);
        if (readed < 0)
        {
            continue;
        }
        if (readed != frameCount)
        {
            //todo
        }
        if (m_audioOperation.playback)
        {
            pcm_write(audiobuf, readed);
        }
        if (m_audioOperation.mp3Encode)
        {
            if (m_pLameMp3Encoder != nullptr)
            {
                unsigned char* mp3_buffer = nullptr;
                int out_size = 0;
                m_pLameMp3Encoder->Encode(audiobuf, readed, &mp3_buffer, out_size, false);
                if (m_audioOperation.mp3Save && m_mp3File != nullptr)
                {
                    fwrite(mp3_buffer, 1, out_size, m_mp3File);
                }
                if (m_audioOperation.mp4Mux && m_pAVMP4Muxer != nullptr)
                {
                    m_pAVMP4Muxer->SendEncodedAudio(mp3_buffer, out_size, readed);
                }
                if (mp3_buffer != nullptr)
                {
                    delete mp3_buffer;
                    mp3_buffer = nullptr;
                }
            }
        }
        usleep(50);
    }
    free(audiobuf);
    audiobuf = nullptr;
    close();
}

void AlsaAudioStreamReader::StartAudioThread()
{
    m_audio_thread = new std::thread(AlsaPcmProcesser, this);
}

void AlsaAudioStreamReader::AlsaPcmProcesser(AlsaAudioStreamReader* ctx)
{
    ctx->audioProcess();
}

int AlsaAudioStreamReader::pcm_read(unsigned char* data, int rcount)
{
    int result;
    int readedCount = 0;
    int unreadCount = rcount;
    if (m_pAlsaCapture == nullptr)
    {
        printf("Error! AlsaCapture is nullptr in AlsaAudioStreamReader::pcm_read. \n");
        return -1;
    }
    while (unreadCount > 0 && !m_forceStop)
    {
        result = m_pAlsaCapture->ProcessData(data, unreadCount);
        if (result == -EAGAIN || (result >= 0 && result < unreadCount))
        {
            printf("Error Audio pcm read ProcessWait in AlsaAudioStreamReader::pcm_read -EAGAIN: %d \n", result);
            m_pAlsaCapture->ProcessWait();
        }
        else if (result == -EPIPE)
        {
            printf("Error Audio pcm read result in AlsaAudioStreamReader::pcm_read -EPIPE: %d \n", result);
            m_pAlsaCapture->PrintError(result);
            m_pAlsaCapture->XRun();
        }
        else if (result == -ESTRPIPE)
        {
            printf("Error Audio pcm read result in AlsaAudioStreamReader::pcm_read -ESTRPIPE: %d \n", result);
            m_pAlsaCapture->PrintError(result);
            //suspend();
        }
        else if (result < 0)
        {
            printf("Error in in AlsaAudioStreamReader::pcm_read Audio pcm read result : %d, You can run cmd like \"arecord -vD hw:0,0 -r 48000 -c 2 -f S16_LE file.wav -d20\" to check whether VCD audio input work fine. \n", result);
            return -1;
        }
        if (result > 0)
        {
            //printf("read result : %d \n", result);
            readedCount += result;
            unreadCount -= result;
            data += result * m_captureParam.bits_per_frame / 8;
        }
    }
    return readedCount;
}

int AlsaAudioStreamReader::pcm_write(unsigned char* data, int count)
{
    int result;
    int writtenCount = 0;
    int unWrittenCount = count;

    while (unWrittenCount > 0 && !m_forceStop)
    {
        result = m_pAlsaPlayback->ProcessData(data, unWrittenCount);
        if (result == -EAGAIN || (result >= 0 && result < unWrittenCount))
        {
            m_pAlsaPlayback->ProcessWait();
        }
        else if (result == -EPIPE)
        {
            printf("Error in AlsaAudioStreamReader::pcm_write, write result -EPIPE: %d \n", result);
            m_pAlsaPlayback->PrintError(result);
            m_pAlsaPlayback->XRun();
        }
        else if (result == -ESTRPIPE)
        {
            printf("Error in AlsaAudioStreamReader::pcm_write, write result -ESTRPIPE: %d \n", result);
            m_pAlsaPlayback->PrintError(result);
            //suspend();
        }
        else if (result < 0)
        {
            printf("Error in AlsaAudioStreamReader::pcm_write, write : %d \n", result);
            m_pAlsaPlayback->PrintError(result);
        }
        if (result > 0)
        {
            writtenCount += result;
            unWrittenCount -= result;
            data += result * m_playbackParam.bits_per_frame / 8;
        }
    }
    return writtenCount;
}