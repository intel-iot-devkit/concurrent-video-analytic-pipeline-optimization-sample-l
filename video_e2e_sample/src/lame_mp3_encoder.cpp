/******************************************************************************\
Copyright (c) 2005-2021, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/
#include "lame_mp3_encoder.h"

LameMp3Encoder::LameMp3Encoder()
    :m_glf(nullptr)
{
}
LameMp3Encoder::~LameMp3Encoder()
{
}

void LameMp3Encoder::Close()
{
    if (m_glf != nullptr)
    {
        lame_close(m_glf);
        m_glf = nullptr;
    }
}

void LameMp3Encoder::Encode(unsigned char* input_buffer, const int input_size, unsigned char** mp3_buffer, int& out_size, bool is_last)
{
    int mp3buf_size = 1.25 * input_size + 7200;//todo get_mp3_size(input_size);
    out_size = 0;
    *mp3_buffer = new unsigned char[mp3buf_size];
    if (is_last)
    {
        out_size = lame_encode_flush(m_glf, *mp3_buffer, mp3buf_size);
    }
    else
    {
        out_size = lame_encode_buffer_interleaved(m_glf, (short int*)input_buffer, input_size, *mp3_buffer,
            mp3buf_size);
    }
}

int LameMp3Encoder::Init(int inSampleRate, int outSampleRate, int outChannel, int outBitrate, int quality)
{
    if (m_glf != nullptr)
    {
        lame_close(m_glf);
        m_glf = nullptr;
    }
    m_glf = lame_init();
    if (outSampleRate != -1)
    {
        lame_set_in_samplerate(m_glf, inSampleRate);
    }
    if (outChannel != -1)
    {
        lame_set_num_channels(m_glf, outChannel);
    }
    if (outBitrate != -1)
    {
        lame_set_brate(m_glf, outBitrate);
    }
    if (outSampleRate != -1)
    {
        lame_set_out_samplerate(m_glf, outSampleRate);
    }
    if (quality != -1)
    {
        lame_set_quality(m_glf, quality);
    }
    int ret_code = lame_init_params(m_glf);
    if (ret_code < 0)
    {
        printf("Error! lame_init error:%d \n", ret_code);
        return -1;
    }
    return 0;
}
