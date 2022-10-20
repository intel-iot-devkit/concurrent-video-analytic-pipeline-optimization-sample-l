/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#include "v4l2_bitstream_reader.hpp"

V4l2BitstreamReader::V4l2BitstreamReader()
    :CSmplBitstreamReader()
{
    m_pV4l2VcdDevice = nullptr;
}

mfxStatus V4l2BitstreamReader::Init(ELTDevicePort devicePort, const msdk_char* strVideoDeviceName, const msdk_char* strSubdeviceName)
{
    MSDK_CHECK_POINTER(strVideoDeviceName, MFX_ERR_NULL_PTR);
    MSDK_CHECK_POINTER(strSubdeviceName, MFX_ERR_NULL_PTR);

    m_pV4l2VcdDevice = new V4l2VcdDevice(devicePort, strVideoDeviceName, strSubdeviceName);
    if (m_pV4l2VcdDevice->Open() < 0)
    {
        return MFX_ERR_UNSUPPORTED;
    }
    if (m_pV4l2VcdDevice->Init() < 0)
    {
        return MFX_ERR_UNSUPPORTED;
    }
    return MFX_ERR_NONE;
}

mfxStatus V4l2BitstreamReader::ReadNextFrame(mfxFrameSurface1* pSurface)
{
    if (m_pV4l2VcdDevice != nullptr)
    {
        struct v4l2_buffer v4l2_buf;
        userMapBuffer* mapBuffer = m_pV4l2VcdDevice->ReadFrame(v4l2_buf);
        if (mapBuffer == nullptr)
        {
            printf("ReadFrame failed \n");//TODO
            return MFX_ERR_UNKNOWN;
        }
        //TODO Currently only consider UYVY format, will have more format supported in the future.
        MSDK_MEMCPY(pSurface->Data.U, (mfxU8*)(mapBuffer->mem[0]), mapBuffer->size[0]);
        pSurface->Data.Pitch = mapBuffer->size[0];
        mapBuffer = nullptr;
        m_pV4l2VcdDevice->ReadFrameDone(v4l2_buf);
    }
    return MFX_ERR_NONE;
}

void V4l2BitstreamReader::QueryFrameInfo(v4l2FrameInfo& frameInfo)
{
    if (m_pV4l2VcdDevice != nullptr)
    {
        m_pV4l2VcdDevice->QueryFrameInfo(frameInfo);
        //m_frameWidth = frameInfo.frameWidth;
        //m_frameHeight = frameInfo.frameHeight;
    }
}

void V4l2BitstreamReader::Close()
{
    if (m_pV4l2VcdDevice != nullptr)
    {
        m_pV4l2VcdDevice->Uninit();
        m_pV4l2VcdDevice->Close();
        delete m_pV4l2VcdDevice;
        m_pV4l2VcdDevice = nullptr;
    }
}

V4l2BitstreamReader::~V4l2BitstreamReader()
{
    if (m_pV4l2VcdDevice != nullptr)
    {
        delete m_pV4l2VcdDevice;
    }
    m_pV4l2VcdDevice = nullptr;
}
