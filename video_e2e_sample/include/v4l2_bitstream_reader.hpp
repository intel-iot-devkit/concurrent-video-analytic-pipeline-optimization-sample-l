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

#include "v4l2_vcd_device.hpp"
#include "mfx_samples_config.h"
#include "vm/strings_defs.h"
#include "sample_defs.h"
#include "sample_utils.h"
#include "vpl/mfxcommon.h"

class V4l2BitstreamReader : public CSmplBitstreamReader
{
public:
    V4l2BitstreamReader();
    ~V4l2BitstreamReader();

    mfxStatus Init(ELTDevicePort devicePort, const msdk_char* strVideoDeviceName, const msdk_char* strSubdeviceName);
    mfxStatus ReadNextFrame(mfxFrameSurface1* pSurface);
    void QueryFrameInfo(v4l2FrameInfo& frameInfo);
    void Close();

private:
    V4l2BitstreamReader(const V4l2BitstreamReader& obj);
    V4l2BitstreamReader& operator=(const V4l2BitstreamReader& obj);
    V4l2VcdDevice* m_pV4l2VcdDevice;
    //int m_frameWidth;
    //int m_frameHeight;
};
