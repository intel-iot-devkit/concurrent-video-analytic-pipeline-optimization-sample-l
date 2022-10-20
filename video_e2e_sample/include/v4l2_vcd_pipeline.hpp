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

#include <string>
#include <stdlib.h>
#include <mutex>

#define SHELL_MEDIA_CTL_RESET    "sudo media-ctl -r"
//lt6911uxc a
#define LT_A_MEDIA_CTL_SET_FMT_SUBDEV(sfmt)    ("sudo media-ctl -v -V \"'lt6911uxc a':0 [fmt:"+sfmt+"]\"")
#define LT_A_MEDIA_CTL_SET_FMT_IPU(sfmt)    ("sudo media-ctl -v -V \"'Intel IPU6 CSI-2 1':0 [fmt:"+sfmt+"]\"")
#define LT_A_MEDIA_CTL_SET_FMT_SOC(sfmt)    ("sudo media-ctl -v -V \"'Intel IPU6 CSI2 BE SOC':0 [fmt:"+sfmt+"]\"")

#define LT_A_MEDIA_CTL_LINK_SUBDEV    "sudo media-ctl -v -l \"'lt6911uxc a':0 -> 'Intel IPU6 CSI-2 1':0[1]\""
#define LT_A_MEDIA_CTL_LINK_IPU    "sudo media-ctl -v -l \"'Intel IPU6 CSI-2 1':1 -> 'Intel IPU6 CSI2 BE SOC':0[5]\""
#define LT_A_MEDIA_CTL_LINK_SOC    "sudo media-ctl -v -l \"'Intel IPU6 CSI2 BE SOC':16 -> 'Intel IPU6 BE SOC capture 0':0[5]\""


//lt6911uxc c
#define LT_C_MEDIA_CTL_SET_FMT_SUBDEV(sfmt)    ("sudo media-ctl -v -V \"'lt6911uxc c':0 [fmt:"+sfmt+"]\"")
#define LT_C_MEDIA_CTL_SET_FMT_IPU(sfmt)    ("sudo media-ctl -v -V \"'Intel IPU6 CSI-2 5':0 [fmt:"+sfmt+"]\"")
#define LT_C_MEDIA_CTL_SET_FMT_SOC(sfmt)    ("sudo media-ctl -v -V \"'Intel IPU6 CSI2 BE SOC':3 [fmt:"+sfmt+"]\"")

#define LT_C_MEDIA_CTL_LINK_SUBDEV    "sudo media-ctl -v -l \"'lt6911uxc c':0 -> 'Intel IPU6 CSI-2 5':0[1]\""
#define LT_C_MEDIA_CTL_LINK_IPU    "sudo media-ctl -v -l \"'Intel IPU6 CSI-2 5':1 -> 'Intel IPU6 CSI2 BE SOC':3[5]\""
#define LT_C_MEDIA_CTL_LINK_SOC    "sudo media-ctl -v -l \"'Intel IPU6 CSI2 BE SOC':19 -> 'Intel IPU6 BE SOC capture 3':0[5]\""

//lt6911uxc b
#define LT_B_MEDIA_CTL_SET_FMT_SUBDEV(sfmt)    ("sudo media-ctl -v -V \"'lt6911uxc b':0 [fmt:"+sfmt+"]\"")
#define LT_B_MEDIA_CTL_SET_FMT_IPU(sfmt)    ("sudo media-ctl -v -V \"'Intel IPU6 CSI-2 2':0 [fmt:"+sfmt+"]\"")
#define LT_B_MEDIA_CTL_SET_FMT_SOC(sfmt)    ("sudo media-ctl -v -V \"'Intel IPU6 CSI2 BE SOC':1 [fmt:"+sfmt+"]\"")

#define LT_B_MEDIA_CTL_LINK_SUBDEV    "sudo media-ctl -v -l \"'lt6911uxc b':0 -> 'Intel IPU6 CSI-2 2':0[1]\""
#define LT_B_MEDIA_CTL_LINK_IPU    "sudo media-ctl -v -l \"'Intel IPU6 CSI-2 2':1 -> 'Intel IPU6 CSI2 BE SOC':1[5]\""
#define LT_B_MEDIA_CTL_LINK_SOC    "sudo media-ctl -v -l \"'Intel IPU6 CSI2 BE SOC':17 -> 'Intel IPU6 BE SOC capture 1':0[5]\""

enum ELTDevicePort
{
    LT_MIPI_DUAL_1,
    LT_MIPI_DUAL_2,
    LT_MIPI_SINGLE
};

class V4l2VcdPipeline
{
public:
    static std::mutex m_mutex;
    static bool m_bIsReset;
    static int Reset();
    static int Setup(const ELTDevicePort ltDevice, std::string format, unsigned int width, unsigned int height);
};