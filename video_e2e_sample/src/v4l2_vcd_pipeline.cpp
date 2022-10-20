/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/
#include "v4l2_vcd_pipeline.hpp"

bool V4l2VcdPipeline::m_bIsReset = false;
std::mutex V4l2VcdPipeline::m_mutex;

int V4l2VcdPipeline::Reset()
{
    if (system(SHELL_MEDIA_CTL_RESET) < 0)
    {
        printf("Error in V4l2VcdPipeline::Reset, Reset pipeline failed!\n");
        return -1;
    }
    return 0;
}
int V4l2VcdPipeline::Setup(const ELTDevicePort device, std::string format, unsigned int width, unsigned int height)
{
    std::unique_lock<std::mutex> ulock(m_mutex);
    if (!m_bIsReset)
    {
        Reset();
        m_bIsReset = true;
    }
    ulock.unlock();
    std::string fmt = format + "/" + std::to_string(width) + "x" + std::to_string(height);
    switch (device)
    {
        case LT_MIPI_DUAL_1:
        {
            if (system(LT_A_MEDIA_CTL_SET_FMT_SUBDEV(fmt).c_str()) < 0 ||
                system(LT_A_MEDIA_CTL_SET_FMT_IPU(fmt).c_str()) < 0 ||
                system(LT_A_MEDIA_CTL_SET_FMT_SOC(fmt).c_str()) < 0 ||
                system(LT_A_MEDIA_CTL_LINK_SUBDEV) < 0 ||
                system(LT_A_MEDIA_CTL_LINK_IPU) < 0 ||
                system(LT_A_MEDIA_CTL_LINK_SOC) < 0
                )
            {
                printf("Error in V4l2VcdPipeline::Setup, Setup LT6911UXC_TWO_MIPIS_Primary pipeline failed \n");
                return -1;
            }
            break;
        }
        case LT_MIPI_DUAL_2:
        {
            if (system(LT_C_MEDIA_CTL_SET_FMT_SUBDEV(fmt).c_str()) < 0 ||
                system(LT_C_MEDIA_CTL_SET_FMT_IPU(fmt).c_str()) < 0 ||
                system(LT_C_MEDIA_CTL_SET_FMT_SOC(fmt).c_str()) < 0 ||
                system(LT_C_MEDIA_CTL_LINK_SUBDEV) < 0 ||
                system(LT_C_MEDIA_CTL_LINK_IPU) < 0 ||
                system(LT_C_MEDIA_CTL_LINK_SOC) < 0
                )
            {
                printf("Error in V4l2VcdPipeline::Setup, Setup v4l2 media ctl pipeline failed \n");
                return -1;
            }
            break;
        }
        case LT_MIPI_SINGLE:
        {
            if (system(LT_B_MEDIA_CTL_SET_FMT_SUBDEV(fmt).c_str()) < 0 ||
                system(LT_B_MEDIA_CTL_SET_FMT_IPU(fmt).c_str()) < 0 ||
                system(LT_B_MEDIA_CTL_SET_FMT_SOC(fmt).c_str()) < 0 ||
                system(LT_B_MEDIA_CTL_LINK_SUBDEV) < 0 ||
                system(LT_B_MEDIA_CTL_LINK_IPU) < 0 ||
                system(LT_B_MEDIA_CTL_LINK_SOC) < 0
                )
            {
                printf("Error in V4l2VcdPipeline::Setup, Setup v4l2 media ctl pipeline failed \n");
                return -1;
            }
            break;
        }
    }
    //printf(SHELL_MEDIA_CTL_RESET);
    //printf("\n");
    //printf(SHELL_MEDIA_CTL_SET_FMT_SUBDEV(fmt).c_str());
    //printf("\n");
    //printf(SHELL_MEDIA_CTL_SET_FMT_IPU(fmt).c_str());
    //printf("\n");
    //printf(SHELL_MEDIA_CTL_SET_FMT_SOC(fmt).c_str());
    //printf("\n");
    //printf(SHELL_MEDIA_CTL_LINK_SUBDEV);
    //printf("\n");
    //printf(SHELL_MEDIA_CTL_LINK_IPU);
    //printf("\n");
    //printf(SHELL_MEDIA_CTL_LINK_SOC);
    //printf("\n");
    return 0;
}
