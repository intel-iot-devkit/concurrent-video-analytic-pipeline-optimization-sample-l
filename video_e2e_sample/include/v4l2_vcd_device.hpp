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
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <string>
#include <sys/mman.h>

#include "vm/strings_defs.h"
#include "v4l2_vcd_pipeline.hpp"
//#include "sample_defs.h"
//#include "sample_utils.h"
#include <sys/syscall.h>



#define ARRAY_SIZE(a)	(sizeof(a)/sizeof((a)[0]))

#define VIDEO_FORMAT {{MEDIA_BUS_FMT_UYVY8_1X16, V4L2_PIX_FMT_UYVY, 1, "UYVY"},}

//This is the struct to discribe the user space point mapped from v4l2_buffer.
struct userMapBuffer
{
    unsigned int idx;
    unsigned int padding[VIDEO_MAX_PLANES];
    unsigned int size[VIDEO_MAX_PLANES];
    void* mem[VIDEO_MAX_PLANES];
};

//This is the struct to discribe the frame info for above layer.
struct v4l2FrameInfo
{
    //    unsigned int bitsPerPixel;
    unsigned int frameWidth;
    unsigned int frameHeight;
    unsigned int fourcc;
    unsigned int frameRate;
};

//This is the struct to discribe the format parsed from the busFmtCode;
struct video_format_info
{
    unsigned int busFmtCode;
    unsigned int fourcc;
    unsigned int nPlanes;
    std::string fourccName;
    //    unsigned int bitsPerPixel;
};


class V4l2VcdDevice
{
public:
    V4l2VcdDevice(ELTDevicePort devicePort, std::string strVideoDeviceName, std::string strSubdeviceName);
    ~V4l2VcdDevice();

    int Open();
    int Init();
    void QueryFrameInfo(v4l2FrameInfo& frameInfo);
    char* ReadFrame(int& frameSize);
    userMapBuffer* ReadFrame(v4l2_buffer& v4l2_buf);
    int ReadFrameDone(v4l2_buffer& v4l2_buf);
    int Uninit();
    int Close();

protected:

private:
    V4l2VcdDevice(const V4l2VcdDevice& obj);
    V4l2VcdDevice& operator=(const V4l2VcdDevice& obj);

    static video_format_info video_formats[];

    std::string m_videoDevName;
    std::string m_subdevName;
    ELTDevicePort m_eLTDevicePort;

    int m_fd = -1;
    bool m_bStreamOn;
    bool m_bIsMutliPlane;

    enum v4l2_buf_type m_v4l2BufType;
    enum v4l2_memory m_v4l2Memory;
    unsigned int m_buffersNum;
    struct userMapBuffer* m_usrBuffers;

    unsigned int m_frameWidth;
    unsigned int m_frameHeight;
    unsigned int m_planesNum;
    unsigned int m_busFmtCode;
    unsigned int m_fourcc;
    unsigned int m_bitsPerPixel;
    unsigned int m_field;
    unsigned int m_colorSpace;
    unsigned int m_fps;
    std::string m_fourccName;

    int startStreaming();
    int fillCap();
    video_format_info* getVideoFmtByBusFmt(unsigned int busFmt);
    int fillFormatFromSubdev();
    int setFormat();
    int prepareBuffers();
    int stopStreaming();
};