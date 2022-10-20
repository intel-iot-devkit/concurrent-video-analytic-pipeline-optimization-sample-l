/******************************************************************************\
Copyright (c) 2005-2020, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\**********************************************************************************/

#include "v4l2_vcd_device.hpp"


video_format_info V4l2VcdDevice::video_formats[] = VIDEO_FORMAT;

V4l2VcdDevice::V4l2VcdDevice(ELTDevicePort devicePort, std::string strVideoDeviceName, std::string strSubdeviceName)
    :m_fd(-1),
    m_bStreamOn(false),
    m_bIsMutliPlane(true),
    m_v4l2BufType(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE),
    m_v4l2Memory(V4L2_MEMORY_MMAP),
    m_buffersNum(2),
    m_usrBuffers(nullptr),
    m_planesNum(1),
    m_busFmtCode(MEDIA_BUS_FMT_UYVY8_1X16)
{
    m_videoDevName = strVideoDeviceName;
    m_subdevName = strSubdeviceName;
    m_eLTDevicePort = devicePort;
}

int V4l2VcdDevice::Open()
{
    if (m_fd > 0)
    {
        printf("%s already has been opened \n", m_videoDevName.c_str());
        return 0;
    }
    m_fd = open(m_videoDevName.c_str(), O_RDWR);
    if (m_fd <= 0)
    {
        printf("open %s error \n", m_videoDevName.c_str());
        return -1;
    }
    return 0;
}

int V4l2VcdDevice::Init()
{

    if (m_fd <= 0)
    {
        printf("The video device is not open. \n");
        return -1;
    }
    if (fillCap() < 0 ||
        fillFormatFromSubdev() < 0 ||
        V4l2VcdPipeline::Setup(m_eLTDevicePort, m_fourccName, m_frameWidth, m_frameHeight) < 0 ||
        setFormat() < 0 ||
        prepareBuffers() < 0 ||
        startStreaming() < 0)
    {
        return -1;
    }
    return 0;
}

int V4l2VcdDevice::startStreaming()
{
    enum v4l2_buf_type type = m_v4l2BufType;
    if (ioctl(m_fd, VIDIOC_STREAMON, &type) < 0)
    {
        printf("Unable to start streaming: %s (%d).\n", strerror(errno), errno);
        return -1;
    }
    m_bStreamOn = true;
    return 0;
}

userMapBuffer* V4l2VcdDevice::ReadFrame(v4l2_buffer& v4l2_buf)
{
    if (!m_bStreamOn)
    {
        printf("m_bStreamOn is false \n");
        return nullptr;
    }
    struct v4l2_plane planes;
    memset(&planes, 0, sizeof(v4l2_plane));
    memset(&v4l2_buf, 0, sizeof(v4l2_buffer));

    v4l2_buf.type = m_v4l2BufType;
    v4l2_buf.memory = m_v4l2Memory;
    v4l2_buf.m.planes = &planes;
    v4l2_buf.length = m_planesNum;
    if (ioctl(m_fd, VIDIOC_DQBUF, &v4l2_buf) < 0)
    {
        printf("VIDIOC_DQBUF failed, dropped frame: %s (%d).\n", strerror(errno), errno);
        return nullptr;
    }
    return &m_usrBuffers[v4l2_buf.index];
}


int V4l2VcdDevice::ReadFrameDone(v4l2_buffer& v4l2_buf)
{
    if (ioctl(m_fd, VIDIOC_QBUF, &v4l2_buf) < 0)
    {
        printf("VIDIOC_QBUF failed, %s (%d).\n", strerror(errno), errno);
        return -1;
    }
    return 0;
}

int V4l2VcdDevice::stopStreaming()
{
    if (!m_bStreamOn || m_fd <= 0)
    {
        m_bStreamOn = false;
        return 0;
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(m_fd, VIDIOC_STREAMOFF, &type) < 0)
    {
        printf("VIDIOC_STREAMOFF failed, %s (%d).\n", strerror(errno), errno);
        return -1;
    }
    m_bStreamOn = false;
    return 0;
}

int V4l2VcdDevice::Uninit()
{
    stopStreaming();
    if (m_usrBuffers == nullptr)
    {
        return 0;
    }
    for (unsigned int i = 0; i < m_buffersNum; i++)
    {
        for (unsigned int j = 0; j < m_planesNum; j++)
        {
            int ret = munmap(m_usrBuffers[i].mem[j], m_usrBuffers[i].size[j]);
            if (ret < 0)
            {
                printf("munmap failed\n");
                return -1;
            }
        }
    }
    free(m_usrBuffers);
    m_usrBuffers = nullptr;
    return 0;
}

int V4l2VcdDevice::Close()
{
    if (m_fd > 0)
    {
        close(m_fd);
    }
    m_fd = -1;
    return 0;
}

void V4l2VcdDevice::QueryFrameInfo(v4l2FrameInfo& frameInfo)
{
    memset(&frameInfo, 0, sizeof frameInfo);
    //   frameInfo.bitsPerPixel = m_bitsPerPixel;
    frameInfo.frameWidth = m_frameWidth;
    frameInfo.frameHeight = m_frameHeight;
    frameInfo.fourcc = m_fourcc;
    frameInfo.frameRate = m_fps;
}

int V4l2VcdDevice::setFormat()
{
    struct v4l2_format fmt;
    unsigned int i;
    int ret;
    memset(&fmt, 0, sizeof fmt);
    fmt.type = m_v4l2BufType;
    if (m_bIsMutliPlane)
    {
        fmt.fmt.pix_mp.width = m_frameWidth;
        fmt.fmt.pix_mp.height = m_frameHeight;
        fmt.fmt.pix_mp.pixelformat = m_fourcc;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = m_planesNum;
        for (i = 0; i < fmt.fmt.pix_mp.num_planes; i++)
        {
            //fmt.fmt.pix_mp.plane_fmt[i].bytesperline = m_frameWidth * m_bitsPerPixel/8;
            //fmt.fmt.pix_mp.plane_fmt[i].sizeimage = m_frameWidth * m_frameHeight * m_bitsPerPixel /8;
        }
    }
    else
    {
        fmt.fmt.pix.width = m_frameWidth;
        fmt.fmt.pix.height = m_frameHeight;
        fmt.fmt.pix.pixelformat = m_fourcc;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        //        fmt.fmt.pix.bytesperline = m_frameWidth * m_bitsPerPixel;
        //        fmt.fmt.pix.sizeimage = m_frameWidth * m_frameHeight * m_bitsPerPixel;
        fmt.fmt.pix.priv = V4L2_PIX_FMT_PRIV_MAGIC;
    }

    if (ioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        printf("Unable to set format VIDIOC_S_FMT: %s (%d).\n", strerror(errno),
            errno);
    }
    return 0;
}

V4l2VcdDevice::~V4l2VcdDevice()
{
    if (m_fd > 0)
    {
        Uninit();
        Close();
    }
    if (m_usrBuffers != nullptr)
    {
        free(m_usrBuffers);
    }
    m_usrBuffers = nullptr;
}

int V4l2VcdDevice::fillCap()
{
    if (m_fd <= 0)
    {
        printf("The video device has not been opened \n");
        return -1;
    }
    v4l2_capability cap;
    memset(&cap, 0, sizeof cap);
    if (ioctl(m_fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        printf("VIDIOC_QUERYCAP failed, %s (%d).\n", strerror(errno), errno);
        return -1;
    }
    //printf("driver : %s\n", cap.driver);
    //printf("driver name : %s\n", cap.card);
    //printf("bus info : %s\n", cap.bus_info);
    //printf("driver version : %d\n", cap.version);

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    {
        m_v4l2BufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        m_bIsMutliPlane = false;
    }
    else if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
    {
        m_v4l2BufType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        m_bIsMutliPlane = true;
    }
    else
    {
        printf("Error, Can not support VIDEO CAPTURE CAP\n");
        return -1;
    }
    return 0;
}
int V4l2VcdDevice::fillFormatFromSubdev()
{
    v4l2_subdev_format format;
    memset(&format, 0, sizeof(format));

    v4l2_subdev_frame_interval frame_interval;
    memset(&frame_interval, 0, sizeof(frame_interval));

    int m_videoSubDevFD = open(m_subdevName.c_str(), O_RDWR);
    if (m_videoSubDevFD < 0)
    {
        printf("open %s error\n", m_subdevName.c_str());
        return -1;
    }
    if (ioctl(m_videoSubDevFD, VIDIOC_SUBDEV_G_FRAME_INTERVAL, &frame_interval) < 0)
    {
        printf("VIDIOC_SUBDEV_G_FRAME_INTERVAL failed, %s (%d).\n", strerror(errno), errno);
        m_fps = 30;
    }
    else
    {
        m_fps = frame_interval.interval.denominator / frame_interval.interval.numerator;
    }

    if (ioctl(m_videoSubDevFD, VIDIOC_SUBDEV_G_FMT, &format) < 0)
    {
        printf("VIDIOC_SUBDEV_G_FMT failed, %s (%d).\n", strerror(errno), errno);
        close(m_videoSubDevFD);
        return -1;
    }
    m_frameWidth = format.format.width;
    m_frameHeight = format.format.height;
    m_busFmtCode = format.format.code;
    //    m_field = format.format.field;
    //    m_colorSpace = format.format.colorspace;
    video_format_info* format_info = getVideoFmtByBusFmt(m_busFmtCode);
    if (format_info == nullptr)
    {
        printf("The media bus format code is not supported\n");
        close(m_videoSubDevFD);
        return -1;
    }
    m_fourcc = format_info->fourcc;
    m_planesNum = format_info->nPlanes;
    m_fourccName = format_info->fourccName;
    format_info = nullptr;
    close(m_videoSubDevFD);
    return 0;
}

video_format_info* V4l2VcdDevice::getVideoFmtByBusFmt(unsigned int busFmt)
{
    for (unsigned int i = 0; i < ARRAY_SIZE(video_formats); i++)
    {
        if (video_formats[i].busFmtCode == busFmt)
        {
            return &video_formats[i];
        }
    }
    return nullptr;
}

int V4l2VcdDevice::prepareBuffers()
{
    //request kernel buffer
    v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = m_buffersNum;
    req.type = m_v4l2BufType;
    req.memory = m_v4l2Memory;
    if (ioctl(m_fd, VIDIOC_REQBUFS, &req) < 0)
    {
        printf("VIDIOC_REQBUFS failed, %s (%d).\n", strerror(errno), errno);
        return -1;
    }

    m_usrBuffers = (userMapBuffer*)calloc(m_buffersNum, sizeof(userMapBuffer));

    if (!m_usrBuffers)
    {
        printf("calloc user frame buffers error : Out of memory \n");
        return -1;
    }

    v4l2_buffer v4l2_buf;
    struct v4l2_plane planes;
    for (unsigned int i = 0; i < m_buffersNum; i++)
    {
        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(&planes, 0, sizeof(planes));
        v4l2_buf.type = m_v4l2BufType;
        v4l2_buf.memory = m_v4l2Memory;
        v4l2_buf.m.planes = &planes;
        v4l2_buf.length = m_planesNum;
        v4l2_buf.index = i;

        if (ioctl(m_fd, VIDIOC_QUERYBUF, &v4l2_buf) < 0)
        {
            printf("VIDIOC_QUERYBUF failed, %s (%d).\n", strerror(errno), errno);
            return -1;
        }

        m_usrBuffers[i].idx = i;
        switch (m_v4l2Memory)
        {
        case V4L2_MEMORY_MMAP:
        {
            unsigned int length;
            unsigned int offset;
            for (unsigned int j = 0; j < m_planesNum; j++)
            {
                if (m_bIsMutliPlane)
                {
                    length = v4l2_buf.m.planes[j].length;
                    offset = v4l2_buf.m.planes[j].m.mem_offset;
                }
                else
                {
                    length = v4l2_buf.length;
                    offset = v4l2_buf.m.offset;
                }
                m_usrBuffers[i].mem[j] = mmap(0, length, PROT_READ | PROT_WRITE, MAP_SHARED,
                    m_fd, offset);
                if (m_usrBuffers[i].mem[j] == MAP_FAILED)
                {
                    printf("Unable to map buffer %u/%u: %s (%d)\n",
                        m_usrBuffers[i].idx, j, strerror(errno), errno);
                    return -1;
                }
                m_usrBuffers[i].size[j] = length; //Issue: the size is one height bigger than the image size calculated.
            }
            break;
        }
        default:
        {
            printf("The memory type is not implemented yet \n");
            return -1;
        }
        }
        if (ioctl(m_fd, VIDIOC_QBUF, &v4l2_buf) < 0)
        {
            printf("VIDIOC_QBUF failed, %s (%d).\n", strerror(errno), errno);
            return -1;
        }
    }
    return 0;
}
