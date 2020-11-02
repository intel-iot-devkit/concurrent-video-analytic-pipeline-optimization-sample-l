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

//Whenever reach the end of input file, start from from beginning again.
//If user doesn't want infinit playing in loop, user can add "-n number"
//to every line in par file
#define LOOP_INPUT 1
#define RTSP_RETRY_MAX 5
#define RTSP_QUEUE_MAX_SIZE 60
#define RTSP_ERROR_MAX_COUNT 10

#include <queue>
#include <thread>
#include <stddef.h>

#include "sample_queue.h"
#include "sample_utils.h"

#define RTSP_SUPPORT

#ifdef RTSP_SUPPORT
extern "C" {
    #include <libavformat/avformat.h>
    #include <libavformat/avio.h>
}
#endif

#ifdef RTSP_SUPPORT
typedef struct
{
    size_t size;
    void *data;
} PacketData;
#endif

class FileAndRTSPBitstreamReader : public CSmplBitstreamReader
{
public :

    enum RTSP_STATUS {RTSP_NOT_CONNECT, RTSP_CONNECTED, RTSP_PLAY, RTSP_PAUSE, RTSP_STOP };
    FileAndRTSPBitstreamReader();
    virtual ~FileAndRTSPBitstreamReader();

    //resets position to file begin
    virtual void      Reset();
    virtual void      Close();
    virtual mfxStatus Init(const msdk_char *strFileName);
    virtual mfxStatus ReadNextFrame(mfxBitstream *pBS);


#ifdef RTSP_SUPPORT
    RTSP_STATUS GetRTSPStatus() { return m_rtsp_status;};
    void CreateRtspDumpFile(const msdk_char * rtsp_file_name);
    static void RtspPacketReader(FileAndRTSPBitstreamReader *);

    static void RtspSaveToLocalFile(const msdk_char *rtsp_url, 
            const msdk_char *rtsp_file_name, unsigned int max_frame_count);
protected:

    void ClearRTSPQueue();
    void StartRTSPThread();

    /* RTSP streaming */
    bool m_bIsRTSP;
    RTSP_STATUS   m_rtsp_status; // whether to stop RTSP streaming
    AVFormatContext *m_context;
    int m_video_stream_index;
    SampleQueue<PacketData *> *m_packets;
    std::thread *m_rtsp_thread;
    size_t   m_rtsp_queue_size;

    /* Saving RTSP stream to local file */

    FILE *m_rtsp_file;
#endif
private:
    FileAndRTSPBitstreamReader(FileAndRTSPBitstreamReader const&);
    FileAndRTSPBitstreamReader& operator=(FileAndRTSPBitstreamReader const&);
};


