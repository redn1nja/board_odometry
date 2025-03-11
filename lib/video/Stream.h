#pragma once

#include "Logger.h"
#include "ret_codes.h"
#include "VideoChannel.h"

#include <easymedia/rkmedia_api.h>

#include <functional>
#include <thread>

namespace vision {

using FrameCallback = std::function<void(void *frame, size_t size, uint64_t time_stamp, int is_iframe)>;

class Stream: public Logger
{
public:
    Stream(std::string stream_name="");
    Stream(std::string stream_name, VideoChannel &source, VideoChannel &dest);
    ~Stream();
    rc_t SetDestinationCb(VideoChannel &source, FrameCallback callback);
    rc_t StartFrameReceive();
    rc_t StopFrameReceive();
    rc_t Bind(VideoChannel &source, VideoChannel &dest);
    rc_t Unbind();
    rc_t ExecCb(void *frame, size_t size, uint64_t time_stamp, int is_iframe);
    void DumpChannels();
private:
    std::string stream_name;
    MPP_CHN_S source_chn;
    MPP_CHN_S dest_chn;
    FrameCallback callback;
    bool is_running;
    // bool is_cb_set;
    std::thread m_thread;
    bool m_thread_running;
    void processingThread(void* arg);
    static bool mpi_sys_init_done;
};

}