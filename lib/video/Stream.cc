#include "Stream.h"

namespace vision {

static const char* GetModName(int mod_id)
{
    switch (mod_id)
    {
        case RK_ID_VI:
            return "VI";
        case RK_ID_VO:
            return "VO";
        case RK_ID_RGA:
            return "RGA";
        default:
            return "UNKNOWN";
    }
}

bool Stream::mpi_sys_init_done = false;

static void video_packet_cb(MEDIA_BUFFER mb, RK_VOID *pHandle)
{
    Stream* stream = (Stream*)pHandle;
    printf("video_packet_cb\n");
    RK_MPI_MB_BeginCPUAccess(mb, RK_FALSE);
    stream->ExecCb(RK_MPI_MB_GetPtr(mb), RK_MPI_MB_GetSize(mb), RK_MPI_MB_GetTimestamp(mb), RK_MPI_MB_IsViFrame(mb));
    RK_MPI_MB_EndCPUAccess(mb, RK_FALSE);
    RK_MPI_MB_ReleaseBuffer(mb);
}

Stream::Stream(std::string stream_name):
    Logger("Stream", stream_name),
    stream_name(stream_name)
{
    callback = nullptr;
    is_running = false;
    LogInfo("Stream created");
}

Stream::Stream(std::string stream_name, VideoChannel &source, VideoChannel &dest):
    Stream(stream_name)
{
    Bind(source, dest);
}

Stream::~Stream()
{
    LogInfo("Stream %s destroyed", stream_name.c_str());
    if (is_running) { 
        if (callback) {
            StopFrameReceive();
        } else {
            Unbind();
        }
    }
}

rc_t Stream::SetDestinationCb(VideoChannel &source, FrameCallback callback)
{
    this->callback = std::move(callback);
    source_chn = *source.GetBindAttr();
    if (source_chn.enModId == RK_ID_VENC) {
        LogInfo("Setting %s:%d:%d destination to callback",
                GetModName(source_chn.enModId), source_chn.s32DevId, source_chn.s32ChnId);

        int ret = RK_MPI_SYS_RegisterOutCbEx(&source_chn, video_packet_cb, this);
        if (ret != 0)
        {
            LogError("Failed to register callback for video packet");
            return RC_ERROR;
        }
    } else if (source_chn.enModId == RK_ID_RGA || source_chn.enModId == RK_ID_VI) {
        m_thread = std::thread(&Stream::processingThread, this, nullptr);
    } else {
        LogError("Unsupported source channel type");
        return RC_ERROR_NOT_SUPPORTED;
    }

    LogInfo("Callback set");
    is_running = true;
    return RC_OK;
}

rc_t Stream::ExecCb(void *frame, size_t size, uint64_t time_stamp, int is_iframe)
{
    callback(frame, size, time_stamp, is_iframe);
    return RC_OK;
}

rc_t Stream::Bind(VideoChannel &source, VideoChannel &dest)
{
    source_chn = *source.GetBindAttr();
    dest_chn = *dest.GetBindAttr();
    LogInfo("Binding %s:%d:%d and %s:%d:%d channels",
            GetModName(source_chn.enModId), source_chn.s32DevId, source_chn.s32ChnId,
            GetModName(dest_chn.enModId), dest_chn.s32DevId, dest_chn.s32ChnId);
    int ret = RK_MPI_SYS_Bind(&source_chn, &dest_chn);
    if (ret) {
        LogError("Failed to bind source and destination channels");
        return RC_ERROR;
    }
    is_running = true;
    return RC_OK;
}

rc_t Stream::Unbind()
{
    LogInfo("Stopping stream %s", stream_name.c_str());
    if (callback)
    {
        StopFrameReceive();
    } else {
        int ret = RK_MPI_SYS_UnBind(&source_chn, &dest_chn);
        if (ret) {
            LogError("Failed to disable VI channel");
            return RC_ERROR;
        }
    }
    is_running = false;
    return RC_OK;
}

void Stream::DumpChannels()
{
    RK_MPI_SYS_DumpChn(RK_ID_VI);
    RK_MPI_SYS_DumpChn(RK_ID_VO);
    RK_MPI_SYS_DumpChn(RK_ID_RGA);
}

void Stream::processingThread(void* arg)
{
    LogInfo("Processing thread started %d", is_running);
    MEDIA_BUFFER mb = nullptr;
    m_thread_running = true;
    while (is_running) {
        mb = RK_MPI_SYS_GetMediaBuffer(source_chn.enModId, source_chn.s32ChnId, 1000);
        if (!mb) {
            LogError("Failed to get media buffer");
            continue;
        }
        RK_MPI_MB_BeginCPUAccess(mb, RK_FALSE);
        ExecCb(RK_MPI_MB_GetPtr(mb), RK_MPI_MB_GetSize(mb), RK_MPI_MB_GetTimestamp(mb), RK_MPI_MB_IsViFrame(mb));
        RK_MPI_MB_EndCPUAccess(mb, RK_FALSE);
        RK_MPI_MB_ReleaseBuffer(mb);
    }
    LogInfo("Processing thread stopped");
    m_thread_running = false;
}

rc_t Stream::StartFrameReceive()
{
    LogInfo("Starting stream %s", stream_name.c_str());
    if (callback)
    {
        int ret = RK_MPI_SYS_StartGetMediaBuffer(source_chn.enModId, source_chn.s32ChnId);
        if (ret != 0)
        {
            LogError("Failed to start getting media buffer");
            return RC_ERROR;
        }
    }
    is_running = true;
    return RC_OK;
}

rc_t Stream::StopFrameReceive()
{
    LogInfo("Stopping stream %s", stream_name.c_str());
    if (callback)
    {
        is_running = false;
        if ( m_thread_running && m_thread.joinable()) {
            m_thread.join();
        }
        int ret = RK_MPI_SYS_StopGetMediaBuffer(source_chn.enModId, source_chn.s32ChnId);
        if (ret != 0)
        {
            LogError("Failed to stop getting media buffer");
            return RC_ERROR;
        }
    }
    
    return RC_OK;
}

}