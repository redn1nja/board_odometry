#include "Display.h"

#include <string.h>

#include <easymedia/rkmedia_api.h>
#include <rga/im2d.h>
#include <rga/rga.h>

namespace vision {

rc_t Display::Init()
{
    LogDebug("Display init");
    int bytes_per_pixel = 4; // Assume that Display works always with BGRA_8888

    LogInfo("buffer width = %d, height = %d", m_destination->GetWidth(), m_destination->GetHeight());
    st_buffer_pool_param = {};
    st_buffer_pool_param.u32Cnt = 3;
    st_buffer_pool_param.u32Size = 0; // Automatic calculation using imgInfo internally
    st_buffer_pool_param.enMediaType = MB_TYPE_VIDEO;
    st_buffer_pool_param.bHardWare = RK_TRUE;
    st_buffer_pool_param.u16Flag = MB_FLAG_NOCACHED;
    st_buffer_pool_param.stImageInfo.enImgType = st_vo_attr.enImgType; // image type same with VO
    st_buffer_pool_param.stImageInfo.u32Width = m_destination->GetWidth();
    st_buffer_pool_param.stImageInfo.u32Height = m_destination->GetHeight();
    st_buffer_pool_param.stImageInfo.u32HorStride = m_destination->GetWidth();
    st_buffer_pool_param.stImageInfo.u32VerStride = m_destination->GetHeight();
    mbp_size = m_destination->GetHeight() * m_destination->GetWidth() * bytes_per_pixel;

    /** Media buffer poll for fast redraw image */
    mbp = RK_MPI_MB_POOL_Create(&st_buffer_pool_param);
    if (!mbp) {
        LogError("Create buffer pool for vo failed!");
        return RC_ERROR;
    }

    st_image_info.u32Width = m_destination->GetWidth();
    st_image_info.u32Height = m_destination->GetHeight();
    st_image_info.u32HorStride = m_destination->GetWidth();
    st_image_info.u32VerStride = m_destination->GetHeight();
    st_image_info.enImgType = st_vo_attr.enImgType;

    size_t aligned_size = (mbp_size + 15) & ~15;
    m_mixed_frame = std::make_unique<uint8_t[]>(aligned_size);
    if (!m_mixed_frame) {
        LogError("Failed to allocate mixed frame buffer");
        return RC_ERROR;
    }

    overlays.resize(MAX_OVERLAYS);

    return RC_OK;
}

rc_t Display::SetDestination(VideoOutput &destination)
{
    LogDebug("setDestination: VO:%d", destination.GetBindAttr()->s32ChnId);
    m_destination = std::make_shared<VideoOutput>(destination);
    st_vo_attr = *(destination.GetVoAttr());
    
    return RC_OK;
}

rc_t Display::ShowFrame(void* frame, size_t size)
{
    MEDIA_BUFFER mb = RK_MPI_MB_POOL_GetBuffer(mbp, RK_TRUE);
    RK_MPI_MB_BeginCPUAccess(mb, RK_FALSE);
    if (!mb) {
        LogError("WARN: BufferPool get null buffer...");
        return RC_ERROR;
    }

    mb = RK_MPI_MB_ConvertToImgBuffer(mb, &st_image_info);
    if (!mb) {
        LogError("convert to img buffer failed!");
        return RC_ERROR;
    }

    memcpy(RK_MPI_MB_GetPtr(mb), frame, size);

    RK_MPI_MB_SetSize(mb, size);

    RK_MPI_MB_EndCPUAccess(mb, RK_FALSE);

    int ret = RK_MPI_SYS_SendMediaBuffer(m_destination->GetBindAttr()->enModId, m_destination->GetBindAttr()->s32ChnId, mb);
    if (ret) {
        LogError("ERROR: RK_MPI_SYS_SendMediaBuffer to VO[%d] failed! ret=%d\n", m_destination->GetBindAttr()->s32ChnId, ret);
        RK_MPI_MB_ReleaseBuffer(mb);
        return RC_ERROR;
    }

    RK_MPI_MB_ReleaseBuffer(mb);
    return RC_OK;
}






}