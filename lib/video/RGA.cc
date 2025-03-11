#include "RGA.h"
#include "Camera.h"
#include "VideoOutput.h"

#define DEFAULT_RGA_BUFF_CNT    (3)

namespace vision {

bool RGA::channels[MAX_RGA_CHN] = {0};

RGA::RGA():
    Logger("RGA")
{
    curr_channed_id = GetNewChannelId();
    LogInfo("Creating RGA %d", curr_channed_id);

    rga_chn_attr = {};
    rga_chn_attr.bEnBufPool = RK_TRUE;
    rga_chn_attr.u16BufPoolCnt = DEFAULT_RGA_BUFF_CNT;
    rga_chn_attr.u16Rotaion = 0;
    rga_chn_attr.stImgIn.u32X = 0;
    rga_chn_attr.stImgIn.u32Y = 0;
    rga_chn_attr.stImgOut.u32X = 0;
    rga_chn_attr.stImgOut.u32Y = 0;

    bind_attr.enModId = RK_ID_RGA;
    bind_attr.s32DevId = 0;
    bind_attr.s32ChnId = curr_channed_id;
}

RGA::~RGA()
{
    LogInfo("Destroying RGA %d", curr_channed_id);
    int ret = RK_MPI_RGA_DestroyChn(curr_channed_id);
    if (ret) {
        LogError("Failed to destroy RGA channel");
    }
    channels[curr_channed_id] = 0;
}

rc_t RGA::GetSrcParamsFrom(VideoChannel &src)
{
    LogInfo("Getting source params from VideoChannel");
    void* attr = src.GetAttr();
    if (typeid(src) == typeid(Camera)) {
        VI_CHN_ATTR_S* vi_attr = (VI_CHN_ATTR_S*)attr;
        rga_chn_attr.stImgIn.u32Width = vi_attr->u32Width;
        rga_chn_attr.stImgIn.u32Height = vi_attr->u32Height;
        rga_chn_attr.stImgIn.u32HorStride = vi_attr->u32Width;
        rga_chn_attr.stImgIn.u32VirStride = vi_attr->u32Height;
        rga_chn_attr.stImgIn.imgType = vi_attr->enPixFmt;
    } else if (typeid(src) == typeid(VideoOutput)) {
        VO_CHN_ATTR_S* vo_attr = (VO_CHN_ATTR_S*)attr;
        rga_chn_attr.stImgIn.u32Width = vo_attr->stImgRect.u32Width;
        rga_chn_attr.stImgIn.u32Height = vo_attr->stImgRect.u32Height;
        rga_chn_attr.stImgIn.u32HorStride = vo_attr->stImgRect.u32Width;
        rga_chn_attr.stImgIn.u32VirStride = vo_attr->stImgRect.u32Height;
        rga_chn_attr.stImgIn.imgType = vo_attr->enImgType;
    } else {
        LogError("Unsupported source channel type");
        return RC_ERROR;
    }
    return RC_OK;
}

rc_t RGA::GetDstParamsFrom(VideoChannel &dst)
{
    LogInfo("Getting destination params from VideoChannel");
    void* attr = dst.GetAttr();
    if (typeid(dst) == typeid(Camera)) {
        VI_CHN_ATTR_S* vi_attr = (VI_CHN_ATTR_S*)attr;
        rga_chn_attr.stImgOut.u32Width = vi_attr->u32Width;
        rga_chn_attr.stImgOut.u32Height = vi_attr->u32Height;
        rga_chn_attr.stImgOut.u32HorStride = vi_attr->u32Width;
        rga_chn_attr.stImgOut.u32VirStride = vi_attr->u32Height;
        rga_chn_attr.stImgOut.imgType = vi_attr->enPixFmt;
    } else if (typeid(dst) == typeid(VideoOutput)) {
        VO_CHN_ATTR_S* vo_attr = (VO_CHN_ATTR_S*)attr;
        rga_chn_attr.stImgOut.u32Width = vo_attr->stImgRect.u32Width;
        rga_chn_attr.stImgOut.u32Height = vo_attr->stImgRect.u32Height;
        rga_chn_attr.stImgOut.u32HorStride = vo_attr->stImgRect.u32Width;
        rga_chn_attr.stImgOut.u32VirStride = vo_attr->stImgRect.u32Height;
        rga_chn_attr.stImgOut.imgType = vo_attr->enImgType;
    } else {
        LogError("Unsupported destination channel type");
        return RC_ERROR;
    }
    return RC_OK;
}


rc_t RGA::SetDstParams(int dst_width, int dst_height, int dst_x, int dst_y, IMAGE_TYPE_E dst_format)
{
    LogInfo("Setting destination params");
    rga_chn_attr.stImgOut.u32Width = dst_width;
    rga_chn_attr.stImgOut.u32Height = dst_height;
    rga_chn_attr.stImgOut.u32HorStride = dst_width;
    rga_chn_attr.stImgOut.u32VirStride = dst_height;
    rga_chn_attr.stImgOut.u32X = dst_x;
    rga_chn_attr.stImgOut.u32Y = dst_y;
    rga_chn_attr.stImgOut.imgType = dst_format;
    return RC_OK;
}

rc_t RGA::Start()
{
    LogInfo("Starting RGA %d", curr_channed_id);
    int ret = RK_MPI_RGA_CreateChn(curr_channed_id, &rga_chn_attr);
    if (ret) {
        LogError("Failed to create RGA channel");
        return RC_ERROR;
    }
    return RC_OK;
}

rc_t RGA::Stop()
{
    LogInfo("Stopping RGA %d", curr_channed_id);
    return RC_OK;
}

int RGA::GetNewChannelId()
{
    for (int i = 0; i < sizeof(channels)/sizeof(channels[0]); i++) {
        if (channels[i] == 0) {
            channels[i] = 1;
            return i;
        }
    }
    return -1;
}

}