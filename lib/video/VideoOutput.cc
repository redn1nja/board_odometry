#include "VideoOutput.h"

#define DEFAULT_VO_CVBS_DEVICE_NAME "/dev/dri/card0"

namespace vision {

bool VideoOutput::channels[MAX_VO_CHN] = {0};

VideoOutput::VideoOutput(unsigned int width, unsigned int height, bool is_overlay, unsigned int zpos):
    Logger("VideoOutput")
{
    LogInfo("Creating VideoOutput");
    curr_channed_id = GetNewChannelId();

    vo_chn_attr = {};
    vo_chn_attr.pcDevNode = DEFAULT_VO_CVBS_DEVICE_NAME;
    vo_chn_attr.emPlaneType = is_overlay ? VO_PLANE_OVERLAY : VO_PLANE_PRIMARY;
    vo_chn_attr.enImgType = is_overlay ? IMAGE_TYPE_ARGB8888 : IMAGE_TYPE_RGB888;
    // vo_chn_attr.enImgType = IMAGE_TYPE_ARGB8888;
    vo_chn_attr.u16Zpos = zpos;
    vo_chn_attr.u16Fps = 0;
    vo_chn_attr.stImgRect.s32X = 0;
    vo_chn_attr.stImgRect.s32Y = 0;
    vo_chn_attr.stImgRect.u32Width = width;
    vo_chn_attr.stImgRect.u32Height = height;
    vo_chn_attr.stDispRect.s32X = 0;
    vo_chn_attr.stDispRect.s32Y = 0;
    vo_chn_attr.stDispRect.u32Width = width;
    vo_chn_attr.stDispRect.u32Height = height;
    vo_chn_attr.stDispRect.s32Y = 0;

    bind_attr.enModId = RK_ID_VO;
    bind_attr.s32DevId = 0;
    bind_attr.s32ChnId = curr_channed_id;
}

VideoOutput::~VideoOutput()
{
    LogInfo("Destroying VideoOutput");
    int ret = RK_MPI_VO_DestroyChn(curr_channed_id);
    if (ret) {
        LogError("Failed to destroy VO channel");
    }
    channels[curr_channed_id] = 0;
}

rc_t VideoOutput::Start()
{
    LogInfo("Starting VideoOutput");
    int ret = RK_MPI_VO_CreateChn(curr_channed_id, &vo_chn_attr);
    if (ret) {
        LogError("Failed to create VO channel");
        return RC_ERROR;
    }
    return RC_OK;
}

rc_t VideoOutput::Stop()
{
    LogInfo("Stopping VideoOutput");
    return RC_OK;
}

int VideoOutput::GetNewChannelId()
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