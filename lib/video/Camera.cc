#include "Camera.h"


#define DEFAULT_ISP_DEVICE_NAME "rkispp_scale0"
#define DEFAULT_ISP_BUFF_CNT    (3)

namespace vision {

bool Camera::channels[MAX_VI_CHN] = {0};

Camera::Camera(uint8_t cam_id, uint16_t width, uint16_t height, uint8_t fps):
    Logger("Camera"),
    isp(cam_id),
    camera_id(cam_id)
{
    // Initialize camera
    vi_chn_attr.pcVideoNode = DEFAULT_ISP_DEVICE_NAME;
    vi_chn_attr.u32BufCnt = DEFAULT_ISP_BUFF_CNT;
    vi_chn_attr.u32Width = width;
    vi_chn_attr.u32Height = height;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;
    vi_chn_attr.enBufType = VI_CHN_BUF_TYPE_MMAP;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;
    curr_channel_id = GetNewChannelId();

    bind_attr.enModId = RK_ID_VI;
    bind_attr.s32DevId = camera_id;
    bind_attr.s32ChnId = curr_channel_id;
}

Camera::~Camera() {
    Stop();
}

rc_t Camera::Start() 
{
    isp.Start();
    int ret = RK_MPI_VI_SetChnAttr(camera_id, curr_channel_id, &vi_chn_attr);
    if (ret) {
        LogError("Failed to set VI channel %d attributes", curr_channel_id);
        return RC_ERROR;
    }
    ret  = RK_MPI_VI_EnableChn(camera_id, curr_channel_id);
    if (ret) {
        LogError("Failed to enable VI channel");
        return RC_ERROR;
    }
    return RC_OK;
}

rc_t Camera::Stop() {
    int ret = RK_MPI_VI_DisableChn(camera_id, curr_channel_id);
    if (ret) {
        LogError("Failed to disable VI channel");
        return RC_ERROR;
    }
    return RC_OK;
}

int Camera::GetNewChannelId() {
    for (int i = 0; i < sizeof(channels)/sizeof(channels[0]); i++) {
        if (channels[i] == 0) {
            channels[i] = 1;
            return i;
        }
    }
    return -1;
}

}