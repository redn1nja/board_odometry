#pragma once

#include <cstdint>

#include "ISP.h"
#include "VideoChannel.h"

#include <easymedia/rkmedia_api.h>

#define MAX_VI_CHN 16

namespace vision {

class Camera: public Logger, public VideoChannel {
public:
    Camera(uint8_t cam_id, uint16_t width, uint16_t height, uint8_t fps);
    ~Camera();
    rc_t Start();
    rc_t Stop();
    int GetWidth() { return vi_chn_attr.u32Width; };
    int GetHeight() { return vi_chn_attr.u32Height; };
    MPP_CHN_S* GetBindAttr() { return &bind_attr; };
    void* GetAttr() { return &vi_chn_attr; };
private:
    int GetNewChannelId();
    static bool channels[MAX_VI_CHN];
    unsigned int curr_channel_id;
    unsigned int camera_id;
    ISP isp;
    VI_CHN_ATTR_S vi_chn_attr;
    MPP_CHN_S bind_attr;

};

}