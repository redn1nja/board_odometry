#pragma once

#include "Logger.h"
#include "ret_codes.h"
#include "VideoChannel.h"

#include <easymedia/rkmedia_api.h>

#define MAX_VO_CHN 3

namespace vision {

class VideoOutput: public Logger, public VideoChannel {
public:
    VideoOutput(unsigned int width, unsigned int height, bool is_overlay=false, unsigned int zpos=0);
    ~VideoOutput();
    rc_t Start();
    rc_t Stop();
    MPP_CHN_S* GetBindAttr() { return &bind_attr; };
    void* GetAttr() { return &vo_chn_attr; };
    int GetWidth() { return vo_chn_attr.stDispRect.u32Width; };
    int GetHeight() { return vo_chn_attr.stDispRect.u32Height; };
    IMAGE_TYPE_E GetImgType() { return vo_chn_attr.enImgType; };
    VO_CHN_ATTR_S* GetVoAttr() { return &vo_chn_attr; };
private:
    int GetNewChannelId();
    VO_CHN_ATTR_S vo_chn_attr;
    MPP_CHN_S bind_attr;
    static bool channels[MAX_VO_CHN];
    unsigned int curr_channed_id;
    bool is_running;
};

}