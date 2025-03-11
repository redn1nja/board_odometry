#pragma once

#include "Logger.h"
#include "ret_codes.h"
#include "VideoChannel.h"


#include <easymedia/rkmedia_api.h>

#define MAX_RGA_CHN (6)

namespace vision {

class RGA: public Logger, public VideoChannel {
public:
    RGA();
    ~RGA();
    rc_t GetSrcParamsFrom(VideoChannel &src);
    rc_t GetDstParamsFrom(VideoChannel &dst);
    rc_t SetDstParams(int dst_width, int dst_height, int dst_x, int dst_y, IMAGE_TYPE_E dst_format);
    rc_t Start();
    rc_t Stop();
    int GetWidth() { return rga_chn_attr.stImgOut.u32Width; };
    int GetHeight() { return rga_chn_attr.stImgOut.u32Height; };
    MPP_CHN_S* GetBindAttr() { return &bind_attr; };
    void* GetAttr() { return &rga_chn_attr; };

private:
    int GetNewChannelId();
    RGA_ATTR_S rga_chn_attr;
    MPP_CHN_S bind_attr;
    static bool channels[MAX_RGA_CHN];
    unsigned int curr_channed_id;
    bool is_running;
};

}