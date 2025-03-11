#pragma once

#include "ret_codes.h"

#include <easymedia/rkmedia_api.h>

namespace vision {
class VideoChannel {
    public:
        VideoChannel() = default;
        virtual ~VideoChannel() = default;
        virtual rc_t Start() = 0;
        virtual rc_t Stop() = 0;
        virtual int GetWidth() = 0;
        virtual int GetHeight() = 0;
        virtual MPP_CHN_S* GetBindAttr() = 0;
        virtual void* GetAttr() = 0;
};

}