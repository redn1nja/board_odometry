#pragma once

#include <thread>
#include <unordered_map>
#include <memory>

#include "types.h"
#include "Overlay.h"
#include "Logger.h"
#include "VideoOutput.h"

#define MAX_OVERLAYS 16
namespace vision {

class Display : public Logger {
public:
    Display():
        Logger("Display")
    {
    }
    virtual ~Display() {}
    rc_t Init();
    rc_t SetDestination(VideoOutput &destination);
    void VideoStreamCb(void *frame, size_t size, uint64_t time_stamp, int is_iframe);
    rc_t ShowFrame(void* frame, size_t size);
    void Stop() {
        // if (stream != nullptr) {
        //     stream->StopFrameReceive();
        // }
    }
private:

    VO_CHN_ATTR_S st_vo_attr;
    MB_IMAGE_INFO_S st_image_info;
    MB_POOL_PARAM_S st_buffer_pool_param;
    MEDIA_BUFFER_POOL mbp;

    std::vector<std::shared_ptr<Overlay>> overlays;
    std::shared_ptr<VideoOutput> m_destination;
    std::unique_ptr<uint8_t[]> m_mixed_frame;
    int mbp_size;
    // std::thread 

};

}