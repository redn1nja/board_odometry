#pragma once

#include <memory>

#include "types.h"
#include "Logger.h"
#include "VideoChannel.h"

namespace vision {

class Overlay: private virtual Logger {
public:
    Overlay(unsigned int width, unsigned int height, unsigned int zpos=0);
    Overlay(VideoChannel &ch, unsigned int zpos=0);
    ~Overlay();
    
    int GetWidth() { return width; };
    int GetHeight() { return height; };
    int GetZpos() { return zpos; };
    void CleanFrame();
    void* GetFrameFd() { return buffer.get(); };
    int GetFrameSize() { return width * height * 4; };
protected:
    rc_t AllocateFrame();
    int width;
    int height;
    int zpos;
    std::shared_ptr<uint8_t> buffer;
};

}