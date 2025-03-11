#include "Overlay.h"
#include <string.h>

namespace vision {

Overlay::Overlay(unsigned int width, unsigned int height, unsigned int zpos):
    Logger("Overlay"),
    width(width),
    height(height),
    zpos(zpos)
{
    LogInfo("Creating Overlay");
    AllocateFrame();
}

Overlay::Overlay(VideoChannel &ch, unsigned int zpos):
    Overlay(ch.GetWidth(), ch.GetHeight(), zpos)
{}

Overlay::~Overlay()
{
    LogInfo("Destroying Overlay");
    buffer.reset();
}

rc_t Overlay::AllocateFrame()
{
    buffer = std::shared_ptr<uint8_t>(new uint8_t[GetFrameSize()], std::default_delete<uint8_t[]>());
    CleanFrame();
    
    return RC_OK;
}

void Overlay::CleanFrame()
{
    memset(GetFrameFd(), 0, GetFrameSize());
}

}