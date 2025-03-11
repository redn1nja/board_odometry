#pragma once

#include <memory>

#include "types.h"
#include "Logger.h"
#include "Overlay.h"

#include <opencv2/core.hpp>

namespace vision {

typedef enum Color {
    BLACK = 0x000000,
    WHITE = 0xFFFFFF,
    RED = 0xFF0000,
    GREEN = 0x00FF00,
    BLUE = 0x0000FF,
    YELLOW = 0xFFFF00,
    MAGENTA = 0xFF00FF,
    CYAN = 0x00FFFF
} Color;

// Helper function to convert Color to cv::Scalar
inline cv::Scalar ColorToScalar(Color color, int alpha = 255) {
    return cv::Scalar(
        (color & 0xFF),          // B
        (color & 0xFF00) >> 8,   // G
        (color & 0xFF0000) >> 16,// R
        alpha                    // A
    );
}

class DrawingOverlay: public Overlay, private virtual Logger {
public:
    DrawingOverlay(unsigned int width, unsigned int height, unsigned int zpos=0);
    DrawingOverlay(VideoChannel &ch, unsigned int zpos=0);
    ~DrawingOverlay();

    void CleanFrame();
    void PrintText(std::string text, int x, int y, int size, Color color, int transparency=255);
    void DrawRect(int x1, int y1, int x2, int y2, int thickness, Color color, int transparency);
    void DrawRect(const NormalizedBoxRect &nbox, int thickness, Color color, int transparency);
    void DrawLine(int x1, int y1, int x2, int y2, int thickness, Color color, int transparency);
    void DrawCircle(int x, int y, int radius, int thickness, Color color, int transparency);
private:
    rc_t AllocateFrame();
    cv::Mat m_frame;
};

}