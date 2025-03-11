#include "DrawingOverlay.h"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

namespace vision {

DrawingOverlay::DrawingOverlay(unsigned int width, unsigned int height, unsigned int zpos):
    Overlay(width, height, zpos),
    Logger("DrawingOverlay")
{
    LogInfo("Creating DrawingOverlay");
    AllocateFrame();
}

DrawingOverlay::DrawingOverlay(VideoChannel &ch, unsigned int zpos):
    DrawingOverlay(ch.GetWidth(), ch.GetHeight(), zpos)
{}

DrawingOverlay::~DrawingOverlay()
{
    LogInfo("Destroying Overlay");
    m_frame.release();
}

rc_t DrawingOverlay::AllocateFrame()
{
    m_frame = cv::Mat(height, width, CV_8UC4, GetFrameFd());
    m_frame.addref();
    CleanFrame();
    
    return RC_OK;
}

void DrawingOverlay::CleanFrame()
{
    m_frame = cv::Scalar(0, 0, 0, 0);
}

void DrawingOverlay::DrawRect(int x1, int y1, int x2, int y2, int thickness, Color color, int transparency)
{
    cv::rectangle(m_frame, cv::Point(x1, y1), cv::Point(x2, y2), ColorToScalar(color, transparency), thickness);
}

void DrawingOverlay::DrawRect(const NormalizedBoxRect &nbox, int thickness, Color color, int transparency)
{
    int x1 = nbox.left * width;
    int y1 = nbox.top * height;
    int x2 = nbox.right * width;
    int y2 = nbox.bottom * height;
    cv::rectangle(m_frame, cv::Point(x1, y1), cv::Point(x2, y2), ColorToScalar(color, transparency), thickness);
}

void DrawingOverlay::PrintText(std::string text, int x, int y, int size, Color color, int transparency)
{
    cv::putText(m_frame, text, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, size, ColorToScalar(color, transparency), 2);
}

void DrawingOverlay::DrawLine(int x1, int y1, int x2, int y2, int thickness, Color color, int transparency)
{
    cv::line(m_frame, cv::Point(x1, y1), cv::Point(x2, y2), ColorToScalar(color, transparency), thickness);
}

void DrawingOverlay::DrawCircle(int x, int y, int radius, int thickness, Color color, int transparency)
{
    cv::circle(m_frame, cv::Point(x, y), radius, ColorToScalar(color, transparency), thickness);
}

}