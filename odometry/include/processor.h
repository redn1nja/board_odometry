#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "odometry.h"
#include "correction.h"

namespace nav {
    class ImageProc {
        Odometry m_odometry;
        ImageCorrection m_correction;
        cv::Vec2d m_total_offset;
    public:
        void calclulate_offsets(cv::Mat frame, const ImageCorrection::Attitude& attitude, double altitude);
        cv::Vec2d pixel_to_meter(const cv::Vec2d& offset, double altitude);
        cv::Vec2d total_offset() { return m_total_offset; }
    };
} //namespace nav
#endif //PROCESSOR_H
