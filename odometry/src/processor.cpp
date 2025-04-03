#include "processor.h"

namespace nav {
    void ImageProc::calclulate_offsets(cv::Mat frame, const ImageCorrection::Attitude &attitude, double altitude) {
        auto corrected_frame = m_correction.transform_frame(frame, attitude);
        m_odometry.ProcessFrame(corrected_frame);
        m_total_offset += pixel_to_meter(m_odometry.offset(), altitude);
        // m_odometry.DrawFrame();

    }

    cv::Vec2d ImageProc::pixel_to_meter(const cv::Vec2d &offset, double altitude) {
        double pitch_mult = altitude * std::tan(m_correction.K().hfov / 2);
        double roll_mult = altitude * std::tan(m_correction.K().vfov / 2);
        double dx = 2 * offset[0] * pitch_mult / m_correction.K().width;
        double dy = 2 * offset[1] * roll_mult / m_correction.K().height;
        return {dx, dy};
    }


} //namespace nav