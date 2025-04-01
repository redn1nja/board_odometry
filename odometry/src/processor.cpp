#include "processor.h"

void ImageProc::calclulate_offsets(cv::Mat frame, const ImageCorrection::Attitude &attitude) {
    auto corrected_frame = m_correction.transform_frame(frame, attitude);
    m_odometry.ProcessFrame(corrected_frame, false);
    m_total_offset += m_odometry.offset();
    m_odometry.DrawFrame();
}