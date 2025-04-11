#ifndef PROCESSOR_H
#define PROCESSOR_H

#include "odometry.h"
#include "correction.h"
#include "ekf.h"

namespace nav {
    template <typename Odom>
    class ImageProc {
    public:
        void set_modes(ROLL_MODE r, PITCH_MODE p, SVD_MODE s) {
            m_roll_mode = r;
            m_pitch_mode = p;
            m_svd_mode = s;
            m_correction.set_modes(r, p);
            m_odometry.set_svd_mode(s);
        }
    private:
        Odom m_odometry;
        ImageCorrection m_correction;
        cv::Vec2d m_total_offset;

        ROLL_MODE m_roll_mode = ROLL_MODE::ROLL_POSITIVE;
        PITCH_MODE m_pitch_mode = PITCH_MODE::PITCH_POSITIVE;
        SVD_MODE m_svd_mode = SVD_MODE::SVD_POSITIVE;

    public:

        void calclulate_offsets(cv::Mat frame, const ImageCorrection::Attitude& attitude, double altitude) {
            auto corrected_frame = m_correction.transform_frame(frame, attitude);
            m_odometry.process_frame(corrected_frame, false);
            m_total_offset += pixel_to_meter(m_odometry.offset(), altitude);
        }
        cv::Vec2d pixel_to_meter(const cv::Vec2d& offset, double altitude) {
            double pitch_mult = altitude * std::tan(m_correction.K().hfov / 2);
            double roll_mult = altitude * std::tan(m_correction.K().vfov / 2);
            double dx = 2 * offset[0] * pitch_mult / m_correction.K().width;
            double dy = 2 * offset[1] * roll_mult / m_correction.K().height;
            return {dx, dy};
        }
        cv::Vec2d total_offset() { return m_total_offset; }
    };
} //namespace nav
#endif //PROCESSOR_H
