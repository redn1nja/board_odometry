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
        AccelOdometryFilter m_ekf;

        bool m_draw = false;
        ROLL_MODE m_roll_mode = ROLL_MODE::ROLL_POSITIVE;
        PITCH_MODE m_pitch_mode = PITCH_MODE::PITCH_POSITIVE;
        SVD_MODE m_svd_mode = SVD_MODE::SVD_POSITIVE;

        bool is_collinear(const cv::Vec2d& a, const cv::Vec2d& b) {
            return std::abs(a[0] * b[1] - a[1] * b[0]) < 1e-6;
        }

    public:
        ImageProc(cv::Mat Q, cv::Mat R, bool draw) {
            m_ekf.set_QR(Q, R);
            m_draw = draw;
        }

        void set_start_offset(const cv::Vec2d& offset) {
            m_total_offset = offset;
        }
        cv::Vec2d calclulate_offsets(cv::Mat frame, const ImageCorrection::Attitude& attitude, const cv::Vec3d& acceleration, double dt, double altitude) {
            auto corrected_frame = m_correction.transform_frame(frame, attitude);
            m_odometry.process_frame(corrected_frame, m_draw);
            auto odometry = pixel_to_meter(m_odometry.offset(), altitude);
            auto corrected = m_ekf.step(acceleration, odometry, attitude, dt);

            m_total_offset += (corrected * dt);
            std::cout << "Raw odometry: " << odometry << " Corrected odometry: " << corrected << "\n";
            return corrected * dt;
            m_total_offset += odometry;
            std::cout << "Raw odometry: " << odometry << " Corrected odometry: " << m_total_offset << "\n";
            return odometry;

        }
        cv::Vec2d pixel_to_meter(const cv::Vec2d& offset, double altitude) {
            auto K = m_correction.K();
            double h = altitude * std::tan(K.hfov / 2);
            double v = altitude * std::tan(K.vfov / 2);
            double dx = 2 * offset[0] * h / K.width;
            double dy = 2 * offset[1] * v / K.height;
            return {dx, dy};
        }
        cv::Vec2d total_offset() { return m_total_offset; }
    };
} //namespace nav
#endif //PROCESSOR_H
