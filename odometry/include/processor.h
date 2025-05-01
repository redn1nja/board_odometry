#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <angular.h>

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

    public:
        ImageProc(cv::Mat Q, cv::Mat R, bool draw, double yaw = 0.0) {
            m_ekf.set_QR(Q, R);
            m_draw = draw;
            m_odometry.set_R(yaw);
        }

        void set_start_offset(const cv::Vec2d& offset) {
            m_total_offset = offset;
        }

        void set_odom_R (double yaw) {
            m_odometry.set_R(yaw);
        }

        double R() { return m_odometry.R_angle();}

        cv::Vec2d calclulate_offsets(cv::Mat frame, ImageCorrection::Attitude& attitude, const cv::Vec3d& acceleration, double dt, double altitude) {
            auto corrected_frame = m_correction.transform_frame(frame, attitude);
            m_odometry.process_frame(corrected_frame, m_draw);
            cv::Vec2d odometry = m_odometry.offset();
            odometry = pixel_to_meter(odometry, altitude);
            cv::Mat odometry_mat = cv::Mat(odometry);
            attitude.yaw = m_odometry.R_angle();
            attitude.roll = attitude.roll * m_roll_mode;
            attitude.pitch = attitude.pitch * m_pitch_mode;


            cv::Mat R = (cv::Mat_<double>(2, 2) << std::cos(attitude.yaw), -std::sin(attitude.yaw),
                std::sin(attitude.yaw), std::cos(attitude.yaw));
            odometry_mat = (R.t() * m_odometry.last_R())* odometry_mat;
            odometry = cv::Vec2d(odometry_mat.at<double>(0, 0), odometry_mat.at<double>(1, 0));
            if (odometry[0] == 0 && odometry[1] == 0) {
                std::cerr << "Zero odometry\n";
                return odometry;
            }

            auto corrected = m_ekf.step(acceleration, odometry, attitude, dt);
            m_total_offset += (odometry);
            return corrected * dt;

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
