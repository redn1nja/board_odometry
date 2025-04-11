#ifndef EKF_H
#define EKF_H

#include <opencv2/opencv.hpp>

namespace nav {
    class AcclelerationNormalizer {
        static inline double m_g = 9.81;
    public:
        cv::Vec3d operator()(const cv::Vec3d& acc, const ImageCorrection::Attitude& attitude) const {
            cv::Mat R = cv::Mat::eye(3,3,CV_64F);
            cv::Mat pitch = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(attitude.pitch), -sin(attitude.pitch), 0, sin(attitude.pitch), cos(attitude.pitch));
            cv::Mat roll = (cv::Mat_<double>(3, 3) << cos(attitude.roll), 0, sin(attitude.roll), 0, 1, 0, -sin(attitude.roll), 0, cos(attitude.roll));
            R *= (roll * pitch);
            cv::Mat acc_mat = cv::Mat(acc).reshape(1);
            cv::Mat acc_rotated = R * acc_mat;
            cv::Vec3d acc_normalized = {acc_rotated.at<double>(0, 0), acc_rotated.at<double>(1, 0), acc_rotated.at<double>(2, 0) - m_g};
            return acc_normalized;
        }
    };

    class AccelOdometryFilter {
        AcclelerationNormalizer m_norm;
        cv::Mat Q, R;
        cv::Mat P;
        cv::Vec2d m_state_vel;
        cv::Vec2d m_odometry;
    public:
        AccelOdometryFilter() {
            Q = (cv::Mat_<double>(2, 2) << 0.1, 0, 0, 0.1);
            R = (cv::Mat_<double>(2, 2) << 0.1, 0, 0, 0.1);
        }
        void predict(const cv::Vec3d& acc, const ImageCorrection::Attitude& attitude, double dt) {
            cv::Vec3d acc_normalized = m_norm(acc, attitude);
            m_state_vel[0] += acc_normalized[0] * dt;
            m_state_vel[1] += acc_normalized[1] * dt;
        }

        void correct(const cv::Vec2d& odometry, double dt) {
            auto vel = (odometry - m_odometry) / dt;
            cv::Mat K = P * (P + R).inv();
            m_state_vel[0] += K.at<double>(0, 0) * (vel[0] - m_state_vel[0]);
            m_state_vel[1] += K.at<double>(1, 0) * (vel[1] - m_state_vel[1]);
            P = ((cv::Mat_<double>(2, 2) << 1, 0, 0, 1) - K)* P;

        }

        cv::Vec2d step(const cv::Vec3d& acc, const cv::Vec2d& odom, const ImageCorrection::Attitude& attitude, double dt) {
            predict(acc, attitude, dt);
            correct(odom, dt);
            return m_state_vel;
        }
    };

} // namespace nav
#endif //EKF_H
