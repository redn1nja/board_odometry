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
            cv::Vec3d acc_normalized = {-acc_rotated.at<double>(0, 0), -acc_rotated.at<double>(1, 0), acc_rotated.at<double>(2, 0) + m_g};
            return acc_normalized;
        }
    };

    class AccelOdometryFilter {
        AcclelerationNormalizer m_norm;
        cv::Mat R, Q, P, eye;
        cv::Vec2d m_state_vel;
        // static constexpr double ratio = 0.75;
    public:
        AccelOdometryFilter() {
            Q = cv::Mat::eye(2,2,CV_64F);
            Q.at<double>(0,0) = 0.3;
            Q.at<double>(1,1) = 0.2775;
            R = cv::Mat::eye(2,2,CV_64F) * 0.5;
            P = cv::Mat::eye(2,2,CV_64F);
            eye = cv::Mat::eye(2,2,CV_64F);
            m_state_vel = cv::Vec2d(0, 0);
        }
        void correct(const cv::Vec3d& acc, const ImageCorrection::Attitude& attitude, double dt){
            cv::Vec3d acc_normalized = m_norm(acc, attitude);
            cv::Vec2d vel = {acc_normalized[0] * dt, acc_normalized[1] * dt};
            cv::Mat K = P  * (P + R).inv();
            m_state_vel[0] += K.at<double>(0, 0) * (vel[0] - m_state_vel[0]);
            m_state_vel[1] += K.at<double>(1, 1) * (vel[1] - m_state_vel[1]);
            P = (eye - K) * P;
        }

        void predict(const cv::Vec2d& odometry, double dt) {
            m_state_vel += (odometry / dt);
            P += Q;
        }

        cv::Vec2d step(const cv::Vec3d& acc, const cv::Vec2d& odom, const ImageCorrection::Attitude& attitude, double dt) {
            predict(odom, dt);
            correct(acc, attitude, dt);
            return m_state_vel;
        }
    };
} // namespace nav
#endif //EKF_H
