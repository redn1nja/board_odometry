#ifndef EKF_H
#define EKF_H

#include <opencv2/opencv.hpp>

namespace nav {
    class AcclelerationNormalizer {
        static inline double m_g = 9.81;
    public:
        cv::Vec3d operator()(const cv::Vec3d& acc, const ImageCorrection::Attitude& attitude) const {
            cv::Mat R = cv::Mat::eye(3,3,CV_64F);
            cv::Mat yaw = (cv::Mat_<double>(3, 3) << cos(attitude.yaw), -sin(attitude.yaw), 0, sin(attitude.yaw), cos(attitude.yaw), 0, 0, 0, 1);
            cv::Mat pitch = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(attitude.pitch), -sin(attitude.pitch), 0, sin(attitude.pitch), cos(attitude.pitch));
            cv::Mat roll = (cv::Mat_<double>(3, 3) << cos(attitude.roll), 0, sin(attitude.roll), 0, 1, 0, -sin(attitude.roll), 0, cos(attitude.roll));
            R *= (yaw * pitch * roll);

            cv::Mat acc_mat = cv::Mat(acc).reshape(1);
            cv::Mat acc_rotated = R * acc_mat;
            cv::Vec3d acc_normalized = {acc_rotated.at<double>(0, 0), acc_rotated.at<double>(1, 0), acc_rotated.at<double>(2, 0) + m_g};
            return acc_normalized;
        }
    };



    class AccelOdometryFilter {
        AcclelerationNormalizer m_norm;
        cv::Mat R, Q, P, eye;
        cv::Vec2d m_state_vel{0,0};

    public:
        void set_QR(cv::Mat q, cv::Mat r) {
            Q = q.clone();
            R = r.clone();
        }
        AccelOdometryFilter() {
            P = cv::Mat::eye(2,2,CV_64F);
            eye = cv::Mat::eye(2,2,CV_64F);
        }
        void correct(const cv::Vec3d& acc, double dt){
            cv::Vec2d vel = {acc[0] * dt, acc[1] * dt};
            cv::Mat K = P  * (P + R).inv();
            m_state_vel+= (vel - m_state_vel).mul({K.at<double>(0,0), K.at<double>(1,1)});
            P = (eye - K) * P;
        }

        void predict(const cv::Vec2d& odometry, double dt) {
            m_state_vel += (odometry / dt );
            P += Q;
        }

        cv::Vec2d step(const cv::Vec3d& acc, const cv::Vec2d& odom, const ImageCorrection::Attitude& attitude, double dt) {
            cv::Vec3d acc_normalized = m_norm(acc, attitude);
            predict(odom, dt);
            correct(acc_normalized, dt);
            return m_state_vel;
        }
    };
} // namespace nav
#endif //EKF_H
