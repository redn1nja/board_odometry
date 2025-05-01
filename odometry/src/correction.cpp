#include "correction.h"
#include "odometry.h"

namespace nav {
    cv::Mat ImageCorrection::transform_frame(cv::Mat in_frame, const Attitude &attitude)  {
        auto D = m_K.get_distortion_matrix();
        if (D) {
            cv::Mat out_frame = cv::Mat(in_frame.size(), in_frame.type());
            cv::undistort(in_frame, out_frame, m_K.get_intrinsic_matrix(), D.value());
            in_frame = out_frame;
        }
        cv::Mat output;
        cv::Mat rotation_matrix = cv::Mat::eye(3, 3, CV_64F);

        auto a_roll = attitude.roll * m_roll_mode;
        auto a_pitch = attitude.pitch * m_pitch_mode;

        cv::Mat pitch = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(a_pitch), -sin(a_pitch), 0, sin(attitude.pitch), cos(attitude.pitch));
        cv::Mat roll = (cv::Mat_<double>(3, 3) << cos(a_roll), 0, sin(a_roll), 0, 1, 0, -sin(attitude.roll), 0, cos(attitude.roll));
        rotation_matrix *= (roll * pitch);
        cv::Mat Ki = m_K.get_intrinsic_matrix().clone();
        Ki.at<double>(0, 2) = m_K.width / 2;
        Ki.at<double>(1, 2) = m_K.height / 2;
        cv::Mat Kt = Ki.inv();
        cv::Mat H = Ki * rotation_matrix * Kt;
        cv::warpPerspective(in_frame, output, H, in_frame.size());
        return output;
    }
} //namespace nav
