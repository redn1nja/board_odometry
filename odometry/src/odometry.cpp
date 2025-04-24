#include <numeric>
#include <odometry.h>

namespace nav {
    Odometry::Odometry() {
        m_R = cv::Mat::eye(2, 2, CV_64F);
        m_last_R = cv::Mat::eye(2, 2, CV_64F);
    }


    void Odometry::draw_frame() const {
        imshow("Frame", m_draw_frame);
        if (cv::waitKey(30) == 32) {
            cv::waitKey(0);
        }
    }

    cv::Vec2d Odometry::SVD_offset(std::vector<cv::Point2f> &p1_2d, std::vector<cv::Point2f> &p2_2d) {
        cv::Point2f p1_mean = std::accumulate(p1_2d.begin(), p1_2d.end(), cv::Point2f{},
            [](const cv::Point2d& p1, const cv::Point2d& p2) {
                return cv::Point2f(p1.x + p2.x, p1.y + p2.y);
        });
        cv::Point2f p2_mean = std::accumulate(p2_2d.begin(), p2_2d.end(), cv::Point2f{},
            [](const cv::Point2d& p1, const cv::Point2d& p2) {
                return cv::Point2f(p1.x + p2.x, p1.y + p2.y);
            });


        cv::Mat U, W, Vt;

        p1_mean /= static_cast<float>(p1_2d.size());
        p2_mean /= static_cast<float>(p2_2d.size());

        std::for_each(p1_2d.begin(), p1_2d.end(), [p1_mean](cv::Point2f& p) {
            p = p - p1_mean;
        });

        std::for_each(p2_2d.begin(), p2_2d.end(), [p2_mean](cv::Point2f& p) {
            p = p - p2_mean;
        });

        cv::Mat H = cv::Mat(p1_2d).reshape(1).t() * cv::Mat(p2_2d).reshape(1);

        cv::SVD::compute(H, W, U, Vt);
        cv::Mat R = Vt.t() * U.t();
        R.convertTo(R, CV_64F);



        cv::Vec2d center_v(m_frame.cols/2, m_frame.rows/2);
        cv::Mat center = cv::Mat(center_v);


        cv::Mat p1_mean_mat = cv::Mat(p1_mean);
        cv::circle(m_draw_frame, cv::Point2f(p1_mean.x, p1_mean.y), 5, cv::Scalar(255, 0, 0), -1);

        p1_mean_mat.convertTo(p1_mean_mat, CV_64F);
        p1_mean_mat -= center;

        cv::Mat p2_mean_mat = cv::Mat(p2_mean);
        p2_mean_mat.convertTo(p2_mean_mat, CV_64F);

        cv::Mat p2_cpy = p2_mean_mat.clone();
        p2_cpy = R.t() * p2_cpy;
        cv::circle(m_draw_frame, cv::Point2f(p2_cpy.at<double>(0,0), p2_cpy.at<double>(1,0)), 5, cv::Scalar(0, 255, 0), -1);

        p2_mean_mat -= center;

        p2_mean_mat = R.t() * p2_mean_mat;



        cv::Mat t = (p1_mean_mat - p2_mean_mat);
        m_R *= R;
        m_last_R = R;
        double x = t.at<double>(0, 0);
        double y = t.at<double>(1, 0);

        return { -x , y};

    }

    void Odometry::set_R(double yaw) {
        m_R = cv::Mat::eye(2, 2, CV_64F);
        m_R.at<double>(0, 0) = cos(yaw);
        m_R.at<double>(0, 1) = -sin(yaw);
        m_R.at<double>(1, 0) = sin(yaw);
        m_R.at<double>(1, 1) = cos(yaw);
    }

} //namespace nav
