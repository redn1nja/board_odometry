#include <numeric>
#include <odometry.h>

namespace nav {

    void Odometry::draw_frame() const {
        imshow("Frame", m_draw_frame);
        cv::waitKey(0);
    }

    cv::Vec2d Odometry::SVD_offset(std::vector<cv::Point3f> &p1_3d, std::vector<cv::Point3f> &p2_3d) {
        cv::Point3f p1_mean = std::accumulate(p1_3d.begin(), p1_3d.end(), cv::Point3f(0, 0, 0),
            [](const cv::Point3f& p1, const cv::Point3f& p2) {
            return cv::Point3f(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
        });
        cv::Point3f p2_mean = std::accumulate(p2_3d.begin(), p2_3d.end(), cv::Point3f(0, 0, 0),
            [](const cv::Point3f& p1, const cv::Point3f& p2) {
            return cv::Point3f(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
        });
        p1_mean /= static_cast<float>(p1_3d.size());
        p2_mean /= static_cast<float>(p2_3d.size());

        std::for_each(p1_3d.begin(), p1_3d.end(), [p1_mean](cv::Point3f& p) {
            p = p - p1_mean;
        });

        std::for_each(p2_3d.begin(), p2_3d.end(), [p2_mean](cv::Point3f& p) {
            p = p - p2_mean;
        });

        cv::Mat p1_mat = cv::Mat(p1_3d).reshape(1);
        cv::Mat p2_mat = cv::Mat(p1_3d).reshape(1);

        cv::Mat H = p1_mat.t() * p2_mat;

        // cv::Mat U, Vt, _;
        // cv::SVD::compute(H, _, U, Vt);
        //
        // cv::Mat R = Vt.t() * U.t();
        // if (determinant(R) < 0) {
        //     R.col(2) *= -1;
        //
        // }
        // cv::Mat t = cv::Mat(p2_mean) - R * cv::Mat(p1_mean);
        //
        //
        // return { -t.at<float>(0, 0) * m_svd_mode , t.at<float>(1, 0) * m_svd_mode};
        auto r = p2_mean - p1_mean;
        return {r.x, r.y};
    }


} //namespace nav
