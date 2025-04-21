#include <numeric>
#include <odometry.h>

namespace nav {

    void Odometry::draw_frame() const {
        imshow("Frame", m_draw_frame);
        cv::waitKey(0);
    }

    cv::Vec2d Odometry::SVD_offset(std::vector<cv::Point2f> &p1_2d, std::vector<cv::Point2f> &p2_2d) {
        cv::Point2d p1_mean = std::accumulate(p1_2d.begin(), p1_2d.end(), cv::Point2d{},
            [](const cv::Point2d& p1, const cv::Point2d& p2) {
                return cv::Point2d{p1.x + p2.x, p1.y + p2.y};
        });
        cv::Point2d p2_mean = std::accumulate(p2_2d.begin(), p2_2d.end(), cv::Point2d{},
            [](const cv::Point2d& p1, const cv::Point2d& p2) {
                return cv::Point2d{p1.x + p2.x, p1.y + p2.y};
            });
        p1_mean /= static_cast<float>(p1_2d.size());
        p2_mean /= static_cast<float>(p2_2d.size());

        return p2_mean - p1_mean;

    }


} //namespace nav
