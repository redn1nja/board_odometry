#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <opencv2/opencv.hpp>
namespace nav {
    class Odometry {
    public:
        Odometry();
        ~Odometry();
        void FeatureDetection(cv::Mat frame);
        void ProcessFrame(cv::Mat frame, bool draw = true);
        void DrawFrame() const;
        [[nodiscard]] cv::Vec2d GetOffset(cv::Mat frame, const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2) const;
        [[nodiscard]] cv::Vec2d offset() { return m_offset; }
    private:
        [[nodiscard]] cv::Point2f computeAverageMatchOffset(const std::vector<cv::Point2f>& ref_features ) const;
        cv::Mat m_frame;
        cv::Mat m_draw_frame;
        cv::Vec2d m_offset;

        std::vector<cv::Point2f> m_features;
        [[nodiscard]] std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> FeatureMatching(cv::Mat old_frame, const std::vector<cv::Point2f>& points, bool draw) const;

    };
}//namespace nav
#endif //ODOMETRY_H
