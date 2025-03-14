#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <opencv2/opencv.hpp>

class Odometry {
public:
    Odometry();
    ~Odometry();
    void FeatureDetection(cv::Mat frame);
    void ProcessFrame(cv::Mat frame, bool draw = true);

    void DrawFrame() const;
private:
    cv::Mat m_frame;
    cv::Mat m_draw_frame;
    std::vector<cv::Point2f> m_features;
    [[nodiscard]] std::vector<cv::Point2f> FeatureMatching(cv::Mat old_frame, const std::vector<cv::Point2f>& points) const;

};
#endif //ODOMETRY_H
