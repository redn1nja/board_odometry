#include <odometry.h>

Odometry::Odometry() = default;

Odometry::~Odometry() = default;

void Odometry::ProcessFrame(cv::Mat frame, bool draw) {
    auto features = FeatureMatching(frame, m_features);
    if (draw) DrawFrame();
    frame.copyTo(m_frame);
    frame.copyTo(m_draw_frame);
    m_features = features;
}

void Odometry::FeatureDetection(cv::Mat frame) {
    frame.copyTo(m_frame);
    frame.copyTo(m_draw_frame);
    cv::Mat gray;
    cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
    goodFeaturesToTrack(gray, m_features, 500, 0.3, 7, cv::Mat(), 7, false, 0.04);
}

std::vector<cv::Point2f> Odometry::FeatureMatching(cv::Mat frame, const std::vector<cv::Point2f>& points) const {
    cv::Mat gray_old;
    cv::Mat gray_mat;
    cvtColor(m_frame, gray_old, cv::COLOR_BGR2GRAY);
    cvtColor(frame, gray_mat, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> new_points;
    std::vector<uchar> status;
    std::vector<float> err;
    calcOpticalFlowPyrLK(gray_old, gray_mat, points, new_points, status, err);

    std::vector<cv::Point2f> good_new;
    for (size_t i = 0; i < points.size(); i++) {
        if (status[i]) {
            good_new.push_back(new_points[i]);
            cv::line(m_draw_frame, new_points[i], points[i], cv::Scalar(0, 0, 255), 2);
            cv::circle(m_draw_frame, new_points[i], 5, cv::Scalar(0, 255, 0), -1);
        }
    }
    return good_new;
}

void Odometry::DrawFrame() const {
    imshow("Frame", m_draw_frame);
}