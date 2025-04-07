#include "odometry.h"
#include <numeric>

namespace nav {

    void FlowOdometry::process_frame(cv::Mat frame, bool draw) {
        if (m_frame.empty()) {
            feature_detection(frame);
            return;
        }
        if (m_features.size() < 4) {
            feature_detection(frame);
        }
        auto [old_features, new_features] = feature_matching(frame, m_features, draw);
        if (new_features.size() < 4) {
            feature_detection(frame);
            std::cerr << "Not enough features detected\n";
            // m_offset = {0,0};
            return;
        }
        m_offset = get_offset(old_features, new_features);
        if (draw) draw_frame();
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        m_features = new_features;
        if (m_features.size() < 5) {
            feature_detection(frame);
        }
    }

    void FlowOdometry::feature_detection(cv::Mat frame) {
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        cv::Mat gray;
        cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
        goodFeaturesToTrack(gray, m_features, 500, 0.1, 2, cv::Mat(), 7, false, 0.04);
    }

    std::pair<FlowOdometry::FeatureVector, FlowOdometry::FeatureVector> FlowOdometry::feature_matching(cv::Mat frame, const FeatureVector &points, bool draw) {
        cv::Mat gray_old;
        cv::Mat gray_mat;
        cvtColor(m_frame, gray_old, cv::COLOR_BGR2GRAY);
        cvtColor(frame, gray_mat, cv::COLOR_BGR2GRAY);
        FeatureVector new_points;
        std::vector<uchar> status;
        std::vector<float> err;
        calcOpticalFlowPyrLK(gray_old, gray_mat, points, new_points, status, err);

        FeatureVector good_old;
        FeatureVector good_new;
        for (size_t i = 0; i < points.size(); i++) {
            if (status[i] && err[i] < 10) {
                good_old.push_back(points[i]);
                good_new.push_back(new_points[i]);
                if (draw){
                    line(m_draw_frame, new_points[i], points[i], cv::Scalar(0, 0, 255), 2);
                    circle(m_draw_frame, new_points[i], 3, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
        return {good_old, good_new};
    }

    cv::Vec2d FlowOdometry::get_offset(const FeatureVector &p1, const FeatureVector &p2) {

        std::vector<cv::Point3f> p1_3d;
        std::vector<cv::Point3f> p2_3d;
        ptrdiff_t feature_size = static_cast<ptrdiff_t>(std::min(p1.size(), p2.size()));
        std::transform(p1.begin(), p1.begin() + feature_size, std::back_inserter(p1_3d),
            [](const cv::Point2f& p) { return cv::Point3f(p.x, p.y, 0); });
        std::transform(p2.begin(), p2.begin() + feature_size, std::back_inserter(p2_3d),
            [](const cv::Point2f& p) { return cv::Point3f(p.x, p.y, 0); });

       return SVD_offset(p1_3d, p2_3d);
    }

    cv::Vec2d FlowOdometry::compute_average_feature_offset(const FeatureVector &ref_features) {
        cv::Vec2d offset(0, 0);
        for (const auto& feature : ref_features) {
            offset += cv::Vec2d(feature.x, feature.y);
        }
        offset /= static_cast<double>(ref_features.size());
        return offset;
    }


}// namespace nav