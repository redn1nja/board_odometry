#include "odometry.h"
#include <numeric>


namespace nav {

    void FlowOdometry::process_frame(cv::Mat frame, bool draw) {
        if (m_frame.empty()) {
            feature_detection(frame);
            return;
        }
        auto [old_features, new_features] = feature_matching(frame, m_features, draw);
        m_offset = get_offset(old_features, new_features);
        if (draw) draw_frame();
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        feature_detection(frame);
    }

    void FlowOdometry::feature_detection(cv::Mat frame) {
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        cv::Mat gray;
        cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
        goodFeaturesToTrack(gray, m_features, 1000, 0.1, 2, cv::Mat(), 7, false, 0.04);
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
            if (status[i]) {
                good_old.push_back(points[i]);
                good_new.push_back(new_points[i]);
            }
        }
        cv::Mat inliers_mask;
        FeatureVector inliers_old;
        FeatureVector inliers_new;

        cv::Mat _ = cv::estimateAffinePartial2D(good_old, good_new, inliers_mask, cv::RANSAC, 3);

        for (size_t i = 0; i < inliers_mask.rows; i++) {
            if (inliers_mask.at<uchar>(i)) {
                inliers_old.push_back(good_old[i]);
                inliers_new.push_back(new_points[i]);
            }
        }
        if (draw) {
            for (size_t i = 0; i < inliers_old.size(); i++) {
                cv::line(m_draw_frame, inliers_old[i], inliers_new[i], cv::Scalar(0, 255, 0), 2);
                cv::circle(m_draw_frame, inliers_new[i], 5, cv::Scalar(0, 0, 255), -1);
            }
        }
        return {inliers_old, inliers_new};
    }

    cv::Vec2d FlowOdometry::get_offset(const FeatureVector &p1, const FeatureVector &p2) {
        auto p1_2d = std::vector(p1.begin(), p1.end());
        auto p2_2d = std::vector(p2.begin(), p2.end());
       return SVD_offset(p1_2d, p2_2d);
    }




}// namespace nav