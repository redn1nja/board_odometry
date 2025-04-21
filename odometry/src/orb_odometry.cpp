#include <odometry.h>
#include <numeric>

namespace nav {

    ORBOdometry::ORBOdometry() {
        m_orb = cv::ORB::create(250, 1.2f, 12, 60, 0, 2, cv::ORB::HARRIS_SCORE, 60);
        m_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    }

    void ORBOdometry::feature_detection(cv::Mat frame) {
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        cv::Mat gray;
        cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
        m_features.clear();
        // m_features.reserve(250);
        m_descriptors = cv::Mat();
        m_orb->detect(gray, m_features);
        m_orb->compute(gray, m_features, m_descriptors);
    }

    std::pair<ORBOdometry::FeatureVector, ORBOdometry::FeatureVector> ORBOdometry::feature_matching(cv::Mat frame, const FeatureVector &points, bool draw) {
        FeatureVector new_keypoints;
        cv::Mat new_descriptors;
        new_descriptors.resize(640);
        cv::Mat gray_mat;
        std::vector<cv::DMatch> matches;
        cvtColor(frame, gray_mat, cv::COLOR_BGR2GRAY);
        m_orb->detectAndCompute(gray_mat, cv::Mat(), new_keypoints, new_descriptors);

        m_matcher->match(m_descriptors, new_descriptors, matches);
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        cv::Mat inliers_mask;
        std::vector<cv::DMatch> good_matches;
        std::vector<cv::KeyPoint> new_keypoints_filtered;
        std::vector<cv::KeyPoint> points_filtered;

        std::vector<cv::Point2f> points_2d;
        std::vector<cv::Point2f> new_points_2d;

        cv::Mat good_descriptors;

        std::transform(matches.begin(), matches.end(), std::back_inserter(points_2d),
            [&points](const cv::DMatch& p) { return points[p.queryIdx].pt; });
        std::transform(matches.begin(), matches.end(), std::back_inserter(new_points_2d),
            [&new_keypoints](const cv::DMatch& p) { return new_keypoints[p.trainIdx].pt; });

        cv::Mat H = cv::estimateAffinePartial2D(points_2d, new_points_2d, inliers_mask, cv::RANSAC, 3);
        for (size_t i = 0; i < inliers_mask.rows; i++) {
            if (inliers_mask.at<uchar>(i)) {
                good_matches.push_back(matches[i]);
                new_keypoints_filtered.push_back(new_keypoints[matches[i].trainIdx]);
                points_filtered.push_back(points[matches[i].queryIdx]);
                good_descriptors.push_back(m_descriptors.row(matches[i].queryIdx));
            }
        }

        if(draw) {
            drawMatches(m_frame, points, frame, new_keypoints
                ,good_matches, m_draw_frame);
        }
        m_descriptors = std::move(good_descriptors);
        return {points_filtered, new_keypoints_filtered};
    }

    void ORBOdometry::process_frame(cv::Mat frame, bool draw) {
        if (m_frame.empty()) {
            feature_detection(frame);
            return;
        }
        if (m_features.size() < 4) {
            feature_detection(frame);
        }
        auto [old_features, new_features] = feature_matching(frame, m_features, draw);
        if (new_features.size() < 4 ) {
            feature_detection(frame);
            std::cerr << "Not enough features detected\n";
            m_offset = {0,0};
            return;
        }
        m_offset = get_offset(old_features, new_features);
        if (draw) draw_frame();
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        m_features = new_features;
        feature_detection(frame);
    }

    cv::Vec2d ORBOdometry::get_offset(const FeatureVector& p1, const FeatureVector& p2) {

        std::vector<cv::Point2f> p1_2d;
        std::vector<cv::Point2f> p2_2d;
        ptrdiff_t feature_size = static_cast<ptrdiff_t>(std::min(p1.size(), p2.size()));
        std::transform(p1.begin(), p1.begin() + feature_size, std::back_inserter(p1_2d),
            [](const cv::KeyPoint& p) { return cv::Point2f(p.pt.x, p.pt.y); });
        std::transform(p2.begin(), p2.begin() + feature_size, std::back_inserter(p2_2d),
            [](const cv::KeyPoint& p) { return cv::Point2f(p.pt.x, p.pt.y); });

        return SVD_offset(p1_2d, p2_2d);
    }


} //namespace nav
