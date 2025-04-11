#include <odometry.h>
#include <numeric>

namespace nav {

    ORBOdometry::ORBOdometry() {
        m_orb = cv::ORB::create(1000, 1.2f, 8, 15, 0, 2, cv::ORB::HARRIS_SCORE, 15);
        m_matcher = cv::FlannBasedMatcher();
    }

    void ORBOdometry::feature_detection(cv::Mat frame) {
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        cv::Mat gray;
        cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
        m_features.clear();
        m_descriptors = cv::Mat();
        m_orb->detectAndCompute(gray, cv::Mat(), m_features, m_descriptors);
        m_descriptors.convertTo(m_descriptors, CV_32F);
    }

    std::pair<ORBOdometry::FeatureVector, ORBOdometry::FeatureVector> ORBOdometry::feature_matching(cv::Mat frame, const FeatureVector &points, bool draw) {
        FeatureVector new_keypoints;
        cv::Mat new_descriptors;
        cv::Mat gray_mat;
        std::vector<cv::DMatch> matches;
        cvtColor(frame, gray_mat, cv::COLOR_BGR2GRAY);
        m_orb->detectAndCompute(gray_mat, cv::Mat(), new_keypoints, new_descriptors);

        new_descriptors.convertTo(new_descriptors, CV_32F);
        m_matcher.match(m_descriptors, new_descriptors, matches);
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

        cv::Mat H = findHomography(points_2d, new_points_2d, cv::RANSAC, 3, inliers_mask);
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
        std::cout << good_matches.size() << "\n";
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

        std::vector<cv::Point3f> p1_3d;
        std::vector<cv::Point3f> p2_3d;
        ptrdiff_t feature_size = static_cast<ptrdiff_t>(std::min(p1.size(), p2.size()));
        std::transform(p1.begin(), p1.begin() + feature_size, std::back_inserter(p1_3d),
            [](const cv::KeyPoint& p) { return cv::Point3f(p.pt.x, p.pt.y, 0); });
        std::transform(p2.begin(), p2.begin() + feature_size, std::back_inserter(p2_3d),
            [](const cv::KeyPoint& p) { return cv::Point3f(p.pt.x, p.pt.y, 0); });

        return SVD_offset(p1_3d, p2_3d);
    }


} //namespace nav
