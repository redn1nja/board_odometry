#include <odometry.h>
#include <numeric>

namespace nav {

    ORBOdometry::ORBOdometry() {
        m_orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
        m_matcher = cv::FlannBasedMatcher();
    }

    void ORBOdometry::feature_detection(cv::Mat frame) {
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        cv::Mat gray;
        cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
        m_orb->detectAndCompute(gray, cv::Mat(), m_features, m_descriptors);
        m_descriptors.convertTo(m_descriptors, CV_32F);
    }

    std::pair<ORBOdometry::FeatureVector, ORBOdometry::FeatureVector> ORBOdometry::feature_matching(cv::Mat frame, const FeatureVector &points, bool draw) const {
        std::vector<cv::KeyPoint> new_keypoints;
        cv::Mat new_descriptors;
        cv::Mat gray_mat;
        std::vector<cv::DMatch> matches;
        cvtColor(frame, gray_mat, cv::COLOR_BGR2GRAY);
        m_orb->detectAndCompute(gray_mat, cv::Mat(), new_keypoints, new_descriptors);
        // m_orb->compute(gray_mat, new_keypoints, new_descriptors);

        new_descriptors.convertTo(new_descriptors, CV_32F);
        m_matcher.match(m_descriptors, new_descriptors, matches);
        // matches.end() = std::remove_if(matches.begin(), matches.end(), [](const cv::DMatch& m) {
        //     return m.distance > 100;
        // });
        FeatureVector good_old;
        FeatureVector good_new;
        for (const auto& match : matches) {
            if (match.distance < 30) {
                good_old.push_back(points[match.queryIdx]);
                good_new.push_back(new_keypoints[match.trainIdx]);
                if (draw) {
                    cv::line(m_draw_frame, new_keypoints[match.trainIdx].pt, points[match.queryIdx].pt, cv::Scalar(0, 0, 255), 2);
                    cv::circle(m_draw_frame, new_keypoints[match.trainIdx].pt, 3, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
        std::cout << good_old.size() << "\n";
        return {good_old, good_new};
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
        if (m_features.size() < 5) {
            feature_detection(frame);
        }
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
