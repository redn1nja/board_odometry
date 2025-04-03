#include <numeric>
#include <odometry.h>
namespace nav {
    Odometry::Odometry() = default;

    Odometry::~Odometry() = default;

    void Odometry::ProcessFrame(cv::Mat frame, bool draw) {
        if (m_frame.empty()) {
            FeatureDetection(frame);
            return;
        }
        if (m_features.size() < 10) {
            FeatureDetection(frame);
        }
        auto [old_features, new_features] = FeatureMatching(frame, m_features, draw);
        if (new_features.size() < 10) {
            FeatureDetection(frame);
            std::cerr << "Not enough features detected\n";
            m_offset = {0,0};
            return;
        }
        m_offset = GetOffset(frame, old_features, new_features);
        if (draw) DrawFrame();
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        m_features = new_features;
        if (m_features.size() < 5) {
            FeatureDetection(frame);
        }
    }

    void Odometry::FeatureDetection(cv::Mat frame) {
        frame.copyTo(m_frame);
        frame.copyTo(m_draw_frame);
        cv::Mat gray;
        cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
        goodFeaturesToTrack(gray, m_features, 500, 0.1, 2, cv::Mat(), 7, false, 0.04);
    }

    std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> Odometry::FeatureMatching(cv::Mat frame, const std::vector<cv::Point2f>& points, bool draw) const {
        cv::Mat gray_old;
        cv::Mat gray_mat;
        cvtColor(m_frame, gray_old, cv::COLOR_BGR2GRAY);
        cvtColor(frame, gray_mat, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> new_points;
        std::vector<uchar> status;
        std::vector<float> err;
        calcOpticalFlowPyrLK(gray_old, gray_mat, points, new_points, status, err);

        std::vector<cv::Point2f> good_old;
        std::vector<cv::Point2f> good_new;
        for (size_t i = 0; i < points.size(); i++) {
            if (status[i] && err[i] < 10) {
                good_old.push_back(points[i]);
                good_new.push_back(new_points[i]);
                if (draw){
                    cv::line(m_draw_frame, new_points[i], points[i], cv::Scalar(0, 0, 255), 2);
                    cv::circle(m_draw_frame, new_points[i], 3, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
        return {good_old, good_new};
    }

    void Odometry::DrawFrame() const {
        imshow("Frame", m_draw_frame);
        cv::waitKey(0);
    }
    cv::Vec2d Odometry::GetOffset(cv::Mat frame, const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2) const {

        std::vector<cv::Point3f> p1_3d;
        std::vector<cv::Point3f> p2_3d;
        ptrdiff_t feature_size = static_cast<ptrdiff_t>(std::min(p1.size(), p2.size()));
        std::transform(p1.begin(), p1.begin() + feature_size, std::back_inserter(p1_3d),
            [](const cv::Point2f& p) { return cv::Point3f(p.x, p.y, 0); });
        std::transform(p2.begin(), p2.begin() + feature_size, std::back_inserter(p2_3d),
            [](const cv::Point2f& p) { return cv::Point3f(p.x, p.y, 0); });

        cv::Point3f p1_mean = std::accumulate(p1_3d.begin(), p1_3d.end(), cv::Point3f(0, 0, 0),
            [](const cv::Point3f& p1, const cv::Point3f& p2) {
            return cv::Point3f(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
        });
        cv::Point3f p2_mean = std::accumulate(p2_3d.begin(), p2_3d.end(), cv::Point3f(0, 0, 0),
            [](const cv::Point3f& p1, const cv::Point3f& p2) {
            return cv::Point3f(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
        });
        p1_mean /= static_cast<float>(p1.size());
        p2_mean /= static_cast<float>(p2.size());

        std::for_each(p1_3d.begin(), p1_3d.end(), [p1_mean](cv::Point3f& p) {
            p = p - p1_mean;
        });

        std::for_each(p2_3d.begin(), p2_3d.end(), [p2_mean](cv::Point3f& p) {
            p = p - p2_mean;
        });

        cv::Mat p1_mat = cv::Mat(p1_3d).reshape(1);
        cv::Mat p2_mat = cv::Mat(p1_3d).reshape(1);

        cv::Mat H = p1_mat.t() * p2_mat;

        cv::Mat U, Vt, _;
        cv::SVD::compute(H, _, U, Vt);

        cv::Mat R = Vt.t() * U.t();
        if (determinant(R) < 0) {
            Vt.at<double>(2) *= -1;
            R = Vt.t() * U.t();
        }
        cv::Mat t = cv::Mat(p2_mean) - R * cv::Mat(p1_mean);

        return {-t.at<float>(0, 0), -t.at<float>(1, 0)};
    }

    cv::Point2f Odometry::computeAverageMatchOffset(const std::vector<cv::Point2f>& ref_features) const {
        cv::Point2f offset{0, 0};
        for (size_t i = 0; i < ref_features.size(); i++) {
            offset += (ref_features[i] - m_features[i]);
        }

        offset /= static_cast<float>(ref_features.size());
        return offset;
    }
} //namespace nav
