#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <opencv2/opencv.hpp>
namespace nav {

    enum class ROLL_MODE : short
    {
        ROLL_POSITIVE = 1,
        ROLL_NEGATIVE = -1,
    };
    enum class PITCH_MODE : short {
        PITCH_POSITIVE = 1,
        PITCH_NEGATIVE = -1,
    };
    enum class SVD_MODE : short {
        SVD_POSITIVE = 1,
        SVD_NEGATIVE = -1,
    };

    inline double operator*(const double a, ROLL_MODE b) {
        return a * static_cast<double>(b);
    }

    inline double operator*(const double a, PITCH_MODE b) {
        return a * static_cast<double>(b);
    }

    inline double operator*(double a, SVD_MODE b) {
        return a * static_cast<double>(b);
    }


    class Odometry {
    public:
        Odometry();
        virtual ~Odometry() = default;
        [[nodiscard]] cv::Vec2d offset() { return m_offset;}
        void set_svd_mode(SVD_MODE mode) { m_svd_mode = mode; }
        void set_R(double yaw);
        [[nodiscard]] double R_angle() const { return atan2(m_R.at<double>(1, 0), m_R.at<double>(0, 0)); }
        [[nodiscard]] cv::Mat last_R() const { return m_last_R;}
    protected:
        virtual void feature_detection(cv::Mat frame) = 0;
        cv::Vec2d SVD_offset(std::vector<cv::Point2f>& p1, std::vector<cv::Point2f>& p2);
        void draw_frame() const;
        cv::Vec2d m_offset;
        cv::Mat m_frame;
        cv::Mat m_draw_frame;
        SVD_MODE m_svd_mode = SVD_MODE::SVD_POSITIVE;
        cv::Mat m_R, m_last_R;
    };

    class FlowOdometry : public Odometry {

    public:
        using FeatureVector = std::vector<cv::Point2f>;
        FlowOdometry() = default;
        ~FlowOdometry() override = default;
        void feature_detection(cv::Mat frame) override;
        void process_frame(cv::Mat frame, bool draw = true);
    private:
        FeatureVector m_features;
        [[nodiscard]] std::pair<FeatureVector, FeatureVector> feature_matching(cv::Mat frame, const FeatureVector& points, bool draw) ;
        [[nodiscard]] cv::Vec2d get_offset(const FeatureVector& p1, const FeatureVector& p2) ;
        [[nodiscard]] cv::Vec2d compute_average_feature_offset(const FeatureVector& ref_features);
    };

    class ORBOdometry : public Odometry {
    public:
        using FeatureVector = std::vector<cv::KeyPoint>;
        ORBOdometry();
        ORBOdometry(double start_yaw);
        ~ORBOdometry() override = default;
        void feature_detection(cv::Mat frame) override;
        void process_frame(cv::Mat frame, bool draw = true);

    private:
        FeatureVector m_features;
        [[nodiscard]] std::pair<FeatureVector, FeatureVector> feature_matching(cv::Mat frame, FeatureVector& points, bool draw);
        [[nodiscard]] cv::Vec2d get_offset(const FeatureVector& p1, const FeatureVector& p2) ;
        [[nodiscard]] cv::Vec2d compute_average_feature_offset(const FeatureVector& ref_features) const ;

        cv::Ptr<cv::FeatureDetector> m_orb;
        cv::Ptr<cv::BFMatcher> m_matcher;
        cv::Mat m_descriptors;



    };
}//namespace nav
#endif //ODOMETRY_H
