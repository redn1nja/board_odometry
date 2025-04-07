#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <opencv2/opencv.hpp>
namespace nav {

    class Odometry {
    public:
        virtual ~Odometry() = default;
    protected:
        virtual void feature_detection(cv::Mat frame) = 0;
        static cv::Vec2d SVD_offset(std::vector<cv::Point3f>& p1, std::vector<cv::Point3f>& p2);
        void draw_frame() const;
        cv::Vec2d m_offset;
        cv::Mat m_frame;
        cv::Mat m_draw_frame;
    };

    class FlowOdometry : public Odometry {

    public:
        using FeatureVector = std::vector<cv::Point2f>;
        FlowOdometry() = default;
        ~FlowOdometry() override = default;
        void feature_detection(cv::Mat frame) override;
        [[nodiscard]] cv::Vec2d offset() { return m_offset;}
        void process_frame(cv::Mat frame, bool draw = true);
    private:
        FeatureVector m_features;
        [[nodiscard]] std::pair<FeatureVector, FeatureVector> feature_matching(cv::Mat frame, const FeatureVector& points, bool draw) ;
        [[nodiscard]] static cv::Vec2d get_offset(const FeatureVector& p1, const FeatureVector& p2) ;
        [[nodiscard]] static cv::Vec2d compute_average_feature_offset(const FeatureVector& ref_features);
    };

    class ORBOdometry : public Odometry {
    public:
        using FeatureVector = std::vector<cv::KeyPoint>;
        ORBOdometry();
        ~ORBOdometry() override = default;
        void feature_detection(cv::Mat frame) override;
        [[nodiscard]] cv::Vec2d offset() { return m_offset;}
        void process_frame(cv::Mat frame, bool draw = true);
    private:
        FeatureVector m_features;
        cv::Vec2d m_offset;
        [[nodiscard]] std::pair<FeatureVector, FeatureVector> feature_matching(cv::Mat frame, const FeatureVector& points, bool draw) const;
        [[nodiscard]] static cv::Vec2d get_offset(const FeatureVector& p1, const FeatureVector& p2) ;
        [[nodiscard]] cv::Vec2d compute_average_feature_offset(const FeatureVector& ref_features) const ;

        cv::Ptr<cv::FeatureDetector> m_orb;
        cv::FlannBasedMatcher m_matcher;
        cv::Mat m_descriptors;

    };
}//namespace nav
#endif //ODOMETRY_H
