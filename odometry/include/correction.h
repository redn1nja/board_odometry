#ifndef CORRECTION_H
#define CORRECTION_H

#include <opencv2/opencv.hpp>


class ImageCorrection {

    struct Attitude {
        double roll;
        double pitch;

        Attitude& operator-=(const Attitude& rhs) {
            roll -= rhs.roll;
            pitch -= rhs.pitch;
            return *this;
        }
    };
    cv::Mat m_ref_frame;
    Attitude m_attitude;
    void update_state(cv::Mat ref_frame, const Attitude& attitude) {
        m_ref_frame = ref_frame;
        m_attitude = attitude;
    };
public:
    explicit ImageCorrection(cv::Mat ref_frame): m_ref_frame(ref_frame) {};
    [[nodiscard]] cv::Mat transform_frame(cv::Mat in_frame, const Attitude& attitude) const;
    [[nodiscard]] cv::Mat get_frame() { return m_ref_frame;}
};

#endif //CORRECTION_H
