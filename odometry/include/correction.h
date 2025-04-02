#ifndef CORRECTION_H
#define CORRECTION_H

#include <opencv2/opencv.hpp>

namespace nav {
    class ImageCorrection {
    public:
        struct Attitude {
            double roll;
            double pitch;
            Attitude() : roll(0), pitch(0) {}
            Attitude(double r, double p) : roll(r), pitch(p) {}
            Attitude& operator-=(const Attitude& rhs) {
                roll -= rhs.roll;
                pitch -= rhs.pitch;
                return *this;
            }

            Attitude& operator*= (double scalar) {
                roll *= scalar;
                pitch *= scalar;
                return *this;
            }

            Attitude& operator+=(const Attitude& rhs) {
                roll += rhs.roll;
                pitch += rhs.pitch;
                return *this;
            }

            [[nodiscard]] std::string str() const { return "Roll: " + std::to_string(roll) + " Pitch: " + std::to_string(pitch); }
        };

        struct CameraParams {
            double fx;
            double fy;
            double width;
            double height;
            double hfov;
            double vfov;

            CameraParams(double w, double h, double hfov, double vfov) :
            width(w), height(h),
            hfov(hfov), vfov(vfov)
            {
                fx = w / (std::tan(hfov / 2));
                fy = h / (std::tan(vfov / 2));
            }
            [[nodiscard]] cv::Mat get_intrinsic_matrix() const {
                return (cv::Mat_<double>(3, 3) << fx, 0, width / 2, 0, fy, height / 2, 0, 0, 1);
            }
        };
    private:
        std::deque<Attitude> m_buffer;
        void push_buffer(const Attitude& attitude);
        [[nodiscard]] Attitude get_expontial_average() const;
        [[nodiscard]] Attitude get_mov_average() const;
        static inline CameraParams m_K{960, 540, 1.2, 0.75};
        static constexpr double gamma = 0.9;
    public:
        explicit ImageCorrection() = default;
        CameraParams K() { return m_K;}
        [[nodiscard]] cv::Mat transform_frame(cv::Mat in_frame, const Attitude& attitude) ;

    };
}//namespace nav
#endif //CORRECTION_H
