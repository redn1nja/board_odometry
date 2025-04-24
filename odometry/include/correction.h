#ifndef CORRECTION_H
#define CORRECTION_H

#include <optional>
#include <opencv2/opencv.hpp>

namespace nav {

    enum class ROLL_MODE : short;
    enum class PITCH_MODE: short;
    enum class SVD_MODE : short;

    class ImageCorrection {
    public:
        struct Attitude {
            double roll;
            double pitch;
            double yaw;
            Attitude() : roll(0), pitch(0), yaw(0) {}
            Attitude(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}
            Attitude& operator-=(const Attitude& rhs) {
                roll -= rhs.roll;
                pitch -= rhs.pitch;
                yaw -= rhs.yaw;
                return *this;
            }

            Attitude& operator*= (double scalar) {
                roll *= scalar;
                pitch *= scalar;
                yaw *= scalar;
                return *this;
            }

            Attitude& operator+=(const Attitude& rhs) {
                roll += rhs.roll;
                pitch += rhs.pitch;
                yaw += rhs.yaw;
                return *this;
            }

            [[nodiscard]] std::string str() const { return "Roll: " + std::to_string(roll) + " Pitch: " + std::to_string(pitch) + " Yaw: " + std::to_string(yaw); }
        };

        struct CameraParams {
            double fx;
            double fy;
            double width;
            double height;
            double hfov;
            double vfov;
            cv::Mat m_K;
            std::optional<cv::Mat> m_D;
            CameraParams(cv::Mat K, cv::Mat D) {
                std::cout << "Using camera params from dataset\n";
                m_K = K.clone();
                m_D = D.clone();
                fx = K.at<double>(0, 0);
                fy = K.at<double>(1, 1);
                width = 720;
                height = 540;
                hfov = 2 * std::atan(width / (2 * fx));
                vfov = 2 * std::atan(height / (2 * fy));

            }
            CameraParams(double w, double h, double hfov, double vfov) :
            width(w), height(h),
            hfov(hfov), vfov(vfov)
            {
                fx = w / (2 *std::tan(hfov / 2));
                fy = h / (2 *std::tan(vfov / 2));
                m_K = (cv::Mat_<double>(3, 3) << fx, 0, width / 2, 0, fy, height / 2, 0, 0, 1);
                m_D = std::nullopt;
            }
            [[nodiscard]] cv::Mat get_intrinsic_matrix() const {
                return m_K;
            }

            [[nodiscard]] std::optional<cv::Mat> get_distortion_matrix() const {
                return m_D;
            }
        };
    private:
        std::deque<Attitude> m_buffer;
        void push_buffer(const Attitude& attitude);
        [[nodiscard]] Attitude get_expontial_average() const;
        [[nodiscard]] Attitude get_mov_average() const;
#ifndef DATASET_K
        static inline CameraParams m_K{640, 480, 1.57, 1.29};
#else
        static inline CameraParams m_K{(cv::Mat_<double>(3,3) << 854.383024/2, 0, 780.324522/2, 0, 853.285954/2, 520.690672/2, 0, 0, 1),
                (cv::Mat_<double>(1, 4) << -0.07937700, 0.02228435, -0.03852023, 0.01346873)};
#endif

        static constexpr double gamma = 0.9;
        ROLL_MODE m_roll_mode;
        PITCH_MODE m_pitch_mode;
    public:
        explicit ImageCorrection() = default;
        CameraParams K() { return m_K;}
        [[nodiscard]] cv::Mat transform_frame(cv::Mat in_frame, const Attitude& attitude);

        void set_modes(ROLL_MODE r, PITCH_MODE p) {
            m_roll_mode = r;
            m_pitch_mode = p;
        }
        [[nodiscard]] cv::Mat get_intrinsic_matrix() const {
            return m_K.get_intrinsic_matrix();

        }
    };
}//namespace nav
#endif //CORRECTION_H
