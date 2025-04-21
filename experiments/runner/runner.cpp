#include <madgwick.h>
#include <processor.h>
#include <quat.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <opencv2/opencv.hpp>
#include "csv.h"
#include <fstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "correction.h"


class RogsbagRunner : public rclcpp::Node {
public:
    explicit RogsbagRunner()
        : Node("rosbag_runnder") {
        using std::placeholders::_1;
        RCLCPP_INFO(this->get_logger(), "Rosbag to Dataset Node Initialized");
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
            std::bind(&RogsbagRunner::imu_callback, this, _1));

        image_sub_ = create_subscription<sensor_msgs::msg::Image>("/camera/image_color", 10,
            std::bind(&RogsbagRunner::image_callback, this, _1));

        odom_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10,
            std::bind(&RogsbagRunner::odometry_callback, this, _1));
        visual_odom_ = nav::ImageProc<nav::ORBOdometry>();
        visual_odom_.set_modes(nav::ROLL_MODE::ROLL_POSITIVE, nav::PITCH_MODE::PITCH_POSITIVE, nav::SVD_MODE::SVD_NEGATIVE);
        dt = 0;
        base_yaw = 0;
        m_file.open("output.csv");
        CommaSeparatedWriter::write(m_file, "dt", "Altitude", "GTX", "GTY", "Roll", "Pitch", "Yaw",
            "AccX", "AccY", "AccZ");
        m_dcm = cv::Mat::eye(3, 3, CV_64F);
        writer.open("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(1440, 1080));
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr pdata) {
        // auto imu = from_msg(pdata);
        // q_data_ = madgwick_filter_update(&imu);
        // attitude_ = quat_to_attitude(&q_data_);

        accel_ = to_vec3d(pdata);
        tf2::Quaternion q;
        tf2::fromMsg(pdata->orientation, q);
        tf2::Matrix3x3 m(q);
        m.getRPY(m_rpy[0], m_rpy[1], m_rpy[2]);
        attitude_.roll = m_rpy[0];
        attitude_.pitch = m_rpy[1];
        attitude_.yaw =  m_rpy[2];

    }

    static inline double rotate(double ang) {
        return std::fmod(ang + 2 * M_PI, 2 * M_PI) - M_PI;
    }

    static inline double wrap(double ang, double max) {
        return std::fmod(max + std::fmod(ang, max), max);
    }

    static inline double wrap_pi(double ang) {
        return -M_PI + wrap(ang + M_PI, 2 * M_PI);
    }

    cv::VideoWriter writer;

    void image_callback(sensor_msgs::msg::Image::SharedPtr pdata) {

        if (stamp_.sec == 0) {
            stamp_ = pdata->header.stamp;
            return;

        }
        dt = (static_cast<double>(pdata->header.stamp.sec) + static_cast<double>(pdata->header.stamp.nanosec) * 1e-9) -
            (static_cast<double>(stamp_.sec) + static_cast<double>(stamp_.nanosec) * 1e-9);
        stamp_ = pdata->header.stamp;

        auto data = pdata->data;
        cv::Mat frame = cv::Mat(pdata->height, pdata->width, CV_8UC3, (void *)pdata->data.data(), pdata->step);

        if (m_gt == nullptr) {
            RCLCPP_WARN(get_logger(), "No GT");
            return;
        }

        // auto off = visual_odom_.calclulate_offsets(frame, attitude_, accel_, dt, m_gt->z);
        print_errors(attitude_, visual_odom_.total_offset(), {});
        writer.write(frame);

        // if(i > 300) {
        //     m_file.close();
        //     throw std::runtime_error("End of test");
        // }


    }

    void odometry_callback(sensor_msgs::msg::NavSatFix::SharedPtr pdata) {
        RCLCPP_INFO(get_logger(), "Received GT");
        if (sat_base == nullptr) {

            double lat = pdata->latitude * M_PI / 180;
            double lon = pdata->longitude * M_PI / 180;

            double N = alpha_r / std::sqrt(1 - e2 * std::pow(std::sin(lat), 2));
            sat_base = std::make_shared<geometry_msgs::msg::Vector3>();
            sat_base->x = (N + pdata->altitude) * std::cos(lat) * std::cos(lon);
            sat_base->y = (N + pdata->altitude) * std::cos(lat) * std::sin(lon);
            sat_base->z = ( (1 - e2 ) * N + pdata->altitude) * std::sin(lat);

            // dcm to ENU frame
            m_dcm = (cv::Mat_<double>(3,3) << - std::sin(lon), std::cos(lon), 0,
                -std::sin(lat)*std::cos(lon), -std::sin(lat)*std::sin(lon), std::cos(lat),
                std::cos(lat)*std::cos(lon), std::cos(lat)*std::sin(lon), std::sin(lat));


        }


        double lat = pdata->latitude * M_PI / 180;
        double lon = pdata->longitude * M_PI / 180;
        cv::Vec3d ge_geo;
        double N = alpha_r / std::sqrt(1 - e2 * std::pow(std::sin(lat), 2));
        ge_geo[0] = (N + pdata->altitude) * std::cos(lat) * std::cos(lon) - sat_base->x;
        ge_geo[1] = (N + pdata->altitude) * std::cos(lat) * std::sin(lon) - sat_base->y;
        ge_geo[2] = ((1-e2) * N + pdata->altitude) * std::sin(lat) - sat_base->z;
        cv::Mat geo = cv::Mat(ge_geo).reshape(1);
        geo.convertTo(geo, CV_64F);
        cv::Mat enu = m_dcm * geo;
        m_gt = std::make_shared<geometry_msgs::msg::Vector3>();
        m_gt->x = enu.at<double>(1, 0);
        m_gt->y = enu.at<double>(0, 0);
        m_gt->z = enu.at<double>(2, 0);

    }


private:
    size_t i = 0;
    static constexpr double alpha_r = 6378000;
    static constexpr double beta_r = 6357000;
    static constexpr double e2 = 1 - (std::pow(beta_r, 2) / std::pow(alpha_r, 2));
    static constexpr double f = 1 - (beta_r / alpha_r);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr odom_sub_;


    std::ofstream m_file;
    Quaternion q_data_;
    cv::Vec3d m_rpy;
    nav::ImageProc<nav::ORBOdometry> visual_odom_;
    nav::ImageCorrection::Attitude attitude_;
    cv::Vec3d accel_;
    double dt;
    double base_yaw;
    cv::Mat m_dcm;
    geometry_msgs::msg::Vector3::SharedPtr sat_base;
    geometry_msgs::msg::Vector3::SharedPtr m_gt;
    builtin_interfaces::msg::Time stamp_;

    static IMUStamped from_msg(sensor_msgs::msg::Imu::SharedPtr pdata) {
        IMUStamped imu;
        imu.imu.accX = pdata->linear_acceleration.x;
        imu.imu.accY = pdata->linear_acceleration.y;
        imu.imu.accZ = pdata->linear_acceleration.z;
        imu.imu.gyroX = pdata->angular_velocity.x;
        imu.imu.gyroY = pdata->angular_velocity.y;
        imu.imu.gyroZ = pdata->angular_velocity.z;

        imu.time.sec = pdata->header.stamp.sec;
        imu.time.msec = pdata->header.stamp.nanosec * (1e-6);
        return imu;
    }

    static nav::ImageCorrection::Attitude quat_to_attitude(Quaternion* q) {
        RPY euler;
        quaternion_to_euler(q, &euler);
        nav::ImageCorrection::Attitude attitude;
        attitude.pitch = euler.pitch;
        attitude.roll = euler.roll;
        attitude.yaw = euler.yaw;
        return attitude;
    }

    static cv::Vec3d to_vec3d(sensor_msgs::msg::Imu::SharedPtr pdata) {
        return {-pdata->linear_acceleration.x, -pdata->linear_acceleration.y, pdata->linear_acceleration.z};
    }

    void print_errors(const nav::ImageCorrection::Attitude& att, const cv::Vec2d& offset, const cv::Vec2d & off) {
        cv::Vec2d gt = {m_gt->x, m_gt->y};
        double error = std::sqrt(std::pow(gt[0] - offset[0], 2) +  std::pow(gt[1] - offset[1], 2));
        std::stringstream ss;
        ss << "dt" << dt << ", Offset: " << offset << ", Alt: " << m_gt->z << " GT offset: " << gt << " Error: " << error << ".\n Attitude: " << att.str() << "\n";
        RCLCPP_INFO (get_logger(), ss.str().c_str());
        CommaSeparatedWriter::write(m_file,
            dt,
            m_gt->z,
            gt[0],
            gt[1],
            att.roll,
            att.pitch,
            att.yaw,
            accel_[0],
            accel_[1],
            accel_[2]
        );
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RogsbagRunner>();
    spin(node);
    return 0;
}