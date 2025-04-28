#include <correction.h>
#include <opencv2/opencv.hpp>
#include <odometry.h>
#include <iostream>
#include <serial.h>
#include <thread>
#include <fstream>
#include <types.h>
#include <processor.h>
#include <filesystem>
#include "csv.h"
#include "times.h"
#include "angular.h"
#include "madgwick.h"
#include "structs.h"
#include "ma_buf.h"

enum class EXP_TYPE {
    GAZEBO,
    DATASET
};

double eucledean(const cv::Vec2d& a, const cv::Vec2d& b) {
    return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));
}


std::tuple<Data2, double, bool> read_gazebo(std::ifstream& file) {
    bool frame_rdy = false;
    Data2 data;
    static double last_stamp = 0;
    double current_stamp = 0;
    CommaSeparatedReader::read(file, Data2::id,  frame_rdy, current_stamp, data.attitude.roll, data.attitude.pitch, data.attitude.yaw,
                               data.imu.imu.accX, data.imu.imu.accY, data.imu.imu.accZ,
                               data.imu.imu.gyroX, data.imu.imu.gyroY, data.imu.imu.gyroZ,
                               data.offset.x, data.offset.y, data.offset.z);
    data.imu.imu.accZ *= -1;
    data.imu.imu.gyroZ *= -1;
    double dt = current_stamp - last_stamp;
    last_stamp = current_stamp;
    return {data, dt, frame_rdy};
}

std::tuple<Data2, double, bool> read_dataset(std::ifstream & file) {
    bool frame_rdy = true;
    Data2 data;
    double dt;
    CommaSeparatedReader::read(file, dt, data.offset.z, data.offset.x, data.offset.y,
        data.attitude.roll, data.attitude.pitch, data.attitude.yaw,
        data.imu.imu.accX, data.imu.imu.accY, data.imu.imu.accZ,
        data.imu.imu.gyroX, data.imu.imu.gyroY, data.imu.imu.gyroZ);

    data.imu.imu.accZ *= -1;
    data.imu.imu.gyroZ *= -1;
    return {data, dt, frame_rdy};
}

template <typename T>
void experiment(std::ostream& out_file, nav::ROLL_MODE r, nav::PITCH_MODE p, nav::SVD_MODE s, cv::VideoCapture& cap, std::ifstream& file, EXP_TYPE type = EXP_TYPE::GAZEBO) {
    cv::Mat frame;
    nav::ImageProc<T> processor(cv::Mat(), cv::Mat(), true);
    switch (type) {
        case EXP_TYPE::GAZEBO:{
            cv::Mat Q = (cv::Mat_<double>(2, 2) << 0.35, 0, 0, 0.3);
            cv::Mat R = (cv::Mat_<double>(2, 2) << 0.5, 0, 0, 0.5);
            processor = nav::ImageProc<T>(Q, R, true);
            break;
        }
        case EXP_TYPE::DATASET:{
            cv::Mat Q = (cv::Mat_<double>(2,2) << 0.1, 0, 0, 0.1);
            cv::Mat R = (cv::Mat_<double>(2,2) << 0.15, 0, 0, 0.35);
            processor = nav::ImageProc<T>(Q, R, false);
            processor.set_odom_R(0.3);
            break;
        }
    }
    processor.set_modes(r, p, s);
    std::array<std::string, 16> header;
    CommaSeparatedReader::read(file);
    CommaSeparatedWriter::write(out_file, "Total Offset X", "Total Offset Y",
                                            "Actual X", "Actual Y", "L2 error",
                                            "Roll", "Pitch", "Yaw",
                                            "Actual Roll", "Actual Pitch", "Actual Yaw", "Altitude");
    int i = 0;
    double time= 0;
    size_t start_it = 0;
    size_t stop_it = -1;
    while (cap.read(frame)) {
        try {
            i++;
            std::tuple<Data2, double, bool> data_dt;
            switch (type) {
                case EXP_TYPE::GAZEBO: {
                    data_dt = read_gazebo(file);
                    break;
                }
                case EXP_TYPE::DATASET: {
                    data_dt = read_dataset(file);
                    // cv::resize(frame, frame, cv::Size(720, 540));
                    std::get<Data2>(data_dt).attitude.roll = -rotate(std::get<Data2>(data_dt).attitude.roll);
                    std::get<Data2>(data_dt).attitude.yaw = -std::get<Data2>(data_dt).attitude.yaw - M_PI/2;

                    break;
                }

            }
            auto& [data, dt, frame_rdy] = data_dt;
            time+=dt;
            data.imu.time.sec = static_cast<uint32_t>(time);
            data.imu.time.msec = static_cast<uint16_t>((time - static_cast<double>(data.imu.time.sec))*1000);

            madgwick_filter_update(&data.imu);
            // while (!frame_rdy) {
            //     switch (type) {
            //         case EXP_TYPE::GAZEBO: {
            //             data_dt = read_gazebo(file);
            //             break;
            //         }
            //         case EXP_TYPE::DATASET: {
            //             data_dt = read_dataset(file);
            //             break;
            //         }
            //     }
            //     data = std::get<0>(data_dt);
            //     dt = std::get<1>(data_dt);
            //     frame_rdy = std::get<2>(data_dt);
            //     time+=dt;
            //
            //     data.imu.time.sec = static_cast<uint32_t>(time);
            //     data.imu.time.msec = static_cast<uint16_t>((time - static_cast<double>(data.imu.time.sec))*1000);
            //     madgwick_filter_update(&data.imu);
            // }
            nav::ImageCorrection::Attitude gt_attitude = {data.attitude.roll, data.attitude.pitch, data.attitude.yaw};

            data.attitude.roll = data.imu.rpy.roll;
            data.attitude.pitch = data.imu.rpy.pitch;

            if (i < start_it) {
                continue;
            }
            if (std::abs(i - static_cast<int>(start_it)) <=1  ) {
                processor.set_start_offset({data.offset.x, data.offset.y});
                processor.set_odom_R(-data.attitude.yaw);
            }

            // if (i % 2 == 0) {
            //     continue;
            // }


            cv::Vec3d acceleratiion = {-data.imu.imu.accX, data.imu.imu.accY, data.imu.imu.accZ};
            cv::Vec2d gt = {data.offset.x, data.offset.y};
            auto start = get_current_time_fenced();
            processor.calclulate_offsets(frame, data.attitude, acceleratiion, dt, data.offset.z);
            auto stop = get_current_time_fenced();
            std::cout << i << "th Total Offset: " << processor.total_offset() << ", actual coordinates: " <<
                gt << ", L2 error: " << eucledean(processor.total_offset(), gt) << ", Time processing frame: " << to_ms(stop-start) <<  "\n";
            auto off = processor.total_offset();
            CommaSeparatedWriter::write(out_file, off[0], off[1], gt[0], gt[1], eucledean(off, gt),
                data.attitude.roll, data.attitude.pitch, data.attitude.yaw,
                gt_attitude.roll, gt_attitude.pitch, gt_attitude.yaw, data.offset.z);
            if (i > stop_it) {
                break;
            }
        }
        catch (const std::exception& ex) {
            std::cerr << ex.what();
        }
    }

}

template <typename T>
void full_exp(params p, const std::string& cap_p, const std::string& file_p, const std::string& out_p, const std::string& out_path) {
    cv::VideoCapture cap(cap_p);
    std::ifstream file(file_p);
    std::filesystem::path path = out_p;
    path /= out_path;
    std::ofstream out_file(path);
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file\n";
        return;
    }
    if (!file.is_open()) {
        std::cerr << "Error opening file\n";
        return;
    }
    experiment<T>(out_file, p.r, p.p, p.s, cap, file);

    file.close();
    cap.release();
    out_file.close();

}


int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <video_path> <attitude_file> <output_dir>\n";
        return 1;
    }
    std::string cap_name = argv[1];
    std::string file_name = argv[2];
    std::string out_path = argv[3];
    params p { nav::ROLL_MODE::ROLL_NEGATIVE, nav::PITCH_MODE::PITCH_NEGATIVE , nav::SVD_MODE::SVD_POSITIVE};
    full_exp<nav::ORBOdometry>(p, cap_name, file_name, out_path, "3_4.csv");

    return 0;

}
