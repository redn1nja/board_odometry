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

template <typename T>
void experiment(std::ostream& out_file, nav::ROLL_MODE r, nav::PITCH_MODE p, nav::SVD_MODE s, cv::VideoCapture& cap, std::ifstream& file) {
    cv::Mat frame;
    cv::Mat Q = (cv::Mat_<double>(2,2) << 0.6, 0, 0, 0.2); // decreasing this increases the scale based on odometry
    cv::Mat R = (cv::Mat_<double>(2,2) << 0.2, 0, 0, 1); // increasing this increases the scale based on acceleration
    nav::ImageProc<T> processor(Q, R, false);
    processor.set_modes(r, p, s);
    Data data;
    std::array<std::string, 16> header;
    CommaSeparatedReader::read(file, header[0], header[1], header[2], header[3],
                               header[4], header[5], header[6], header[7],
                               header[8], header[9], header[10], header[11], header[12], header[13], header[14], header[15]);
    out_file << "[\n";
    double time = 0;
    double dt = 0;
    int i = 0;
    size_t start_it = 100;
    size_t stop_it = 1500;
    double base_yaw = 0;
    while (cap.read(frame)) {
        try {
            i++;
            IMUStamped imu;
            CommaSeparatedReader::read(file,dt, data.offset.z, data.offset.x, data.offset.y,
                data.attitude.roll, data.attitude.pitch, data.attitude.yaw,
                                   data.lin_acc.x, data.lin_acc.y, data.lin_acc.z,
                                      data.ang_vel.x, data.ang_vel.y, data.ang_vel.z, imu.imu.magX, imu.imu.magY, imu.imu.magZ);
            if (base_yaw == 0) {
                base_yaw = data.attitude.yaw;
            }
            time+=dt;
            imu.imu.accX = data.lin_acc.x;
            imu.imu.accY = data.lin_acc.y;
            imu.imu.accZ = -data.lin_acc.z;
            imu.imu.gyroX = data.ang_vel.x;
            imu.imu.gyroY = data.ang_vel.y;
            imu.imu.gyroZ = -data.ang_vel.z;
            imu.imu.magX = -imu.imu.magX;
            imu.imu.magY = -imu.imu.magY;
            imu.imu.magZ = -imu.imu.magZ;

            imu.time.sec = static_cast<uint32_t>(time);
            imu.time.msec = static_cast<uint16_t>((time - static_cast<double>(imu.time.sec))*1000);
            Quaternion q = madgwick_filter_update(&imu);
            RPY attitude;
            quaternion_to_euler(&q, &attitude);
            data.attitude.roll = -attitude.roll;
            data.attitude.pitch = attitude.pitch;
            data.attitude.yaw = attitude.yaw + base_yaw - (M_PI / 12);
            // if (i < 900 && i > 500) {
            //     data.attitude.yaw = data.attitude.yaw - (M_PI / 12);
            // }
            if (i < start_it) {
                continue;
            }
            if (i == start_it ) {
                processor.set_start_offset({data.offset.x, data.offset.y});
            }
            if (i % 3 != 0 ) {
                continue;
            }

            cv::Vec3d acceleratiion = {-data.lin_acc.x, -data.lin_acc.y, -data.lin_acc.z};
            cv::Vec2d gt = {data.offset.x, data.offset.y};
            auto start = get_current_time_fenced();
            processor.calclulate_offsets(frame, data.attitude, acceleratiion, dt, data.offset.z);
            auto stop = get_current_time_fenced();
            std::cout << i << "th Total Offset: " << processor.total_offset() << ", actual coordinates: " <<
                gt << ", L2 error: " << eucldean(processor.total_offset(), gt) << ", Time processing frame: " << to_ms(stop-start) <<  "\n";
            auto off = processor.total_offset();
            CommaSeparatedWriter::write(out_file, "[" + std::to_string(off[0]), off[1], gt[0], gt[1], std::to_string(eucldean(off, gt)) + "]");
            if (i > stop_it) {
                break;
            }
        }
        catch (const std::exception& ex) {
            std::cerr << ex.what();
        }
    }
    out_file << "]\n";
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
    params p { nav::ROLL_MODE::ROLL_NEGATIVE, nav::PITCH_MODE::PITCH_POSITIVE , nav::SVD_MODE::SVD_POSITIVE};
    full_exp<nav::ORBOdometry>(p, cap_name, file_name, out_path, "ORB_testMadgwick2.txt");
    // full_exp<nav::FlowOdometry>(p, cap_name, file_name, out_path, "Flow_testMadgwick.txt");

    return 0;

}
