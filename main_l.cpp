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
#include "structs.h"

double eucledean(const cv::Vec2d& a, const cv::Vec2d& b) {
    return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));
}

template <typename T>
void experiment2(std::ostream& out_file, nav::ROLL_MODE r, nav::PITCH_MODE p, nav::SVD_MODE s, cv::VideoCapture& cap, std::ifstream& file) {
    cv::Mat frame;
    cv::Mat Q = (cv::Mat_<double>(2, 2) << 0.35, 0, 0, 0.3);
    cv::Mat R = (cv::Mat_<double>(2, 2) << 0.5, 0, 0, 0.5);
    nav::ImageProc<T> processor(Q, R, true);
    processor.set_modes(r, p, s);
    Data data;
    std::array<std::string, 14> header;
    CommaSeparatedReader::read(file, header[0], header[1], header[2], header[3],
                               header[4], header[5], header[6], header[7],
                               header[8], header[9], header[10], header[11], header[12], header[13]);
    out_file << "[\n";
    double dt = 0;
    double last_stamp = 0;
    double stamp = 0;
    int i = 0;

    while (cap.read(frame)) {
        try {
            i++;
            CommaSeparatedReader::read(file, Data::id, stamp, data.attitude.roll, data.attitude.pitch, data.attitude.yaw,
                data.lin_acc.x, data.lin_acc.y, data.lin_acc.z,
                data.ang_vel.x, data.ang_vel.y, data.ang_vel.z,
                data.offset.x, data.offset.y, data.offset.z);
            dt = stamp - last_stamp;
            last_stamp = stamp;
            cv::Vec3d acceleratiion = {-data.lin_acc.x, -data.lin_acc.y, -data.lin_acc.z};
            cv::Vec2d gt = {data.offset.x, data.offset.y};
            auto start = get_current_time_fenced();
            processor.calclulate_offsets(frame, data.attitude, acceleratiion, dt, data.offset.z);
            auto stop = get_current_time_fenced();
            std::cout << i << "th Total Offset: " << processor.total_offset() << ", Yaw: " << data.attitude.yaw << ", actual coordinates: " <<
               gt << ", L2 error: " << eucledean(processor.total_offset(), gt) << ", Time processing frame: " << to_ms(stop-start) << "\n";
            auto off = processor.total_offset();
            CommaSeparatedWriter::write(out_file, "[" + std::to_string(off[0]), off[1], gt[0], gt[1], std::to_string(eucledean(off, gt)) + "]");
        }
        catch (const std::exception& ex) {
            std::cerr << ex.what();
        }
    }
    out_file << "]\n";

}


template <typename T>
void experiment(std::ostream& out_file, nav::ROLL_MODE r, nav::PITCH_MODE p, nav::SVD_MODE s, cv::VideoCapture& cap, std::ifstream& file) {
    cv::Mat frame;
    cv::Mat Q = (cv::Mat_<double>(2,2) << 0.1, 0, 0, 0.1);
    cv::Mat R = (cv::Mat_<double>(2,2) << 0.05, 0, 0, 0.25);
    nav::ImageProc<T> processor(Q, R, true);
    processor.set_modes(r, p, s);
    Data data;
    cv::Vec2d last_gt = {0,0};
    std::array<std::string, 14> header;
    CommaSeparatedReader::read(file, header[0], header[1], header[2], header[3],
                               header[4], header[5], header[6], header[7],
                               header[8], header[9]);
    out_file << "[\n";
    double dt = 0;
    int i = 0;
    size_t start_it = 400;
    size_t stop_it = 1300;
    while (cap.read(frame)) {

        try {
            i++;
            cv::resize(frame, frame, cv::Size(720, 540));
            CommaSeparatedReader::read(file,dt, data.offset.z, data.offset.x, data.offset.y,
                data.attitude.roll, data.attitude.pitch, data.attitude.yaw,
                                   data.lin_acc.x, data.lin_acc.y, data.lin_acc.z);

            if (i < start_it) {
                continue;
            }
            if (i == start_it ) {
                processor.set_start_offset({data.offset.x, data.offset.y});
                processor.set_odom_R(0.15);
            }
            // if(i > 900 && i < 1200 ) {
            //     cv::imshow("Frame", frame);
            //     cv::waitKey(30);
            // }
            // data.attitude.roll = rotate(data.attitude.roll);
            auto yaw = data.attitude.yaw;
            cv::Vec3d acceleratiion = {data.lin_acc.x, data.lin_acc.y, -data.lin_acc.z};
            cv::Vec2d gt = {data.offset.x, data.offset.y};
            auto start = get_current_time_fenced();
            processor.calclulate_offsets(frame, data.attitude, acceleratiion, dt, data.offset.z);
            auto stop = get_current_time_fenced();
            std::cout << i << "th Total Offset: " << processor.total_offset() << ", actual coordinates: " <<
                gt << ", L2 error: " << eucledean(processor.total_offset(), gt) << ", actual offset: " <<
                    gt - last_gt << ", Time processing frame: " << to_ms(stop-start) << "\n"
            << "Yaw: " << processor.R() << ", GT yaw: " << yaw + M_PI/2 << "\n";
            auto off = processor.total_offset();
            last_gt = gt;
            CommaSeparatedWriter::write(out_file, "[" + std::to_string(off[0]), off[1], gt[0], gt[1], std::to_string(eucledean(off, gt)) + "]");
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
    // params p_n {nav::ROLL_MODE::ROLL_POSITIVE, nav::PITCH_MODE::PITCH_POSITIVE, nav::SVD_MODE::SVD_NEGATIVE};
    params p { nav::ROLL_MODE::ROLL_POSITIVE, nav::PITCH_MODE::PITCH_POSITIVE , nav::SVD_MODE::SVD_POSITIVE};
    full_exp<nav::ORBOdometry>(p, cap_name, file_name, out_path, "ORB_testaffine.txt");
    // full_exp<nav::FlowOdometry>(p, cap_name, file_name, out_path, "Flow_test2affine.txt");

    return 0;

}