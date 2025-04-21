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



struct LinAcc {
    double x;
    double y;
    double z;
};

struct Offset {
    double x;
    double y;
    double z;
};

struct AngVel {
    double x;
    double y;
    double z;
};


struct Data {
    static inline size_t id=1;
    double ts;
    nav::ImageCorrection::Attitude attitude;
    LinAcc lin_acc;
    AngVel ang_vel;
    Offset offset;
};

constexpr std::string_view dev = "/dev/ttyACM0";

static double rotate(double ang) {
    return std::fmod(ang + 2 * M_PI, 2 * M_PI) - M_PI;
}

void signalHandler(int signum) {
    std::cout << "\nSIGINT received. Shutting down gracefully.\n";
    SerialConn::Disable();
}

double eucldean(const cv::Vec2d& a, const cv::Vec2d& b) {
    return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));

}

template <typename T>
void experiment2(std::ostream& out_file, nav::ROLL_MODE r, nav::PITCH_MODE p, nav::SVD_MODE s, cv::VideoCapture& cap, std::ifstream& file) {
    cv::Mat frame;
    cv::Mat Q = (cv::Mat_<double>(2, 2) << 0.35, 0, 0, 0.275);
    cv::Mat R = (cv::Mat_<double>(2, 2) << 0.5, 0, 0, 0.5);
    nav::ImageProc<T> processor(Q, R, false);
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
            processor.calclulate_offsets(frame, data.attitude, acceleratiion, dt, data.offset.z);
            std::cout << i << "th Total Offset: " << processor.total_offset() << ", actual coordinates: " <<
               gt << ", L2 error: " << eucldean(processor.total_offset(), gt) << "\n";
            auto off = processor.total_offset();
            CommaSeparatedWriter::write(out_file, "[" + std::to_string(off[0]), off[1], gt[0], gt[1], std::to_string(eucldean(off, gt)) + "]");
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
    cv::Mat Q = (cv::Mat_<double>(2,2) << 1, 0, 0, 0.225);
    cv::Mat R = (cv::Mat_<double>(2,2) << 0.5, 0, 0, 0.5);
    nav::ImageProc<T> processor(Q, R, false);
    processor.set_modes(r, p, s);
    Data data;
    std::array<std::string, 14> header;
    CommaSeparatedReader::read(file, header[0], header[1], header[2], header[3],
                               header[4], header[5], header[6], header[7],
                               header[8], header[9]);
    out_file << "[\n";
    double dt = 0;
    int i = 0;
    size_t start_it = 100;
    size_t stop_it = 850;
    while (cap.read(frame)) {

        try {
            i++;

            CommaSeparatedReader::read(file,dt, data.offset.z, data.offset.x, data.offset.y,
                data.attitude.roll, data.attitude.pitch, data.attitude.yaw,
                                   data.lin_acc.x, data.lin_acc.y, data.lin_acc.z);
            if (i < start_it) {
                continue;
            }
            if (i == start_it ) {
                processor.set_start_offset({data.offset.x, data.offset.y});
            }
            dt *= 2 ;
            data.attitude.roll = -rotate(data.attitude.roll);
            if (i < 500) {
                data.attitude.yaw = data.attitude.yaw + M_PI_2 ;
            }
            else if (i < 900) {
                data.attitude.yaw = data.attitude.yaw + M_PI_2 + (M_PI / 12);
            }
            // data.attitude.yaw += M_PI_2;
            cv::Vec3d acceleratiion = {-data.lin_acc.x, -data.lin_acc.y, -data.lin_acc.z};
            cv::Vec2d gt = {data.offset.x, data.offset.y};
            processor.calclulate_offsets(frame, data.attitude, acceleratiion, dt, data.offset.z);
            std::cout << i << "th Total Offset: " << processor.total_offset() << ", actual coordinates: " <<
                gt << ", L2 error: " << eucldean(processor.total_offset(), gt) << "\n";
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


struct params {
    nav::ROLL_MODE r;
    nav::PITCH_MODE p;
    nav::SVD_MODE s;
};

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
    params p { nav::ROLL_MODE::ROLL_NEGATIVE, nav::PITCH_MODE::PITCH_POSITIVE , nav::SVD_MODE::SVD_POSITIVE};
    full_exp<nav::ORBOdometry>(p, cap_name, file_name, out_path, "ORB_test2.txt");
    full_exp<nav::FlowOdometry>(p, cap_name, file_name, out_path, "Flow_test2.txt");

    return 0;

}