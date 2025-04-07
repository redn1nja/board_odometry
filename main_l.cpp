#include <correction.h>
#include <opencv2/opencv.hpp>
#include <odometry.h>
#include <iostream>
#include <serial.h>
#include <thread>
#include <fstream>
#include <types.h>
#include <processor.h>

constexpr std::string_view dev = "/dev/ttyACM0";

void signalHandler(int signum) {
    std::cout << "\nSIGINT received. Shutting down gracefully.\n";
    SerialConn::Disable();
}

nav::ImageCorrection::Attitude parse_str(const std::string& str) {
    auto space = str.find(' ');
    double roll = std::stod(str.substr(0, space));
    double pitch = std::stod(str.substr(space + 1));
    return {-roll, -pitch};
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <video_path> <attitude_file>\n";
        return 1;
    }

    cv::VideoCapture cap(argv[1]);
    std::ifstream file(argv[2]);

    if (!cap.isOpened() || !file.is_open()) {
        std::cerr << "Error opening video file\n";
        return 1;
    }
    cv::Mat frame;
    std::string line;
    nav::ImageProc<nav::ORBOdometry> processor;
    while (cap.read(frame)) {
        std::getline(file, line);
        auto attitude = parse_str(line);
        processor.calclulate_offsets(frame, attitude, 10);
        std::cout << "Total Offset: " << processor.total_offset() << "\n";
    }
    cap.release();
    return 0;

}