#include <opencv2/opencv.hpp>
#include <odometry.h>
#include <iostream>
#include <serial.h>
#include <thread>

constexpr std::string_view dev = "/dev/ttyACM0";

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <video_path>\n";
        return 1;
    }
    try {
        std::string video_path = argv[1];
        cv::VideoCapture cap(video_path);
        Odometry odometry;
        SerialConn conn(dev.data(), 115200);
        auto conn_thread = conn.ThreadFn();
        if (!cap.isOpened()) {
            std::cerr << "Error opening video file\n";
            return 1;
        }
        cv::Mat frame;
        cap >> frame;

        odometry.FeatureDetection(frame);
        while (cap.read(frame)) {
            std::cout << conn.GetData();
            odometry.ProcessFrame(frame);
            if (cv::waitKey(30) >= 0) {
                break;
            }
        }
        conn_thread.join();
    }
    catch (const std::exception& ex) {
        std::cerr << "Unknown error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}