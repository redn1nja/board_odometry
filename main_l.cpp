#include <opencv2/opencv.hpp>
#include <odometry.h>
#include <iostream>
#include <serial.h>
#include <thread>

constexpr std::string_view dev = "/dev/ttyACM0";

void signalHandler(int signum) {
    std::cout << "\nSIGINT received. Shutting down gracefully.\n";
    SerialConn::Disable();
}

int main(int argc, char** argv) {
#if 0
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <video_path>\n";
        return 1;
    }
    try {
        std::string video_path = argv[1];
        cv::VideoCapture cap(video_path);
        Odometry odometry;
        // SerialConn conn(dev.data(), 115200);
        // auto conn_thread = conn.ThreadFn();
        if (!cap.isOpened()) {
            std::cerr << "Error opening video file\n";
            return 1;
        }
        cv::Mat frame;
        cap >> frame;

        odometry.FeatureDetection(frame);
        while (cap.read(frame)) {
            // std::cout << conn.GetData();
            odometry.ProcessFrame(frame);
            if зроблений (cv::waitKey(30) >= 0) {
                break;
            }
        }

        // conn_thread.join();
    }
    catch (const std::exception& ex) {
        std::cerr << "Unknown error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
#endif

    Odometry odometry;
    cv::Mat frame1 = cv::imread("assets/A1.png");
    cv::Mat frame2 = cv::imread("assets/A2.png");


    frame1 = frame1(cv::Rect(0, 0, 1000, 700));
    frame2 = frame2(cv::Rect(0, 0, 1000, 700));

    cv::imshow("Frame", frame1);
    cv::waitKey(1000);

    odometry.FeatureDetection(frame1);
    cv::Mat frame2_copy = frame2.clone();
    odometry.ProcessFrame(frame2_copy);

    std::cout << "Frame2:" << std::endl;
    cv::imshow("Frame", frame2);
    cv::waitKey(1000);

    return 0;

}