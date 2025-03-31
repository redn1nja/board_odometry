#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <csignal>
#include <chrono>

struct Attitude {
    double roll;
    double pitch;
};

cv::Mat frame;
Attitude attitude;
std::ofstream output;
cv::VideoWriter writer;
volatile sig_atomic_t run = 1;
std::atomic_bool data_rdy = false;

void signal_handler(int signum) {
    std::cout << "\nSIGINT received. Shutting down gracefully.\n";
    run = 0;
}

void camera_callback(const gz::msgs::Image& msg) {
    frame = cv::Mat(msg.height(), msg.width(), CV_8UC3, const_cast<char*>(msg.data().data()));
    cv::imshow("Frame", frame);
    writer.write(frame);
    data_rdy = true;
    cv::waitKey(1);
}

void imu_callback(const gz::msgs::IMU& msg) {
    if (!data_rdy) return;
    const auto& quat = msg.orientation();
    attitude.roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()));
    attitude.pitch = asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
    std::string stamp(std::to_string(msg.header().stamp().sec()) + "." + std::to_string(msg.header().stamp().nsec() / 1000));
    output << attitude.roll << " " << attitude.pitch << std::endl;
    data_rdy = false;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <topics.txt> <output.txt> <output.avi>\n";
        return 1;
    }
    std::signal(SIGINT, signal_handler);
    std::ifstream topics(argv[1]);
    output.open(argv[2]);
    writer.open(argv[3], cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(640, 480));
    std::string cam_topic, imu_topic;
    topics >> cam_topic >> imu_topic;
    topics.close();
    gz::transport::Node node;
    node.Subscribe<gz::msgs::Image>(cam_topic, camera_callback);
    node.Subscribe<gz::msgs::IMU>(imu_topic, imu_callback);
    while (run){}
    output.close();
    writer.release();
    return 0;
}