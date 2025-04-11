#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <csignal>
#include <chrono>

struct Attitude {
    double roll;
    double pitch;
};

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
    Attitude attitude;
    LinAcc lin_acc;
    AngVel ang_vel;
    Offset offset;
};

class CommaSeparatedWriter {
public:
    template <typename... Args>
    static std::ostream& write(std::ostream& os, const Args&... args) {
        ((os << args << ","), ...);
        os << "\n";
        return os;
    }
};

cv::Mat frame;
Data data;
std::ofstream output;
cv::VideoWriter writer;
volatile sig_atomic_t run = 1;
std::atomic_bool image_rdy = false;
std::atomic_bool imu_rdy = false;
std::atomic_bool pose_rdy = false;


void signal_handler(int signum) {
    std::cout << "\nSIGINT received. Shutting down gracefully.\n";
    run = 0;
}

void camera_callback(const gz::msgs::Image& msg) {
    frame = cv::Mat(msg.height(), msg.width(), CV_8UC3, const_cast<char*>(msg.data().data()));
    cv::imshow("Frame", frame);
    writer.write(frame);
    image_rdy = true;
    cv::waitKey(1);
}

void imu_callback(const gz::msgs::IMU& msg) {
    if (!image_rdy) return;
    const auto& quat = msg.orientation();

    data.attitude.roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()));
    data.attitude.pitch = asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));

    data.lin_acc.x = msg.linear_acceleration().x();
    data.lin_acc.y = msg.linear_acceleration().y();
    data.lin_acc.z = msg.linear_acceleration().z();

    data.ang_vel.x = msg.angular_velocity().x();
    data.ang_vel.y = msg.angular_velocity().y();
    data.ang_vel.z = msg.angular_velocity().z();

    imu_rdy = true;
}

void pose_callback(const gz::msgs::Pose_V& msg) {
    if (!image_rdy) return;

    data.offset.x = msg.pose(82).position().x();
    data.offset.y = msg.pose(82).position().y();
    data.offset.z = msg.pose(82).position().z();

    pose_rdy = true;
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
    std::string cam_topic, imu_topic, pose_topic;
    topics >> cam_topic >> imu_topic >> pose_topic;
    topics.close();
    gz::transport::Node node;
    node.Subscribe<gz::msgs::Image>(cam_topic, camera_callback);
    node.Subscribe<gz::msgs::IMU>(imu_topic, imu_callback);
    node.Subscribe<gz::msgs::Pose_V>(pose_topic, pose_callback);
    CommaSeparatedWriter::write(output, "ID", "Roll", "Pitch", "LinAccX", "LinAccY", "LinAccZ",
                                "AngVelX", "AngVelY", "AngVelZ", "PoseX", "PoseY", "PoseZ");
    while (run) {
        if(image_rdy && imu_rdy && pose_rdy) {
            CommaSeparatedWriter::write(output, Data::id++, data.attitude.roll, data.attitude.pitch,
                                        data.lin_acc.x, data.lin_acc.y, data.lin_acc.z,
                                        data.ang_vel.x, data.ang_vel.y, data.ang_vel.z,
                                        data.offset.x, data.offset.y, data.offset.z);
            image_rdy = false;
            imu_rdy = false;
            pose_rdy = false;
        }
    }
    output.close();
    writer.release();
    return 0;
}