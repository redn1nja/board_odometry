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

struct CommaSeparatedReader {
    template <typename... Args>
    static std::istream& read(std::istream& is, Args&... args) {
        std::string temp;
        ((std::getline(is, temp, ','), std::istringstream(temp) >> args), ...);
        is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return is;
    }
};

struct  CommaSeparatedWriter {
    template <typename... Args>
    static std::ostream& write(std::ostream& os, const Args&... args) {
        ((os << args << ","), ...);
        os << "\n";
        return os;
    }
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
    nav::ImageCorrection::Attitude attitude;
    LinAcc lin_acc;
    AngVel ang_vel;
    Offset offset;
};

constexpr std::string_view dev = "/dev/ttyACM0";

void signalHandler(int signum) {
    std::cout << "\nSIGINT received. Shutting down gracefully.\n";
    SerialConn::Disable();
}

double eucldean(const cv::Vec2d& a, const cv::Vec2d& b) {
    return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2));

}

template <typename T>
void experiment(std::ostream& out_file, nav::ROLL_MODE r, nav::PITCH_MODE p, nav::SVD_MODE s, cv::VideoCapture& cap, std::ifstream& file) {
    cv::Mat frame;
    nav::ImageProc<T> processor;
    processor.set_modes(r, p, s);
    Data data;
    std::array<std::string, 12> header;
    CommaSeparatedReader::read(file, header[0], header[1], header[2], header[3],
                               header[4], header[5], header[6], header[7],
                               header[8], header[9], header[10], header[11]);
    out_file << "[\n";
    while (cap.read(frame)) {
        try {
            CommaSeparatedReader::read(file, Data::id, data.attitude.roll, data.attitude.pitch,
                                   data.lin_acc.x, data.lin_acc.y, data.lin_acc.z,
                                   data.ang_vel.x, data.ang_vel.y, data.ang_vel.z,
                                   data.offset.x, data.offset.y, data.offset.z);
            cv::Vec2d gt = {data.offset.x, data.offset.y};
            processor.calclulate_offsets(frame, data.attitude, data.offset.z);
            std::cout << "Total Offset: " << processor.total_offset() << ", actual coordinates: " <<
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

    params p { nav::ROLL_MODE::ROLL_NEGATIVE, nav::PITCH_MODE::PITCH_POSITIVE, nav::SVD_MODE::SVD_POSITIVE};
    full_exp<nav::ORBOdometry>(p, cap_name, file_name, out_path, "ORB_Clear2.txt");
    // full_exp<nav::FlowOdometry>(p, cap_name, file_name, out_path, "Flow_Clear2.txt");

    return 0;

}