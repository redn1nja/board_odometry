#include <iostream>
#include "madgwick.h"
#include "csv.h"
#include <fstream>
#include "processor.h"
#include "opencv2/opencv.hpp"
#include "angular.h"

void exp(std::ifstream& file) {
    cv::RNG rng(12345);
    IMUStamped data;
    std::string line;
    std::array<std::string, 16> header;
    CommaSeparatedReader::read(file, header[0], header[1], header[2], header[3],
        header[4], header[5], header[6], header[7],
        header[8], header[9], header[10], header[11], header[12], header[13], header[14], header[15]);
    double time = 0;
    double base_yaw = 0;
    int i = 0;
    while (file >> line) {
        i++;
        std::stringstream ss(line);
        nav::ImageCorrection::Attitude att;
        double dt;
        int _1,_2,_3 ;
        CommaSeparatedReader::read(ss, dt, _1, _2,_3 ,att.roll, att.pitch, att.yaw,
            data.imu.accX, data.imu.accY, data.imu.accZ,
            data.imu.gyroX, data.imu.gyroY, data.imu.gyroZ, data.imu.magX, data.imu.magY, data.imu.magZ);
        dt*=2;
        att.roll = -rotate(att.roll);
        if (base_yaw == 0) {
            base_yaw = att.yaw;
        }
        time+=dt;
        // data.imu.accX = -data.imu.accX;
        // data.imu.accY = -data.imu.accY;
        data.imu.accZ = -data.imu.accZ;


        // data.imu.gyroX = -data.imu.gyroX;
        // data.imu.gyroY = -data.imu.gyroY;
        data.imu.gyroZ = -data.imu.gyroZ;

        data.imu.magX = -data.imu.magX;
        data.imu.magY = -data.imu.magY;
        data.imu.magZ = -data.imu.magZ;


        data.time.sec = static_cast<uint32_t>(time);
        data.time.msec = static_cast<uint16_t>((time - static_cast<double>(data.time.sec))*1000);
        Quaternion q = madgwick_filter_update(&data);
        RPY attitude;
        quaternion_to_euler(&q, &attitude);
        nav::ImageCorrection::Attitude att2;
        att2.roll = attitude.roll;
        att2.pitch = attitude.pitch;
        att2.yaw = attitude.yaw ;
        std::cout << "Attitude: " << att.str() << "\n" << "Madgwick: " << att2.str() << "\n";
        att-= att2;
        std::cout << "Diff: " << att.str() << "\n\n";
        if (i > 800) {
            break;
        }
    }
}


int main(int argc, char** argv) {
    std::string filename = argv[1];
    std::ifstream file(filename);
    exp(file);

    return 0;
}