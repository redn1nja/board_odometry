#include <iostream>
#include "madgwick.h"
#include "csv.h"
#include <fstream>
#include "processor.h"

void exp(std::ifstream& file) {
    cv::RNG rng(12345);
    IMUStamped data;
    std::string line;
    std::array<std::string, 15> header;
    CommaSeparatedReader::read(file, header[0], header[1], header[2], header[3],
        header[4], header[5], header[6], header[7],
        header[8], header[9], header[10], header[11], header[12]);
    while (file >> line) {

        std::stringstream ss(line);
        nav::ImageCorrection::Attitude att;
        double time;
        int _ ;
        CommaSeparatedReader::read(ss, _, time, att.roll, att.pitch,
            data.imu.accX, data.imu.accY, data.imu.accZ,
            data.imu.gyroX, data.imu.gyroY, data.imu.gyroZ);
        data.imu.accX = -data.imu.accX;
        data.imu.accY = -data.imu.accY;
        data.imu.accZ= -data.imu.accZ;

        data.imu.gyroX = -data.imu.gyroX + rng.gaussian(0.2);
        data.imu.gyroY = -data.imu.gyroY + rng.gaussian(0.2);
        data.imu.gyroZ = -data.imu.gyroZ + rng.gaussian(0.2);


        data.time.sec = static_cast<uint32_t>(time);
        data.time.msec = static_cast<uint16_t>((time - static_cast<double>(data.time.sec))*1000);
        Quaternion q = madgwick_filter_update(&data);
        RPY attitude;
        quaternion_to_euler(&q, &attitude);
        nav::ImageCorrection::Attitude att2;
        att2.roll = attitude.roll;
        att2.pitch = attitude.pitch;
        att2.yaw = attitude.yaw;
        att-= att2;
        std::cout << "Diff: " << att.str() << "\n";
    }
}


int main(int argc, char** argv) {
    std::string filename = argv[1];
    std::ifstream file(filename);
    exp(file);

    return 0;
}