#ifndef STRUCTS_H
#define STRUCTS_H
#include "../external/madgwick/Core/Inc/imu_filter/imu_data.h"

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

struct Data2 {
    static inline size_t id=1;
    double ts;
    nav::ImageCorrection::Attitude attitude;
    IMUStamped imu;
    Offset offset;
};

struct params {
    nav::ROLL_MODE r;
    nav::PITCH_MODE p;
    nav::SVD_MODE s;
};
#endif //STRUCTS_H
