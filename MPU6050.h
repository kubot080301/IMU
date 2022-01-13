#ifndef IMU_GY65_H_
#define IMU_GY65_H_

#include "imu.h"
#include <stdint.h>
#include <MPU6050.h>

class GY65 : public IMU {
public:
    bool init();
    void get_data(float imu_data[9]);
private:
    MPU6050 mpu6050;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

#endif