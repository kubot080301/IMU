#ifndef IMU_GY87_H_
#define IMU_GY87_H_

#include "imu.h"
#include <stdint.h>
#include <MPU6050.h>
#include <HMC5883L.h>

class GY87 : public IMU {
public:
    bool init();
    void get_data(float imu_data[9]);
private:
    MPU6050 mpu6050;
    HMC5883L mag;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
};

#endif
