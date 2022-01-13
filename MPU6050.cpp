#include "MPU6050.h"
#include "Arduino_print.h"

#define ADX_SCALE 0.00239420166015625  // 4mg/LSB    16/63356*9.80665
#define GRY_SCALE 0.001065264417860243  //rad/s   4000/65536*(PI/180)

#define WRITE_INTERVAL 50000
bool GY65::init() {
    Wire.begin();

    if (!mpu6050.testConnection()) {
        pb_printf("MPU6050 NOT FOUND!");
        return false;
    }

    I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80); // 設備復位
    delayMicroseconds(WRITE_INTERVAL);
    I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01); //設置設備時鐘源
    delayMicroseconds(WRITE_INTERVAL);
    I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);    //低通濾波 acc: 44hz, gyro: 42hz
    delayMicroseconds(WRITE_INTERVAL);
    I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);
    delayMicroseconds(WRITE_INTERVAL);
    I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000 << 3);
    delayMicroseconds(WRITE_INTERVAL);
    I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_8 << 3);
    delayMicroseconds(WRITE_INTERVAL);

    I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0X00);
    delayMicroseconds(WRITE_INTERVAL);
    I2Cdev::writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x02);
    delayMicroseconds(WRITE_INTERVAL);

    uint8_t buffer = 0;
    I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, &buffer);
    pb_printf("MPU6050_RA_PWR_MGMT_1=%d", buffer);
    I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, &buffer);
    pb_printf("MPU6050_RA_GYRO_CONFIG=%d", buffer);
    I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, &buffer);
    pb_printf("MPU6050_RA_ACCEL_CONFIG=%d", buffer);

    pb_printf("GY65 INIT SUCCESS!");

    return true;
}

void GY65::get_data(float imu_data[9]) {
    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#if IMU_DEBUG_ENABLE
    pb_printf("[%d %d %d] [%d %d %d]", ax, ay, az, gx, gy, gz);
#endif

    imu_data[0] = ax * ADX_SCALE;
    imu_data[1] = ay * ADX_SCALE;
    imu_data[2] = az * ADX_SCALE;
    imu_data[3] = gx * GRY_SCALE;
    imu_data[4] = gy * GRY_SCALE;
    imu_data[5] = gz * GRY_SCALE;
}