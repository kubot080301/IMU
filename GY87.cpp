#include "GY87.h"
#include "Arduino_print.h"

#define ADXL345_SCALE 0.0392266  // 4mg/LSB    4*9.80665
#define ITG3205_SCALE 0.001065264417860243  //rad/s   4000/65536*(PI/180)
#define HMC5883L_SCALE 0.92  // mG/LSb

#define WRITE_INTERVAL 20
bool GY87::init() {
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

    delayMicroseconds(WRITE_INTERVAL);
    if (!mag.testConnection()) {
        pb_printf("HMC5883L NOT FOUND!");
        return false;
    }

    mag.initialize();
    delayMicroseconds(WRITE_INTERVAL);
    mag.setGain(HMC5883L_GAIN_1090);
    delayMicroseconds(WRITE_INTERVAL);
    mag.setDataRate(HMC5883L_RATE_75);
    delayMicroseconds(WRITE_INTERVAL);
    mag.setSampleAveraging(HMC5883L_AVERAGING_1);
    delayMicroseconds(WRITE_INTERVAL);
    mag.setMode(HMC5883L_MODE_SINGLE);
    delayMicroseconds(WRITE_INTERVAL);

    I2Cdev::readByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, &buffer);
    pb_printf("HMC5883L_RA_CONFIG_B=%d", buffer);
    I2Cdev::readByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, &buffer);
    pb_printf("HMC5883L_RA_CONFIG_A=%d", buffer);
    I2Cdev::readByte(HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, &buffer);
    pb_printf("HMC5883L_RA_MODE=%d", buffer);

    pb_printf("GY87 INIT SUCCESS!");

    return true;
}

void GY87::get_data(float imu_data[9]) {
    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    mag.getHeading(&mx, &my, &mz);
    mag.setMode(HMC5883L_MODE_SINGLE);

#if IMU_DEBUG_ENABLE
    pb_printf("[%d %d %d] [%d %d %d] [%d %d %d]", ax, ay, az, gx, gy, gz, mx, my, mz);
#endif

    imu_data[0] = ax * ADXL345_SCALE;
    imu_data[1] = ay * ADXL345_SCALE;
    imu_data[2] = az * ADXL345_SCALE;
    imu_data[3] = gx * ITG3205_SCALE;
    imu_data[4] = gy * ITG3205_SCALE;
    imu_data[5] = gz * ITG3205_SCALE;
    imu_data[6] = mx * HMC5883L_SCALE;
    imu_data[7] = my * HMC5883L_SCALE;
    imu_data[8] = mz * HMC5883L_SCALE;
}