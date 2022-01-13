#include "GY85.h"
#include "Arduino_print.h"

#define ADXL345_SCALE 0.0392266  // 4mg/LSB    4*9.80665
#define ITG3205_SCALE 0.001065264417860243  //rad/s   4000/65536*(PI/180)
#define HMC5883L_SCALE 0.92  // mG/LSb

#define WRITE_INTERVAL 20
bool GY85::init() {
    Wire.begin();

    if (!accel.testConnection()) {
        pb_printf("ADXL345 NOT FOUND!");
        return false;
    }

    accel.initialize();
    delayMicroseconds(WRITE_INTERVAL);
    accel.setAutoSleepEnabled(false);
    delayMicroseconds(WRITE_INTERVAL);
    accel.setFullResolution(1);
    delayMicroseconds(WRITE_INTERVAL);
    accel.setRange(ADXL345_RANGE_4G);
    delayMicroseconds(WRITE_INTERVAL);
    accel.setRate(ADXL345_RATE_50);
    delayMicroseconds(WRITE_INTERVAL);
    
    uint8_t buffer=0;
    I2Cdev::readByte(ADXL345_DEFAULT_ADDRESS, ADXL345_RA_POWER_CTL, &buffer);
    pb_printf("ADXL345_RA_POWER_CTL=%d", buffer);
    I2Cdev::readByte(ADXL345_DEFAULT_ADDRESS, ADXL345_RA_DATA_FORMAT, &buffer);
    pb_printf("ADXL345_RA_DATA_FORMAT=%d", buffer);
    I2Cdev::readByte(ADXL345_DEFAULT_ADDRESS, ADXL345_RA_BW_RATE, &buffer);
    pb_printf("ADXL345_RA_BW_RATE=%d", buffer);

    if (!gyro.testConnection()) {
        pb_printf("ITG3205 NOT FOUND!");
        return false;
    }

    gyro.initialize();
    gyro.reset();
    delayMicroseconds(WRITE_INTERVAL);
    gyro.setFullScaleRange(ITG3200_FULLSCALE_2000);
    delayMicroseconds(WRITE_INTERVAL);
    gyro.setDLPFBandwidth(ITG3200_DLPF_BW_42);
    delayMicroseconds(WRITE_INTERVAL);
    gyro.setRate(0x13);     //1khz/(1+0x13)  50hz
    delayMicroseconds(WRITE_INTERVAL);
    gyro.setClockSource(ITG3200_CLOCK_PLL_ZGYRO);
    delayMicroseconds(WRITE_INTERVAL);
    
    I2Cdev::readByte(ITG3200_DEFAULT_ADDRESS, ITG3200_RA_PWR_MGM, &buffer);
    pb_printf("ITG3200_RA_PWR_MGM=%d", buffer);
    I2Cdev::readByte(ITG3200_DEFAULT_ADDRESS, ITG3200_RA_DLPF_FS, &buffer);
    pb_printf("ITG3200_RA_DLPF_FS=%d", buffer);
    I2Cdev::readByte(ITG3200_DEFAULT_ADDRESS, ITG3200_RA_SMPLRT_DIV, &buffer);
    pb_printf("ITG3200_RA_SMPLRT_DIV=%d", buffer);

    delayMicroseconds(WRITE_INTERVAL);
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

    pb_printf("GY85 INIT SUCCESS!");

    return true;
}

void GY85::get_data(float imu_data[9]) {
    accel.getAcceleration(&ax, &ay, &az);
    gyro.getRotation(&gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);
    mag.setMode(HMC5883L_MODE_SINGLE);

#if IMU_DEBUG_ENABLE
    pb_printf("[%d %d %d] [%d %d %d] [%d %d %d]", ax, ay, az, gx, gy, gz, mx, my, mz);
#endif

    imu_data[0] = ax*ADXL345_SCALE;
    imu_data[1] = ay*ADXL345_SCALE;
    imu_data[2] = az*ADXL345_SCALE;
    imu_data[3] = gx*ITG3205_SCALE;
    imu_data[4] = gy*ITG3205_SCALE;
    imu_data[5] = gz*ITG3205_SCALE;
    imu_data[6] = mx*HMC5883L_SCALE;
    imu_data[7] = my*HMC5883L_SCALE;
    imu_data[8] = mz*HMC5883L_SCALE;
}
