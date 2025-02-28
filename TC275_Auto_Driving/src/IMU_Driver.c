/*******************************************************************************
 * @file    IMU_Driver.c
 * @brief   IMU_accel, gyro,mag,heading without filter
 * @version 1.0
 * @date    2025-02-12
 ******************************************************************************/

#include "IMU_Driver.h"
uint8 aaa=0;
static IfxI2c_I2c_Device g_i2cSet;

//static float asa_x = 0;
//static float asa_y = 0;
//static float asa_z = 0;
float asa_x;
float asa_y;
float asa_z;
IMU imu_offset={0,0,0,0,0,0,0,0,0,0};
static IfxI2c_I2c g_i2cMaster;
uint8 now13;
// ak status check reg
 uint8 status1_val;
 uint8 status2_val;
static float magx_max = -10000;
static float magy_max = -10000;
static float magz_max = -10000;
static float magx_min = 10000;
static float magy_min = 10000;
static float magz_min = 10000;
static float avg_rad = 0;

 float scale_x;
 float scale_y;
 float scale_z;
 float x_offset;
 float y_offset;
 float z_offset;

void initIMU ()
{
    uint8 readData = 0;
    uint8 trycnt = 0;
    forceI2CBusReset();
    do
    {
        delay(100000);
        delay(100000);
        initI2c();
        delay(1000000);

        initAK8963();
        uint8 whoAmI = WHOAMI_REG;
        i2cWrite(MPU9250_ADDRESS, &whoAmI, 1);
        delay(10000);
        i2cRead(MPU9250_ADDRESS, &readData, 1);
        trycnt++;
        now13=5;
    }while (readData != 0x71 && (trycnt < 5));
    setDLPF();
    delay(1000000);
    checkoffset();
}

void initI2c (void)
{

    IfxI2c_Pins i2cpinset = {&SCL_PIN, &SDA_PIN, IfxPort_PadDriver_cmosAutomotiveSpeed1};
    IfxI2c_I2c_Config i2cConfig;

    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);
    i2cConfig.pins = &i2cpinset;
    i2cConfig.baudrate = 100000;
    IfxI2c_I2c_initModule(&g_i2cMaster, &i2cConfig);
    g_i2cSet.i2c = &g_i2cMaster;
    g_i2cSet.deviceAddress = (MPU9250_ADDRESS << 1);

    uint8 initData[2] = {PWR_MGMT_1, 0x00};
    i2cWrite(MPU9250_ADDRESS, initData, 2);
    now13=6;
    delay(1000000);
    uint8 initAccelData[2] = {ACCEL_CONFIG_REG, 0x00}; //+-2g
    i2cWrite(MPU9250_ADDRESS, initAccelData, 2);
    uint8 initGyroData[2] = {GYRO_CONFIG_REG, 0x00};  //+-250
    i2cWrite(MPU9250_ADDRESS, initGyroData, 2);
}

IfxI2c_I2c_Status i2cWrite (uint8 slaveAddress, uint8 *data, Ifx_SizeT length)
{
    g_i2cSet.deviceAddress = slaveAddress << 1;
    IfxI2c_I2c_Status status = IfxI2c_I2c_write(&g_i2cSet, data, length);
    delay(100);
    return status;

}

void i2cRead (uint8 slaveAddress, uint8 *data, Ifx_SizeT length)
{
    g_i2cSet.deviceAddress = (slaveAddress << 1) | 1;
    IfxI2c_I2c_read(&g_i2cSet, data, length);
    delay(100);
}

IMU imuRead ()
{
    IMU now_imu={0,0,0,0,0,0,0,0,0,0};

    uint8 accelAddr = ACCEL_REG;
    uint8 gyroAddr = GYRO_REG;
    uint8 magAddr = MAG_REG;

    uint8 accelData[6] = {0};
    uint8 gyroData[6] = {0};
    uint8 magData[6] = {0};

    sint16 accel_x_raw, accel_y_raw, accel_z_raw;
    sint16 gyro_x_raw, gyro_y_raw, gyro_z_raw;
    sint16 mag_x_raw, mag_y_raw, mag_z_raw;

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float heading;


    //stop condition
    i2cStopCondition();
    delay(10);
    i2cStartCondition();


    i2cWrite(MPU9250_ADDRESS, &accelAddr, 1);
    i2cRead(MPU9250_ADDRESS, accelData, 6);

    //stop condition
//    i2cStopCondition();
//    delay(10);
//    i2cStartCondition();


    i2cWrite(MPU9250_ADDRESS, &gyroAddr, 1);
    i2cRead(MPU9250_ADDRESS, gyroData, 6);

    now13=7;
     //stop condition
//    i2cStopCondition();
//    delay(1000);
//    i2cStartCondition();


//    uint8 status_reg = AK_UPDATE_REG;
//    i2cWrite(AK8963_ADDRESS, &status_reg, 1);
//    delay(10);
//    i2cRead(AK8963_ADDRESS, &status1_val, 1);
//
//     //stop condition
//    i2cStopCondition();
//    delay(10);
//    i2cStartCondition();
//    now13=8;

//    i2cWrite(AK8963_ADDRESS, &magAddr, 1);  // AK8963 I2C
//    delay(10);
//    i2cRead(AK8963_ADDRESS, magData, 6);
//
////    //stop condition
////    i2cStopCondition();
////    delay(10);
////    i2cStartCondition();
//
//    now13=10;
//    // ak read
//    uint8 status2_reg = 0x09;
//    i2cWrite(AK8963_ADDRESS, &status2_reg, 1);
//    delay(10);
//    i2cRead(AK8963_ADDRESS, &status2_val, 1);
//    now13=11;

    //stop condition
    i2cStopCondition();
    delay(10);
    i2cStartCondition();

    accel_x_raw = (sint16) (accelData[0] << 8) | accelData[1];
    accel_y_raw = (sint16) (accelData[2] << 8) | accelData[3];
    accel_z_raw = (sint16) (accelData[4] << 8) | accelData[5];

    accel_x = ((float) accel_x_raw) / ACCEL_SEN - imu_offset.accel_x;
    accel_y = ((float) accel_y_raw) / ACCEL_SEN - imu_offset.accel_y;
    accel_z = ((float) accel_z_raw) / ACCEL_SEN - imu_offset.accel_z;

    gyro_x_raw = (sint16) (gyroData[0] << 8) | gyroData[1];
    gyro_y_raw = (sint16) (gyroData[2] << 8) | gyroData[3];
    gyro_z_raw = (sint16) (gyroData[4] << 8) | gyroData[5];

    gyro_x = ((float) gyro_x_raw) / GYRO_SEN - imu_offset.gyro_x;//* (M_PI / 180.0f)
    gyro_y = ((float) gyro_y_raw) / GYRO_SEN - imu_offset.gyro_y;
    gyro_z = ((float) gyro_z_raw) / GYRO_SEN - imu_offset.gyro_z;

    mag_x_raw = (sint16) (magData[1] << 8) | magData[0];
    mag_y_raw = (sint16) (magData[3] << 8) | magData[2];
    mag_z_raw = (sint16) (magData[5] << 8) | magData[4];

    mag_x = ((float) mag_x_raw)* asa_x * MAG_SEN;
    mag_y = ((float) mag_y_raw)* asa_y * MAG_SEN;
    mag_z = ((float) mag_z_raw)* asa_z * MAG_SEN;
    if(mag_x>magx_max)
        magx_max=mag_x;
    if(mag_x<magx_min)
        magx_min=mag_x;
    if(mag_y>magy_max)
        magy_max=mag_y;
    if(mag_y<magy_min)
        magy_min=mag_y;
    if(mag_z>magz_max)
        magz_max=mag_z;
    if(mag_z<magz_min)
        magz_min=mag_z;
    x_offset = (magx_max + magx_min) / 2;
    y_offset = (magy_max + magy_min) / 2;
    z_offset = (magz_max + magz_min) / 2;
    float scale_x_diff = magx_max - magx_min;
    float scale_y_diff = magy_max - magy_min;
    float scale_z_diff = magz_max - magz_min;
    avg_rad = (scale_x_diff + scale_y_diff + scale_z_diff)/ 3.0f;
    scale_x = avg_rad / scale_x_diff;
    scale_y = avg_rad / scale_y_diff;
    scale_z = avg_rad / scale_z_diff;

    //heading ->
//    heading = atan2(mag_x, mag_y) *(180 / M_PI);
//    if (heading < 0)
//        heading += 360;
    if(accel_x<0.02 && accel_x>-0.02) accel_x=0;
    if(accel_y<0.02 && accel_y>-0.02) accel_y=0;
    if(accel_z<1.02 && accel_z>0.98) accel_z=1;
    if(gyro_x<0.02 && gyro_x>-0.02) gyro_x=0;
    if(gyro_y<0.02 && gyro_y>-0.02) gyro_y=0;
    if(gyro_z<0.02 && gyro_z>-0.02) gyro_z=0;

    gyro_x = gyro_x* (M_PI / 180.0f);
    gyro_y = gyro_y* (M_PI / 180.0f);
    gyro_z = gyro_z* (M_PI / 180.0f);
    if(gyro_x<0.02 && gyro_x>-0.02) gyro_x=0;
    if(gyro_y<0.02 && gyro_y>-0.02) gyro_y=0;
    if(gyro_z<0.02 && gyro_z>-0.02) gyro_z=0;


    now_imu.accel_x = accel_x;
    now_imu.accel_y = accel_y;
    now_imu.accel_z = accel_z;

    now_imu.gyro_x = gyro_x;//* (M_PI / 180.0f);
    now_imu.gyro_y = gyro_y;//* (M_PI / 180.0f);
    now_imu.gyro_z = gyro_z;//* (M_PI / 180.0f);

    now_imu.mag_x = (mag_x - x_offset)*scale_x;
    now_imu.mag_y = (mag_y - y_offset)*scale_y;
    now_imu.mag_z = (mag_z - z_offset)*scale_z;
    ///
    now_imu.accel_x = -now_imu.accel_x;
    //now_imu.accel_y = now_imu.accel_y;
    //now_imu.accel_z = now_imu.accel_z;

    //now_imu.gyro_x = now_imu.gyro_x;
    now_imu.gyro_y = -now_imu.gyro_y;
    now_imu.gyro_z = -now_imu.gyro_z;

    float temp = now_imu.mag_x;
    now_imu.mag_x = now_imu.mag_y;
    now_imu.mag_y = -temp;
    //now_imu.mag_z = now_imu.mag_z;


    //heading ->
//    heading = atan2(now_imu.mag_x, now_imu.mag_y) * (180 / M_PI);
//    if (heading < 0)
//        heading += 360;

//    now_imu.accel_x = magx_max;
//    now_imu.accel_y = magx_min;
//    now_imu.accel_z = 0;//accel_z;
//
//    now_imu.gyro_x = magy_max;
//    now_imu.gyro_y = magy_min;
//    now_imu.gyro_z = 0;//gyro_z;
//
//    now_imu.mag_x = magz_max;
//    now_imu.mag_y = magz_min;
//    now_imu.mag_z = 0;// mag_z;


   // now_imu.heading = heading;
    return now_imu;
}

IMU initimuRead ()
{
    now13=9;
    IMU now_imu={0,0,0,0,0,0,0,0,0,0};
    uint8 accelAddr = ACCEL_REG;
    uint8 gyroAddr = GYRO_REG;

    uint8 accelData[6] = {0};
    uint8 gyroData[6] = {0};

    sint16 accel_x_raw, accel_y_raw, accel_z_raw;
    sint16 gyro_x_raw, gyro_y_raw, gyro_z_raw;

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;

    i2cWrite(MPU9250_ADDRESS, &accelAddr, 1);
    i2cRead(MPU9250_ADDRESS, accelData, 6);

    i2cWrite(MPU9250_ADDRESS, &gyroAddr, 1);
    i2cRead(MPU9250_ADDRESS, gyroData, 6);
    accel_x_raw = (sint16) (accelData[0] << 8) | accelData[1];
    accel_y_raw = (sint16) (accelData[2] << 8) | accelData[3];
    accel_z_raw = (sint16) (accelData[4] << 8) | accelData[5];

    accel_x = ((float) accel_x_raw) / ACCEL_SEN;
    accel_y = ((float) accel_y_raw) / ACCEL_SEN;
    accel_z = ((float) accel_z_raw) / ACCEL_SEN;
    //0

    gyro_x_raw = (sint16) (gyroData[0] << 8) | gyroData[1];
    gyro_y_raw = (sint16) (gyroData[2] << 8) | gyroData[3];
    gyro_z_raw = (sint16) (gyroData[4] << 8) | gyroData[5];

    gyro_x = ((float) gyro_x_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    gyro_y = ((float) gyro_y_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    gyro_z = ((float) gyro_z_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    now_imu.accel_x = accel_x;
    now_imu.accel_y = accel_y;
    now_imu.accel_z = accel_z;

    now_imu.gyro_x = gyro_x;
    now_imu.gyro_y = gyro_y;
    now_imu.gyro_z = gyro_z;
    return now_imu;
}


/*
 initAK8963
 AK8963
 */

void initAK8963 (void)
{
    uint8 mag_mode_value = 0;
    uint8 mag_mode_valueoff = 1;
    uint8 status_valfir = 0;

    uint8 data[3];

    // I2C master
    uint8 bypass_reg1[2] = {0x6A, 0x00};
    i2cWrite(MPU9250_ADDRESS, bypass_reg1, 2);
    delay(10000);
    // MPU9250 I2C bypass enable
    uint8 bypass_reg2[2] = {0x37, 0x02};
    i2cWrite(MPU9250_ADDRESS, bypass_reg2, 2);
    delay(10000);

    // AK8963
    // Power down ->  ROM
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_power_down[2] = {AK_CNTL1_REG, 0x00};     // CNTL1
    i2cWrite(AK8963_ADDRESS, mag_power_down, 2);
    delay(10000);

    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_mode_check = AK_CNTL1_REG;                   //offmode
    i2cWrite(AK8963_ADDRESS, &mag_mode_check, 1);
    i2cRead(AK8963_ADDRESS, &mag_mode_valueoff, 1);

    // Fuse ROM access mode-> ROM
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_rom_access[2] = {AK_CNTL1_REG, 0x0F};
    i2cWrite(AK8963_ADDRESS, mag_rom_access, 2);
    delay(10000);

    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 asa_reg = 0x10;
    i2cWrite(AK8963_ADDRESS, &asa_reg, 1);
    delay(100);
    i2cRead(AK8963_ADDRESS, data, 3);

    asa_x = ((float) (data[0] - 128))/ 256.0f + 1.0f;
    asa_y = ((float) (data[1] - 128)) / 256.0f + 1.0f;
    asa_z = ((float) (data[2] - 128)) / 256.0f + 1.0f;

    // Power down -> ROM
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    i2cWrite(AK8963_ADDRESS, mag_power_down, 2);
    delay(10000);

    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    uint8 mag_continuous[2] = {AK_CNTL1_REG, 0x06};  //

    i2cWrite(AK8963_ADDRESS, mag_continuous, 2);
    //mag_mode_check = AK_CNTL1_REG;
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    i2cWrite(AK8963_ADDRESS, &mag_mode_check, 1);
    i2cRead(AK8963_ADDRESS, &mag_mode_value, 1);

    delay(10000);

    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    //drdy
    uint8 status_reg = 0x02;
    i2cWrite(AK8963_ADDRESS, &status_reg, 1);
    delay(10);
    i2cRead(AK8963_ADDRESS, &status_valfir, 1);

}

/*
 forceI2CBusReset
 Bus
 */
void forceI2CBusReset (void)
{
    // I2C
    IfxI2c_disableModule(&MODULE_I2C0);

    // SDA
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    IfxPort_setPinHigh(&MODULE_P13, 1);
    IfxPort_setPinHigh(&MODULE_P13, 2);
    delay(10000);

    //
    for (int i = 0; i < 9; i++)
    {
        IfxPort_setPinLow(&MODULE_P13, 1);
        delay(1000);
        IfxPort_setPinHigh(&MODULE_P13, 2);
        delay(1000);
    }

    // STOP condition -> scl
    IfxPort_setPinLow(&MODULE_P13, 1);   //sda
    delay(1000);
    IfxPort_setPinHigh(&MODULE_P13, 2);  //scl
    delay(1000);
    IfxPort_setPinHigh(&MODULE_P13, 1);  //sda
    delay(10000);

    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 i2cStopCondition
 i2c stop condition
 */
void i2cStopCondition (void)
{
    // I2C
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // STOP condition -> scl
    IfxPort_setPinLow(&MODULE_P13, 1);  // SDA LOW
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 2); // SCL HIGH
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 1); // SDA HIGH (STOP)
    delay(10);

    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 i2cStartCondition
 i2c start condition
 */
void i2cStartCondition (void)
{
    //  I2C
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // Start Condition
    // SCL
    IfxPort_setPinHigh(&MODULE_P13, 2); // SCL HIGH
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 1); // SDA HIGH
    delay(10);
    IfxPort_setPinLow(&MODULE_P13, 1);  // SDA LOW
    delay(10);

    // SCL
    IfxPort_setPinLow(&MODULE_P13, 2);  // SCL LOW
    delay(10);
    // I2C
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 setDLPF
 low pass filter
 */
void setDLPF (void)
{
    uint8 gyro_dlpf[2] = {0x1A, 0x03};
    i2cWrite(MPU9250_ADDRESS, gyro_dlpf, 2);
    delay(10);

    uint8 accel_dlpf[2] = {0x1D, 0x03};
    i2cWrite(MPU9250_ADDRESS, accel_dlpf, 2);
    delay(10);
}

/*
 checkoffset
 */
void checkoffset(void)
{
    IMU now_status={0,0,0,0,0,0,0,0,0,0};
    IMU now_statussum={0,0,0,0,0,0,0,0,0,0};
    uint8 sampling = 100;
    for(int i=0;i<sampling;i++)
    {
        now_status=initimuRead();

        now_statussum.accel_x += now_status.accel_x;
        now_statussum.accel_y += now_status.accel_y;
        now_statussum.accel_z += now_status.accel_z;

        now_statussum.gyro_x += now_status.gyro_x;
        now_statussum.gyro_y += now_status.gyro_y;
        now_statussum.gyro_z += now_status.gyro_z;
    }

    imu_offset.accel_x = now_statussum.accel_x/100;
    imu_offset.accel_y = now_statussum.accel_y/100;
    imu_offset.accel_z = now_statussum.accel_z/100-1;

    imu_offset.gyro_x = now_statussum.gyro_x/100;
    imu_offset.gyro_y = now_statussum.gyro_y/100;
    imu_offset.gyro_z = now_statussum.gyro_z/100;
}
