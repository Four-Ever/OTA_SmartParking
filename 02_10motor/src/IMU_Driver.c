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

/*
 initIMU 占쎈맙占쎈땾
 MPU9250 IMU(揶쏉옙占쎈꺗占쎈즲, 揶쏄낯�꺗占쎈즲) + AK8963(筌욑옙占쎌쁽疫뀐옙) 占쎄쉭占쎈샒
 占쎈꽰占쎈뻿占쎌넇占쎌뵥占쎈릭�⑨옙 5甕곕뜃�돱筌욑옙 占쎈뻻占쎈즲
 */
void initIMU ()
{
    uint8 readData = 0;
    uint8 trycnt = 0;
    forceI2CBusReset();   // 甕곌쑴�뮞 揶쏅벡�젫 �뵳�딅��
    do
    {
        delay(100000);
        delay(100000);
        initI2c();// I2C �룯�뜃由곤옙�넅
        delay(1000000);

        initAK8963();// AK8963(筌욑옙占쎌쁽疫뀐옙)�룯�뜃由곤옙�넅
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

/*
 initI2c 占쎈맙占쎈땾
 MPU9250 IMU(揶쏉옙占쎈꺗占쎈즲, 揶쏄낯�꺗占쎈즲)�몴占� 占쎌맄占쎈립 I2C 占쎄쉭占쎈샒
 揶쏉옙占쎈꺗占쎈즲, 揶쏄낯�꺗占쎈즲 占쎈짗占쎌삂筌뤴뫀諭� 占쎄퐬占쎌젟
 */
void initI2c (void)
{

    IfxI2c_Pins i2cpinset = {&SCL_PIN, &SDA_PIN, IfxPort_PadDriver_cmosAutomotiveSpeed1};
    IfxI2c_I2c_Config i2cConfig;


    //占쎄텢占쎌뒠占쎈막 i2c占쎄퐬占쎌젟
    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);
    i2cConfig.pins = &i2cpinset;
    i2cConfig.baudrate = 100000;   //占쎈꽰占쎈뻿 占쎈꺗占쎈즲 100000;
    IfxI2c_I2c_initModule(&g_i2cMaster, &i2cConfig);
    g_i2cSet.i2c = &g_i2cMaster;
    g_i2cSet.deviceAddress = (MPU9250_ADDRESS << 1);

    uint8 initData[2] = {PWR_MGMT_1, 0x00}; //MPU占쎌읈占쎌뜚
    i2cWrite(MPU9250_ADDRESS, initData, 2); //Sleep占쎈퓠占쎄퐣 繹먥뫁占�
    now13=6;
    delay(1000000);
    //揶쏉옙占쎈꺗占쎈즲, 揶쏄낯�꺗占쎈즲 占쎈짗占쎌삂筌뤴뫀諭�
    uint8 initAccelData[2] = {ACCEL_CONFIG_REG, 0x00}; //+-2g
    i2cWrite(MPU9250_ADDRESS, initAccelData, 2);
    uint8 initGyroData[2] = {GYRO_CONFIG_REG, 0x00};  //+-250
    i2cWrite(MPU9250_ADDRESS, initGyroData, 2);
}

/*
 i2cWrite 占쎈맙占쎈땾
 g_i2cSet i2c�몴占� 占쎈꽰占쎈퉸
 slave占쎈퓠 write
 */
IfxI2c_I2c_Status i2cWrite (uint8 slaveAddress, uint8 *data, Ifx_SizeT length)
{
    g_i2cSet.deviceAddress = slaveAddress << 1;
    IfxI2c_I2c_Status status = IfxI2c_I2c_write(&g_i2cSet, data, length);
    delay(100);
    return status;

}

/*
 i2cRead 占쎈맙占쎈땾
 g_i2cSet i2c�몴占� 占쎈꽰占쎈퉸
 length筌띾슦寃� 占쎌뵭占쎈선 data占쎈퓠 占쏙옙占쎌삢
 */
void i2cRead (uint8 slaveAddress, uint8 *data, Ifx_SizeT length)
{
    g_i2cSet.deviceAddress = (slaveAddress << 1) | 1;
    IfxI2c_I2c_read(&g_i2cSet, data, length);
    delay(100);
}

/*
 imuRead 占쎈맙占쎈땾
 imu揶쏉옙 占쎌뵭占쎈선占쏙옙占쎄퐣 return占쎈릭占쎈뮉 占쎈맙占쎈땾
 */
IMU imuRead ()
{
    IMU now_imu={0,0,0,0,0,0,0,0,0,0};
    // 揶쏉옙占쎈꺗占쎈즲, 揶쏄낯�꺗占쎈즲, 筌욑옙占쎌쁽疫꿸퀗而� 占쎌젔域뱄옙 占쎌쟿筌욑옙占쎈뮞占쎄숲
    uint8 accelAddr = ACCEL_REG;
    uint8 gyroAddr = GYRO_REG;
    uint8 magAddr = MAG_REG;

    // 占쎄쉽占쎄퐣 占쎈쑓占쎌뵠占쎄숲 甕곌쑵�쓠
    uint8 accelData[6] = {0};      // 揶쏉옙占쎈꺗占쎈즲 占쎈쑓占쎌뵠占쎄숲 甕곌쑵�쓠
    uint8 gyroData[6] = {0};       // 占쎌쁽占쎌뵠嚥∽옙 占쎈쑓占쎌뵠占쎄숲 甕곌쑵�쓠
    uint8 magData[6] = {0};        // 筌욑옙占쎌쁽疫뀐옙 占쎈쑓占쎌뵠占쎄숲 甕곌쑵�쓠

    // 筌β돦�젟 raw 揶쏉옙
    sint16 accel_x_raw, accel_y_raw, accel_z_raw;    // 揶쏉옙占쎈꺗占쎈즲 揶쏉옙 (16�뜮袁る뱜)
    sint16 gyro_x_raw, gyro_y_raw, gyro_z_raw;       // 揶쏄낯�꺗占쎈즲 揶쏉옙 (16�뜮袁る뱜)
    sint16 mag_x_raw, mag_y_raw, mag_z_raw;          // 筌욑옙占쎌쁽疫뀐옙 揶쏉옙 (16�뜮袁る뱜)

    // 野껉퀗�궢 揶쏉옙
    float accel_x, accel_y, accel_z;    // 揶쏉옙占쎈꺗占쎈즲 揶쏉옙 (16�뜮袁る뱜)
    float gyro_x, gyro_y, gyro_z;       // 揶쏄낯�꺗占쎈즲 揶쏉옙 (16�뜮袁る뱜)
    float mag_x, mag_y, mag_z;          // 筌욑옙占쎌쁽疫뀐옙 揶쏉옙 (16�뜮袁る뱜)
    float heading;


    //stop condition
    i2cStopCondition();
    delay(10);
    i2cStartCondition();

    // 揶쏉옙占쎈꺗占쎈즲 占쎈쑓占쎌뵠占쎄숲
    i2cWrite(MPU9250_ADDRESS, &accelAddr, 1);
    i2cRead(MPU9250_ADDRESS, accelData, 6);

    //stop condition
//    i2cStopCondition();
//    delay(10);
//    i2cStartCondition();


    // 占쎌쁽占쎌뵠嚥∽옙 占쎈쑓占쎌뵠占쎄숲
    i2cWrite(MPU9250_ADDRESS, &gyroAddr, 1);
    i2cRead(MPU9250_ADDRESS, gyroData, 6);

    now13=7;
    // 筌욑옙占쎌쁽疫뀐옙 setting

     //stop condition
//    i2cStopCondition();
//    delay(1000);
//    i2cStartCondition();

     //筌욑옙占쎌쁽疫뀐옙 揶쏉옙 update占쎈┗占쎈뮉筌욑옙 check
//    uint8 status_reg = AK_UPDATE_REG;       //drdy占쎌넇占쎌뵥 reg
//    i2cWrite(AK8963_ADDRESS, &status_reg, 1);
//    delay(10);
//    i2cRead(AK8963_ADDRESS, &status1_val, 1);
//
//     //stop condition
//    i2cStopCondition();
//    delay(10);
//    i2cStartCondition();
//    now13=8;
//    // 筌욑옙占쎌쁽疫뀐옙 占쎈쑓占쎌뵠占쎄숲
//    i2cWrite(AK8963_ADDRESS, &magAddr, 1);  // AK8963 I2C 雅뚯눘�꺖
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

    /// 占쎈뻻占쎌삂 -> 占쎈뼊占쎌맄 筌ｋ똾寃� //////////////////////////////////////////////////////////////////////////////////////
    // 揶쏉옙占쎈꺗占쎈즲 占쎈쑓占쎌뵠占쎄숲 癰귨옙占쎌넎
    accel_x_raw = (sint16) (accelData[0] << 8) | accelData[1];
    accel_y_raw = (sint16) (accelData[2] << 8) | accelData[3];
    accel_z_raw = (sint16) (accelData[4] << 8) | accelData[5];

    //揶쏅Ŧ猷� 鈺곌퀣�젟 2g
    accel_x = ((float) accel_x_raw) / ACCEL_SEN - imu_offset.accel_x;
    accel_y = ((float) accel_y_raw) / ACCEL_SEN - imu_offset.accel_y;
    accel_z = ((float) accel_z_raw) / ACCEL_SEN - imu_offset.accel_z;

    // 占쎌쁽占쎌뵠嚥∽옙 占쎈쑓占쎌뵠占쎄숲 癰귨옙占쎌넎
    gyro_x_raw = (sint16) (gyroData[0] << 8) | gyroData[1];
    gyro_y_raw = (sint16) (gyroData[2] << 8) | gyroData[3];
    gyro_z_raw = (sint16) (gyroData[4] << 8) | gyroData[5];

    //揶쏅Ŧ猷� 鈺곌퀣�젟 250
    gyro_x = ((float) gyro_x_raw) / GYRO_SEN - imu_offset.gyro_x;//* (M_PI / 180.0f)
    gyro_y = ((float) gyro_y_raw) / GYRO_SEN - imu_offset.gyro_y;
    gyro_z = ((float) gyro_z_raw) / GYRO_SEN - imu_offset.gyro_z;

    // 筌욑옙占쎌쁽疫뀐옙 占쎈쑓占쎌뵠占쎄숲 癰귨옙占쎌넎
    mag_x_raw = (sint16) (magData[1] << 8) | magData[0];
    mag_y_raw = (sint16) (magData[3] << 8) | magData[2];
    mag_z_raw = (sint16) (magData[5] << 8) | magData[4];

    //揶쏅Ŧ猷� 鈺곌퀣�젟
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
    /////.h占쎈솁占쎌뵬 784
    x_offset = (magx_max + magx_min) / 2;//占쎈툡占쎈あ占쎌뵠占쎈걗占쎌삂 占쎈뼄�몴�떯苡� 占쎄텆 占쎌맄占쎈퓠占쎄퐣 占쎌뵠沃섓옙 �뤃�뗫맙
    y_offset = (magy_max + magy_min) / 2;
    z_offset = (magz_max + magz_min) / 2;
    float scale_x_diff = magx_max - magx_min;
    float scale_y_diff = magy_max - magy_min;
    float scale_z_diff = magz_max - magz_min;
    avg_rad = (scale_x_diff + scale_y_diff + scale_z_diff)/ 3.0f;
    scale_x = avg_rad / scale_x_diff;
    scale_y = avg_rad / scale_y_diff;
    scale_z = avg_rad / scale_z_diff;

    //heading -> �겫怨멥걹 0, 占쎌궎�몴紐꾠걹占쎌몵嚥∽옙 +
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


    //heading -> �겫怨멥걹 0, 占쎌궎�몴紐꾠걹占쎌몵嚥∽옙 +
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


    now_imu.heading = heading;
    return now_imu;
}

IMU initimuRead ()
{
    now13=9;
    IMU now_imu={0,0,0,0,0,0,0,0,0,0};
    // 揶쏉옙占쎈꺗占쎈즲, 揶쏄낯�꺗占쎈즲, 筌욑옙占쎌쁽疫꿸퀗而� 占쎌젔域뱄옙 占쎌쟿筌욑옙占쎈뮞占쎄숲
    uint8 accelAddr = ACCEL_REG;
    uint8 gyroAddr = GYRO_REG;

    // 占쎄쉽占쎄퐣 占쎈쑓占쎌뵠占쎄숲 甕곌쑵�쓠
    uint8 accelData[6] = {0};      // 揶쏉옙占쎈꺗占쎈즲 占쎈쑓占쎌뵠占쎄숲 甕곌쑵�쓠
    uint8 gyroData[6] = {0};       // 占쎌쁽占쎌뵠嚥∽옙 占쎈쑓占쎌뵠占쎄숲 甕곌쑵�쓠

    // 筌β돦�젟 raw 揶쏉옙
    sint16 accel_x_raw, accel_y_raw, accel_z_raw;    // 揶쏉옙占쎈꺗占쎈즲 揶쏉옙 (16�뜮袁る뱜)
    sint16 gyro_x_raw, gyro_y_raw, gyro_z_raw;       // 揶쏄낯�꺗占쎈즲 揶쏉옙 (16�뜮袁る뱜)

    // 野껉퀗�궢 揶쏉옙
    float accel_x, accel_y, accel_z;    // 揶쏉옙占쎈꺗占쎈즲 揶쏉옙 (16�뜮袁る뱜)
    float gyro_x, gyro_y, gyro_z;       // 揶쏄낯�꺗占쎈즲 揶쏉옙 (16�뜮袁る뱜)


    // 揶쏉옙占쎈꺗占쎈즲 占쎈쑓占쎌뵠占쎄숲
    i2cWrite(MPU9250_ADDRESS, &accelAddr, 1);
    i2cRead(MPU9250_ADDRESS, accelData, 6);

    // 占쎌쁽占쎌뵠嚥∽옙 占쎈쑓占쎌뵠占쎄숲
    i2cWrite(MPU9250_ADDRESS, &gyroAddr, 1);
    i2cRead(MPU9250_ADDRESS, gyroData, 6);
    /// 占쎈뻻占쎌삂 -> 占쎈뼊占쎌맄 筌ｋ똾寃� //////////////////////////////////////////////////////////////////////////////////////
    // 揶쏉옙占쎈꺗占쎈즲 占쎈쑓占쎌뵠占쎄숲 癰귨옙占쎌넎
    accel_x_raw = (sint16) (accelData[0] << 8) | accelData[1];
    accel_y_raw = (sint16) (accelData[2] << 8) | accelData[3];
    accel_z_raw = (sint16) (accelData[4] << 8) | accelData[5];

    //揶쏅Ŧ猷� 鈺곌퀣�젟 2g
    accel_x = ((float) accel_x_raw) / ACCEL_SEN;
    accel_y = ((float) accel_y_raw) / ACCEL_SEN;
    accel_z = ((float) accel_z_raw) / ACCEL_SEN;
    //0

    // 占쎌쁽占쎌뵠嚥∽옙 占쎈쑓占쎌뵠占쎄숲 癰귨옙占쎌넎
    gyro_x_raw = (sint16) (gyroData[0] << 8) | gyroData[1];
    gyro_y_raw = (sint16) (gyroData[2] << 8) | gyroData[3];
    gyro_z_raw = (sint16) (gyroData[4] << 8) | gyroData[5];

    //揶쏅Ŧ猷� 鈺곌퀣�젟 250
    gyro_x = ((float) gyro_x_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    gyro_y = ((float) gyro_y_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    gyro_z = ((float) gyro_z_raw) / GYRO_SEN;//* (M_PI / 180.0f);
///0 -> 占쎈뼊占쎌맄 鈺곌퀣�젟占쏙옙 占쎄돌餓λ쵐肉됵옙�뵥揶쏉옙
    now_imu.accel_x = accel_x;
    now_imu.accel_y = accel_y;
    now_imu.accel_z = accel_z;

    now_imu.gyro_x = gyro_x;
    now_imu.gyro_y = gyro_y;
    now_imu.gyro_z = gyro_z;
////init占쎈맙占쎈땾占쎈뮉 �눧紐꾩젫 占쎈씨占쎈뮉占쎈쾹!!!!
    return now_imu;
}


/*
 initAK8963 占쎈맙占쎈땾
 AK8963(筌욑옙占쎌쁽疫뀐옙) 占쎄쉭占쎈샒
 */

void initAK8963 (void)
{
    uint8 mag_mode_value = 0;
    uint8 mag_mode_valueoff = 1;
    uint8 status_valfir = 0;

    uint8 data[3];

    // I2C master �뜮袁れ넞占쎄쉐占쎌넅 -> MPU揶쏉옙 筌띾뜆�뮞占쎄숲揶쏉옙 占쎈┷筌욑옙 占쎈륫野껓옙 占쎄퐬占쎌젟-> 占쎄땀揶쏉옙 占쎌쁽筌ｋ똻�읅占쎌몵嚥∽옙 占쎈꽰占쎈뻿占쎈막椰꾧퀡�뵬
    uint8 bypass_reg1[2] = {0x6A, 0x00};
    i2cWrite(MPU9250_ADDRESS, bypass_reg1, 2);
    delay(10000);
    // MPU9250 I2C bypass enable
    uint8 bypass_reg2[2] = {0x37, 0x02};
    i2cWrite(MPU9250_ADDRESS, bypass_reg2, 2);
    delay(10000);

    // AK8963 �룯�뜃由곤옙�넅
    // Power down ->  ROM占쎈퓠 占쎌젔域뱀눛釉�占쎌젻�⑨옙 �④쑴�꺗 占쎌삛繹먲옙 占쎄콢疫뀐옙
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_power_down[2] = {AK_CNTL1_REG, 0x00};     // CNTL1 占쎌쟿筌욑옙占쎈뮞占쎄숲
    i2cWrite(AK8963_ADDRESS, mag_power_down, 2);
    delay(10000);

    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_mode_check = AK_CNTL1_REG;                   //offmode 占쎌삋 占쎈굶占쎈선揶쏅뗀�뮉筌욑옙
    i2cWrite(AK8963_ADDRESS, &mag_mode_check, 1);
    i2cRead(AK8963_ADDRESS, &mag_mode_valueoff, 1);

    // Fuse ROM access mode-> ROM占쎈퓠 占쎌젔域뱀눛鍮먲옙苑� �룯�뜃由� 揶쏅Ŧ猷� 占쎈르占쎈툡占쎄땀占쎌젻�⑨옙
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_rom_access[2] = {AK_CNTL1_REG, 0x0F};
    i2cWrite(AK8963_ADDRESS, mag_rom_access, 2);
    delay(10000);

    // �룯�뜃由� 揶쏅Ŧ猷� 占쎌뵭疫뀐옙
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 asa_reg = 0x10;  //揶쏅Ŧ猷� 占쏙옙占쎌삢占쎈쭆椰꾬옙 占쎈뻻占쎌삂 雅뚯눘�꺖
    i2cWrite(AK8963_ADDRESS, &asa_reg, 1);
    delay(100);
    i2cRead(AK8963_ADDRESS, data, 3);

    // 揶쏅Ŧ猷� 鈺곌퀣�젟揶쏉옙 占쏙옙占쎌삢
    asa_x = ((float) (data[0] - 128))/ 256.0f + 1.0f;
    asa_y = ((float) (data[1] - 128)) / 256.0f + 1.0f;
    asa_z = ((float) (data[2] - 128)) / 256.0f + 1.0f;

    // Power down -> ROM占쎌젔域뱀눖�걟占쎈굡占쎈퓠占쎄퐣 占쎄돌占쏙옙占쎄퐣!!筌β돦�젟筌뤴뫀諭� 占쎌읈占쎈퓠 占쎈툧占쎌젟占쎌넅占쎈릭占쎌젻�⑨옙
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    i2cWrite(AK8963_ADDRESS, mag_power_down, 2);
    delay(10000);

    // 占쎈염占쎈꺗筌β돦�젟筌뤴뫀諭� 占쎈뻻占쎌삂
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    uint8 mag_continuous[2] = {AK_CNTL1_REG, 0x06};  //

    ///1占쏙옙 占쎈립甕곤옙 占쎈릭�⑨옙 off
    i2cWrite(AK8963_ADDRESS, mag_continuous, 2);
    //mag_mode_check = AK_CNTL1_REG;
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    i2cWrite(AK8963_ADDRESS, &mag_mode_check, 1);
    i2cRead(AK8963_ADDRESS, &mag_mode_value, 1);

    delay(10000);

    //占쎈씜占쎈쑓占쎌뵠占쎈뱜 占쎈┗占쎈뮉筌욑옙 占쎌넇占쎌뵥
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    //drdy占쎌넇占쎌뵥 reg
    uint8 status_reg = 0x02;
    i2cWrite(AK8963_ADDRESS, &status_reg, 1);
    delay(10);
    i2cRead(AK8963_ADDRESS, &status_valfir, 1);

}

/*
 forceI2CBusReset 占쎈맙占쎈땾
 Bus �룯�뜃由곤옙�넅
 */
void forceI2CBusReset (void)
{
    // I2C 筌뤴뫀諭� �뜮袁れ넞占쎄쉐占쎌넅
    IfxI2c_disableModule(&MODULE_I2C0);

    // SDA占쏙옙 SCL占쎌뱽 GPIO
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // 占쎈ぎ 占쎈뼄 high嚥∽옙 占쎄퐬占쎌젟
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

    // STOP condition -> scl占쎌뵠 high占쎌뵬 占쎈르 sda low占쎈퓠占쎄퐣 high
    IfxPort_setPinLow(&MODULE_P13, 1);   //sda
    delay(1000);
    IfxPort_setPinHigh(&MODULE_P13, 2);  //scl
    delay(1000);
    IfxPort_setPinHigh(&MODULE_P13, 1);  //sda
    delay(10000);

    // 占쎈뼄占쎈뻻 I2C 占쏙옙占쎌몵嚥∽옙 占쎄퐬占쎌젟
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 i2cStopCondition 占쎈맙占쎈땾
 i2c stop condition 占쎄문占쎄쉐
 */
void i2cStopCondition (void)
{
    // I2C 占쏙옙占쎌뱽 GPIO 筌뤴뫀諭뜻에占�
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // STOP condition -> scl占쎌뵠 high占쎌뵬 占쎈르 sda low占쎈퓠占쎄퐣 high
    IfxPort_setPinLow(&MODULE_P13, 1);  // SDA LOW
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 2); // SCL HIGH
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 1); // SDA HIGH (STOP)
    delay(10);

    // 占쎈뼄占쎈뻻 I2C 筌뤴뫀諭�
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 i2cStartCondition 占쎈맙占쎈땾
 i2c start condition 占쎄문占쎄쉐
 */
void i2cStartCondition (void)
{
    //  I2C 占쏙옙占쎌뱽 GPIO 筌뤴뫀諭뜻에占�
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // Start Condition
    // SCL占쎌뱽 HIGH嚥∽옙 占쎄퐬占쎌젟
    IfxPort_setPinHigh(&MODULE_P13, 2); // SCL HIGH
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 1); // SDA HIGH
    delay(10);
    IfxPort_setPinLow(&MODULE_P13, 1);  // SDA LOW
    delay(10);

    // SCL占쎌뱽 LOW嚥∽옙 占쎄퐬占쎌젟 (占쎄깻占쎌쑏 占쎈뻻占쎌삂)
    IfxPort_setPinLow(&MODULE_P13, 2);  // SCL LOW
    delay(10);
    // I2C 筌뤴뫀諭뜻에占� 癰귣벀�럡
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 setDLPF 占쎈맙占쎈땾
 low pass filter 占쎄퐬占쎌젟
 -> 占쎈땾占쎌젟占쎈퉸占쎈튊占쎈맙
 */
void setDLPF (void)
{
    uint8 gyro_dlpf[2] = {0x1A, 0x03};  // 占쎌쁽占쎌뵠嚥∽옙 DLPF 41Hz 占쎄퐬占쎌젟
    i2cWrite(MPU9250_ADDRESS, gyro_dlpf, 2);
    delay(10);

    uint8 accel_dlpf[2] = {0x1D, 0x03};  // 揶쏉옙占쎈꺗占쎈즲 DLPF 21.2Hz 占쎄퐬占쎌젟
    i2cWrite(MPU9250_ADDRESS, accel_dlpf, 2);
    delay(10);
}

/*
 checkoffset 占쎈맙占쎈땾
 �룯�뜃由� 占쎌궎�몴占� 癰귣똻�젟
 -> 占쎈땾占쎌젟占쎈퉸占쎈튊占쎈맙
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
    imu_offset.accel_z = now_statussum.accel_z/100-1;//餓λ쵎�젾 占쎌겫占쎈샨 占쎌젫椰꾬옙

    imu_offset.gyro_x = now_statussum.gyro_x/100;
    imu_offset.gyro_y = now_statussum.gyro_y/100;
    imu_offset.gyro_z = now_statussum.gyro_z/100;
}
