/*******************************************************************************
 * @file    IMU_Driver.c
 * @brief   IMU_accel, gyro,mag,heading without filter
 * @version 1.0
 * @date    2025-02-12
 ******************************************************************************/

#include "IMU_Driver.h"
static IfxI2c_I2c_Device g_i2cSet;

//static float asa_x = 0;
//static float asa_y = 0;
//static float asa_z = 0;
float asa_x;
float asa_y;
float asa_z;
static IMU imu_offset={0,0,0,0,0,0,0,0,0,0};
static IfxI2c_I2c g_i2cMaster;
// ak status check reg
static float magx_max = -10000;
static float magy_max = -10000;
static float magz_max = -10000;
static float magx_min = 10000;
static float magy_min = 10000;
static float magz_min = 10000;
static float avg_rad = 0;
uint8 status1_val;
uint8 status2_val;
float scale_x;
float scale_y;
float scale_z;
float x_offset;
float y_offset;
float z_offset;
IfxPort_State TouchState = 0;
/*
 initIMU �븿�닔
 MPU9250 IMU(媛��냽�룄, 媛곸냽�룄) + AK8963(吏��옄湲�) �꽭�똿
 �넻�떊�솗�씤�븯怨� 5踰덇퉴吏� �떆�룄
 */
void initIMU ()
{
    uint8 readData = 0;
    uint8 trycnt = 0;
    forceI2CBusReset();   // 踰꾩뒪 媛뺤젣 由ъ뀑
    do
    {
        delay(100000);
        delay(100000);
        initI2c();// I2C 珥덇린�솕
        delay(1000000);

        initAK8963();// AK8963(吏��옄湲�)珥덇린�솕
        uint8 whoAmI = WHOAMI_REG;
        i2cWrite(MPU9250_ADDRESS, &whoAmI, 1);
        delay(10000);
        i2cRead(MPU9250_ADDRESS, &readData, 1);
        trycnt++;
    }while (readData != 0x71 && (trycnt < 5));
    setDLPF();
    delay(1000000);
    checkoffset();
}

/*
 initI2c �븿�닔
 MPU9250 IMU(媛��냽�룄, 媛곸냽�룄)瑜� �쐞�븳 I2C �꽭�똿
 媛��냽�룄, 媛곸냽�룄 �룞�옉紐⑤뱶 �꽕�젙
 */
void initI2c (void)
{

    IfxI2c_Pins i2cpinset = {&SCL_PIN, &SDA_PIN, IfxPort_PadDriver_cmosAutomotiveSpeed1};
    IfxI2c_I2c_Config i2cConfig;


    //�궗�슜�븷 i2c�꽕�젙
    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);
    i2cConfig.pins = &i2cpinset;
    i2cConfig.baudrate = 100000;   //�넻�떊 �냽�룄 100000;
    IfxI2c_I2c_initModule(&g_i2cMaster, &i2cConfig);
    g_i2cSet.i2c = &g_i2cMaster;
    g_i2cSet.deviceAddress = (MPU9250_ADDRESS << 1);

    uint8 initData[2] = {PWR_MGMT_1, 0x00}; //MPU�쟾�썝
    i2cWrite(MPU9250_ADDRESS, initData, 2); //Sleep�뿉�꽌 源⑥�
    delay(1000000);
    //媛��냽�룄, 媛곸냽�룄 �룞�옉紐⑤뱶
    uint8 initAccelData[2] = {ACCEL_CONFIG_REG, 0x00}; //+-2g
    i2cWrite(MPU9250_ADDRESS, initAccelData, 2);
    uint8 initGyroData[2] = {GYRO_CONFIG_REG, 0x00};  //+-250
    i2cWrite(MPU9250_ADDRESS, initGyroData, 2);
}

/*
 i2cWrite �븿�닔
 g_i2cSet i2c瑜� �넻�빐
 slave�뿉 write
 */
IfxI2c_I2c_Status i2cWrite (uint8 slaveAddress, uint8 *data, Ifx_SizeT length)
{
    g_i2cSet.deviceAddress = slaveAddress << 1;
    IfxI2c_I2c_Status status = IfxI2c_I2c_write(&g_i2cSet, data, length);
    delay(100);
    return status;

}

/*
 i2cRead �븿�닔
 g_i2cSet i2c瑜� �넻�빐
 length留뚰겮 �씫�뼱 data�뿉 ���옣
 */
void i2cRead (uint8 slaveAddress, uint8 *data, Ifx_SizeT length)
{
    g_i2cSet.deviceAddress = (slaveAddress << 1) | 1;
    IfxI2c_I2c_read(&g_i2cSet, data, length);
    delay(100);
}

/*
 imuRead �븿�닔
 imu媛� �씫�뼱���꽌 return�븯�뒗 �븿�닔
 */
IMU imuRead ()
{
    IMU now_imu={0,0,0,0,0,0,0,0,0,0};
    // 媛��냽�룄, 媛곸냽�룄, 吏��옄湲곌컪 �젒洹� �젅吏��뒪�꽣
    uint8 accelAddr = ACCEL_REG;
    uint8 gyroAddr = GYRO_REG;
    uint8 magAddr = MAG_REG;

    // �꽱�꽌 �뜲�씠�꽣 踰꾪띁
    uint8 accelData[6] = {0};      // 媛��냽�룄 �뜲�씠�꽣 踰꾪띁
    uint8 gyroData[6] = {0};       // �옄�씠濡� �뜲�씠�꽣 踰꾪띁
    uint8 magData[6] = {0};        // 吏��옄湲� �뜲�씠�꽣 踰꾪띁

    // 痢≪젙 raw 媛�
    sint16 accel_x_raw, accel_y_raw, accel_z_raw;    // 媛��냽�룄 媛� (16鍮꾪듃)
    sint16 gyro_x_raw, gyro_y_raw, gyro_z_raw;       // 媛곸냽�룄 媛� (16鍮꾪듃)
    sint16 mag_x_raw, mag_y_raw, mag_z_raw;          // 吏��옄湲� 媛� (16鍮꾪듃)

    // 寃곌낵 媛�
    float accel_x, accel_y, accel_z;    // 媛��냽�룄 媛� (16鍮꾪듃)
    float gyro_x, gyro_y, gyro_z;       // 媛곸냽�룄 媛� (16鍮꾪듃)
    float mag_x, mag_y, mag_z;          // 吏��옄湲� 媛� (16鍮꾪듃)
    float heading;




    // 媛��냽�룄 �뜲�씠�꽣
    i2cWrite(MPU9250_ADDRESS, &accelAddr, 1);
    i2cRead(MPU9250_ADDRESS, accelData, 6);

    // �옄�씠濡� �뜲�씠�꽣
    i2cWrite(MPU9250_ADDRESS, &gyroAddr, 1);
    i2cRead(MPU9250_ADDRESS, gyroData, 6);
    // 吏��옄湲� setting

     //stop condition
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

     //吏��옄湲� 媛� update�릱�뒗吏� check
    uint8 status_reg = AK_UPDATE_REG;       //drdy�솗�씤 reg
    i2cWrite(AK8963_ADDRESS, &status_reg, 1);
    delay(10);
    i2cRead(AK8963_ADDRESS, &status1_val, 1);

     //stop condition
    i2cStopCondition();
    delay(10);
    i2cStartCondition();
    // 吏��옄湲� �뜲�씠�꽣
    i2cWrite(AK8963_ADDRESS, &magAddr, 1);  // AK8963 I2C 二쇱냼
    delay(10);
    i2cRead(AK8963_ADDRESS, magData, 6);

    // ak read
    uint8 status2_reg = 0x09;
    i2cWrite(AK8963_ADDRESS, &status2_reg, 1);
    delay(10);
    i2cRead(AK8963_ADDRESS, &status2_val, 1);

    /// �떆�옉 -> �떒�쐞 泥댄겕 //////////////////////////////////////////////////////////////////////////////////////
    // 媛��냽�룄 �뜲�씠�꽣 蹂��솚
    accel_x_raw = (sint16) (accelData[0] << 8) | accelData[1];
    accel_y_raw = (sint16) (accelData[2] << 8) | accelData[3];
    accel_z_raw = (sint16) (accelData[4] << 8) | accelData[5];

    //媛먮룄 議곗젙 2g
    accel_x = ((float) accel_x_raw) / ACCEL_SEN - imu_offset.accel_x;
    accel_y = ((float) accel_y_raw) / ACCEL_SEN - imu_offset.accel_y;
    accel_z = ((float) accel_z_raw) / ACCEL_SEN - imu_offset.accel_z;

    // �옄�씠濡� �뜲�씠�꽣 蹂��솚
    gyro_x_raw = (sint16) (gyroData[0] << 8) | gyroData[1];
    gyro_y_raw = (sint16) (gyroData[2] << 8) | gyroData[3];
    gyro_z_raw = (sint16) (gyroData[4] << 8) | gyroData[5];

    //媛먮룄 議곗젙 250
    gyro_x = ((float) gyro_x_raw) / GYRO_SEN - imu_offset.gyro_x;//* (M_PI / 180.0f)
    gyro_y = ((float) gyro_y_raw) / GYRO_SEN - imu_offset.gyro_y;
    gyro_z = ((float) gyro_z_raw) / GYRO_SEN - imu_offset.gyro_z;

    // 吏��옄湲� �뜲�씠�꽣 蹂��솚
    mag_x_raw = (sint16) (magData[1] << 8) | magData[0];
    mag_y_raw = (sint16) (magData[3] << 8) | magData[2];
    mag_z_raw = (sint16) (magData[5] << 8) | magData[4];

    //媛먮룄 議곗젙
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
    /////.h�뙆�씪 784
    x_offset = (magx_max + magx_min) / 2;//�븘�몢�씠�끂�옉 �떎瑜닿쾶 �궃 �쐞�뿉�꽌 �씠誘� 援ы븿
    y_offset = (magy_max + magy_min) / 2;
    z_offset = (magz_max + magz_min) / 2;
    float scale_x_diff = magx_max - magx_min;
    float scale_y_diff = magy_max - magy_min;
    float scale_z_diff = magz_max - magz_min;
    avg_rad = (scale_x_diff + scale_y_diff + scale_z_diff)/ 3.0f;
    scale_x = avg_rad / scale_x_diff;
    scale_y = avg_rad / scale_y_diff;
    scale_z = avg_rad / scale_z_diff;

    //heading -> 遺곸そ 0, �삤瑜몄そ�쑝濡� +
    heading = atan2(mag_x, mag_y) *(180 / M_PI);
    if (heading < 0)
        heading += 360;
    now_imu.accel_x = accel_x;
    now_imu.accel_y = accel_y;
    now_imu.accel_z = accel_z;

    now_imu.gyro_x = gyro_x* (M_PI / 180.0f);
    now_imu.gyro_y = gyro_y* (M_PI / 180.0f);
    now_imu.gyro_z = gyro_z* (M_PI / 180.0f);

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


    //heading -> 遺곸そ 0, �삤瑜몄そ�쑝濡� +
    heading = atan2(now_imu.mag_x, now_imu.mag_y) * (180 / M_PI);
    if (heading < 0)
        heading += 360;

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
    IMU now_imu={0,0,0,0,0,0,0,0,0,0};
    // 媛��냽�룄, 媛곸냽�룄, 吏��옄湲곌컪 �젒洹� �젅吏��뒪�꽣
    uint8 accelAddr = ACCEL_REG;
    uint8 gyroAddr = GYRO_REG;

    // �꽱�꽌 �뜲�씠�꽣 踰꾪띁
    uint8 accelData[6] = {0};      // 媛��냽�룄 �뜲�씠�꽣 踰꾪띁
    uint8 gyroData[6] = {0};       // �옄�씠濡� �뜲�씠�꽣 踰꾪띁

    // 痢≪젙 raw 媛�
    sint16 accel_x_raw, accel_y_raw, accel_z_raw;    // 媛��냽�룄 媛� (16鍮꾪듃)
    sint16 gyro_x_raw, gyro_y_raw, gyro_z_raw;       // 媛곸냽�룄 媛� (16鍮꾪듃)

    // 寃곌낵 媛�
    float accel_x, accel_y, accel_z;    // 媛��냽�룄 媛� (16鍮꾪듃)
    float gyro_x, gyro_y, gyro_z;       // 媛곸냽�룄 媛� (16鍮꾪듃)


    // 媛��냽�룄 �뜲�씠�꽣
    i2cWrite(MPU9250_ADDRESS, &accelAddr, 1);
    i2cRead(MPU9250_ADDRESS, accelData, 6);

    // �옄�씠濡� �뜲�씠�꽣
    i2cWrite(MPU9250_ADDRESS, &gyroAddr, 1);
    i2cRead(MPU9250_ADDRESS, gyroData, 6);
    /// �떆�옉 -> �떒�쐞 泥댄겕 //////////////////////////////////////////////////////////////////////////////////////
    // 媛��냽�룄 �뜲�씠�꽣 蹂��솚
    accel_x_raw = (sint16) (accelData[0] << 8) | accelData[1];
    accel_y_raw = (sint16) (accelData[2] << 8) | accelData[3];
    accel_z_raw = (sint16) (accelData[4] << 8) | accelData[5];

    //媛먮룄 議곗젙 2g
    accel_x = ((float) accel_x_raw) / ACCEL_SEN;
    accel_y = ((float) accel_y_raw) / ACCEL_SEN;
    accel_z = ((float) accel_z_raw) / ACCEL_SEN;
    //0

    // �옄�씠濡� �뜲�씠�꽣 蹂��솚
    gyro_x_raw = (sint16) (gyroData[0] << 8) | gyroData[1];
    gyro_y_raw = (sint16) (gyroData[2] << 8) | gyroData[3];
    gyro_z_raw = (sint16) (gyroData[4] << 8) | gyroData[5];

    //媛먮룄 議곗젙 250
    gyro_x = ((float) gyro_x_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    gyro_y = ((float) gyro_y_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    gyro_z = ((float) gyro_z_raw) / GYRO_SEN;//* (M_PI / 180.0f);
///0 -> �떒�쐞 議곗젙�� �굹以묒뿉�씤媛�
    now_imu.accel_x = accel_x;
    now_imu.accel_y = accel_y;
    now_imu.accel_z = accel_z;

    now_imu.gyro_x = gyro_x;
    now_imu.gyro_y = gyro_y;
    now_imu.gyro_z = gyro_z;
////init�븿�닔�뒗 臾몄젣 �뾾�뒗�벏!!!!
    return now_imu;
}


/*
 initAK8963 �븿�닔
 AK8963(吏��옄湲�) �꽭�똿
 */

void initAK8963 (void)
{
    uint8 mag_mode_value = 0;
    uint8 mag_mode_valueoff = 1;
    uint8 status_valfir = 0;

    uint8 data[3];

    // I2C master 鍮꾪솢�꽦�솕 -> MPU媛� 留덉뒪�꽣媛� �릺吏� �븡寃� �꽕�젙-> �궡媛� �옄泥댁쟻�쑝濡� �넻�떊�븷嫄곕씪
    uint8 bypass_reg1[2] = {0x6A, 0x00};
    i2cWrite(MPU9250_ADDRESS, bypass_reg1, 2);
    delay(10000);
    // MPU9250 I2C bypass enable
    uint8 bypass_reg2[2] = {0x37, 0x02};
    i2cWrite(MPU9250_ADDRESS, bypass_reg2, 2);
    delay(10000);

    // AK8963 珥덇린�솕
    // Power down ->  ROM�뿉 �젒洹쇳븯�젮怨� 怨꾩냽 �옞源� �걚湲�
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_power_down[2] = {AK_CNTL1_REG, 0x00};     // CNTL1 �젅吏��뒪�꽣
    i2cWrite(AK8963_ADDRESS, mag_power_down, 2);
    delay(10000);

    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_mode_check = AK_CNTL1_REG;                   //offmode �옒 �뱾�뼱媛붾뒗吏�
    i2cWrite(AK8963_ADDRESS, &mag_mode_check, 1);
    i2cRead(AK8963_ADDRESS, &mag_mode_valueoff, 1);

    // Fuse ROM access mode-> ROM�뿉 �젒洹쇳빐�꽌 珥덇린 媛먮룄 �븣�븘�궡�젮怨�
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_rom_access[2] = {AK_CNTL1_REG, 0x0F};
    i2cWrite(AK8963_ADDRESS, mag_rom_access, 2);
    delay(10000);

    // 珥덇린 媛먮룄 �씫湲�
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 asa_reg = 0x10;  //媛먮룄 ���옣�맂嫄� �떆�옉 二쇱냼
    i2cWrite(AK8963_ADDRESS, &asa_reg, 1);
    delay(100);
    i2cRead(AK8963_ADDRESS, data, 3);

    // 媛먮룄 議곗젙媛� ���옣
    asa_x = ((float) (data[0] - 128))/ 256.0f + 1.0f;
    asa_y = ((float) (data[1] - 128)) / 256.0f + 1.0f;
    asa_z = ((float) (data[2] - 128)) / 256.0f + 1.0f;

    // Power down -> ROM�젒洹쇰え�뱶�뿉�꽌 �굹���꽌!!痢≪젙紐⑤뱶 �쟾�뿉 �븞�젙�솕�븯�젮怨�
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    i2cWrite(AK8963_ADDRESS, mag_power_down, 2);
    delay(10000);

    // �뿰�냽痢≪젙紐⑤뱶 �떆�옉
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    uint8 mag_continuous[2] = {AK_CNTL1_REG, 0x06};  //

    ///1�� �븳踰� �븯怨� off
    i2cWrite(AK8963_ADDRESS, mag_continuous, 2);
    //mag_mode_check = AK_CNTL1_REG;
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    i2cWrite(AK8963_ADDRESS, &mag_mode_check, 1);
    i2cRead(AK8963_ADDRESS, &mag_mode_value, 1);

    delay(10000);

    //�뾽�뜲�씠�듃 �릱�뒗吏� �솗�씤
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    //drdy�솗�씤 reg
    uint8 status_reg = 0x02;
    i2cWrite(AK8963_ADDRESS, &status_reg, 1);
    delay(10);
    i2cRead(AK8963_ADDRESS, &status_valfir, 1);

}

/*
 forceI2CBusReset �븿�닔
 Bus 珥덇린�솕
 */
void forceI2CBusReset (void)
{
    // I2C 紐⑤뱢 鍮꾪솢�꽦�솕
    IfxI2c_disableModule(&MODULE_I2C0);

    // SDA�� SCL�쓣 GPIO
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // �몮 �떎 high濡� �꽕�젙
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

    // STOP condition -> scl�씠 high�씪 �븣 sda low�뿉�꽌 high
    IfxPort_setPinLow(&MODULE_P13, 1);   //sda
    delay(1000);
    IfxPort_setPinHigh(&MODULE_P13, 2);  //scl
    delay(1000);
    IfxPort_setPinHigh(&MODULE_P13, 1);  //sda
    delay(10000);

    // �떎�떆 I2C ���쑝濡� �꽕�젙
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 i2cStopCondition �븿�닔
 i2c stop condition �깮�꽦
 */
void i2cStopCondition (void)
{
    // I2C ���쓣 GPIO 紐⑤뱶濡�
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // STOP condition -> scl�씠 high�씪 �븣 sda low�뿉�꽌 high
    IfxPort_setPinLow(&MODULE_P13, 1);  // SDA LOW
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 2); // SCL HIGH
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 1); // SDA HIGH (STOP)
    delay(10);

    // �떎�떆 I2C 紐⑤뱶
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 i2cStartCondition �븿�닔
 i2c start condition �깮�꽦
 */
void i2cStartCondition (void)
{
    //  I2C ���쓣 GPIO 紐⑤뱶濡�
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // Start Condition
    // SCL�쓣 HIGH濡� �꽕�젙
    IfxPort_setPinHigh(&MODULE_P13, 2); // SCL HIGH
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 1); // SDA HIGH
    delay(10);
    IfxPort_setPinLow(&MODULE_P13, 1);  // SDA LOW
    delay(10);

    // SCL�쓣 LOW濡� �꽕�젙 (�겢�윮 �떆�옉)
    IfxPort_setPinLow(&MODULE_P13, 2);  // SCL LOW
    delay(10);
    // I2C 紐⑤뱶濡� 蹂듦뎄
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 setDLPF �븿�닔
 low pass filter �꽕�젙
 -> �닔�젙�빐�빞�븿
 */
void setDLPF (void)
{
    uint8 gyro_dlpf[2] = {0x1A, 0x03};  // �옄�씠濡� DLPF 41Hz �꽕�젙
    i2cWrite(MPU9250_ADDRESS, gyro_dlpf, 2);
    delay(10);

    uint8 accel_dlpf[2] = {0x1D, 0x03};  // 媛��냽�룄 DLPF 21.2Hz �꽕�젙
    i2cWrite(MPU9250_ADDRESS, accel_dlpf, 2);
    delay(10);
}

/*
 checkoffset �븿�닔
 珥덇린 �삤瑜� 蹂댁젙
 -> �닔�젙�빐�빞�븿
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
    imu_offset.accel_z = now_statussum.accel_z/100-1;//以묐젰 �쁺�뼢 �젣嫄�

    imu_offset.gyro_x = now_statussum.gyro_x/100;
    imu_offset.gyro_y = now_statussum.gyro_y/100;
    imu_offset.gyro_z = now_statussum.gyro_z/100;
}

void initGPIO(void)
{
    IfxPort_setPinMode(&MODULE_P14, 0, IfxPort_Mode_inputPullUp);  //input?�눖以� ?�끉�젟
}

int Touch(void)
{
    TouchState = IfxPort_getPinState(&MODULE_P14, 0);
    if (TouchState == 1)
        return 1;
    else
        return 0;
}
