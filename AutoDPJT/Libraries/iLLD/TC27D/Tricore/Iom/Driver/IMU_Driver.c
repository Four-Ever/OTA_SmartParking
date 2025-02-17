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
extern float asa_x;
extern float asa_y;
extern float asa_z;
static IMU imu_offset={0,0,0,0,0,0,0,0,0,0};
static IfxI2c_I2c g_i2cMaster;
extern uint8 now13;
// ak status check reg
extern uint8 status1_val;
extern uint8 status2_val;
static float magx_max = -10000;
static float magy_max = -10000;
static float magz_max = -10000;
static float magx_min = 10000;
static float magy_min = 10000;
static float magz_min = 10000;
static float avg_rad = 0;

extern float scale_x;
extern float scale_y;
extern float scale_z;
extern float x_offset;
extern float y_offset;
extern float z_offset;

/*
 initIMU 함수
 MPU9250 IMU(가속도, 각속도) + AK8963(지자기) 세팅
 통신확인하고 5번까지 시도
 */
void initIMU ()
{
    uint8 readData = 0;
    uint8 trycnt = 0;
    forceI2CBusReset();   // 버스 강제 리셋
    do
    {
        delay(100000);
        delay(100000);
        initI2c();// I2C 초기화
        delay(1000000);

        initAK8963();// AK8963(지자기)초기화
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
 initI2c 함수
 MPU9250 IMU(가속도, 각속도)를 위한 I2C 세팅
 가속도, 각속도 동작모드 설정
 */
void initI2c (void)
{

    IfxI2c_Pins i2cpinset = {&SCL_PIN, &SDA_PIN, IfxPort_PadDriver_cmosAutomotiveSpeed1};
    IfxI2c_I2c_Config i2cConfig;


    //사용할 i2c설정
    IfxI2c_I2c_initConfig(&i2cConfig, &MODULE_I2C0);
    i2cConfig.pins = &i2cpinset;
    i2cConfig.baudrate = 100000;   //통신 속도 100000;
    IfxI2c_I2c_initModule(&g_i2cMaster, &i2cConfig);
    g_i2cSet.i2c = &g_i2cMaster;
    g_i2cSet.deviceAddress = (MPU9250_ADDRESS << 1);

    uint8 initData[2] = {PWR_MGMT_1, 0x00}; //MPU전원
    first = i2cWrite(MPU9250_ADDRESS, initData, 2); //Sleep에서 깨움
    now13=6;
    delay(1000000);
    //가속도, 각속도 동작모드
    uint8 initAccelData[2] = {ACCEL_CONFIG_REG, 0x00}; //+-2g
    second = i2cWrite(MPU9250_ADDRESS, initAccelData, 2);
    uint8 initGyroData[2] = {GYRO_CONFIG_REG, 0x00};  //+-250
    third = i2cWrite(MPU9250_ADDRESS, initGyroData, 2);
}

/*
 i2cWrite 함수
 g_i2cSet i2c를 통해
 slave에 write
 */
IfxI2c_I2c_Status i2cWrite (uint8 slaveAddress, uint8 *data, Ifx_SizeT length)
{
    g_i2cSet.deviceAddress = slaveAddress << 1;
    IfxI2c_I2c_Status status = IfxI2c_I2c_write(&g_i2cSet, data, length);
    delay(100);
    return status;

}

/*
 i2cRead 함수
 g_i2cSet i2c를 통해
 length만큼 읽어 data에 저장
 */
void i2cRead (uint8 slaveAddress, uint8 *data, Ifx_SizeT length)
{
    g_i2cSet.deviceAddress = (slaveAddress << 1) | 1;
    IfxI2c_I2c_read(&g_i2cSet, data, length);
    delay(100);
}

/*
 imuRead 함수
 imu값 읽어와서 return하는 함수
 */
IMU imuRead ()
{
    IMU now_imu={0,0,0,0,0,0,0,0,0,0};
    // 가속도, 각속도, 지자기값 접근 레지스터
    uint8 accelAddr = ACCEL_REG;
    uint8 gyroAddr = GYRO_REG;
    uint8 magAddr = MAG_REG;

    // 센서 데이터 버퍼
    uint8 accelData[6] = {0};      // 가속도 데이터 버퍼
    uint8 gyroData[6] = {0};       // 자이로 데이터 버퍼
    uint8 magData[6] = {0};        // 지자기 데이터 버퍼

    // 측정 raw 값
    sint16 accel_x_raw, accel_y_raw, accel_z_raw;    // 가속도 값 (16비트)
    sint16 gyro_x_raw, gyro_y_raw, gyro_z_raw;       // 각속도 값 (16비트)
    sint16 mag_x_raw, mag_y_raw, mag_z_raw;          // 지자기 값 (16비트)

    // 결과 값
    float accel_x, accel_y, accel_z;    // 가속도 값 (16비트)
    float gyro_x, gyro_y, gyro_z;       // 각속도 값 (16비트)
    float mag_x, mag_y, mag_z;          // 지자기 값 (16비트)
    float heading;




    // 가속도 데이터
    i2cWrite(MPU9250_ADDRESS, &accelAddr, 1);
    i2cRead(MPU9250_ADDRESS, accelData, 6);

    // 자이로 데이터
    i2cWrite(MPU9250_ADDRESS, &gyroAddr, 1);
    i2cRead(MPU9250_ADDRESS, gyroData, 6);

    now13=7;
    // 지자기 setting

     //stop condition
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

     //지자기 값 update됐는지 check
    uint8 status_reg = AK_UPDATE_REG;       //drdy확인 reg
    i2cWrite(AK8963_ADDRESS, &status_reg, 1);
    delay(10);
    i2cRead(AK8963_ADDRESS, &status1_val, 1);

     //stop condition
    i2cStopCondition();
    delay(10);
    i2cStartCondition();
    now13=8;
    // 지자기 데이터
    i2cWrite(AK8963_ADDRESS, &magAddr, 1);  // AK8963 I2C 주소
    delay(10);
    i2cRead(AK8963_ADDRESS, magData, 6);

    now13=10;
    // ak read
    uint8 status2_reg = 0x09;
    i2cWrite(AK8963_ADDRESS, &status2_reg, 1);
    delay(10);
    i2cRead(AK8963_ADDRESS, &status2_val, 1);
    now13=11;
    /// 시작 -> 단위 체크 //////////////////////////////////////////////////////////////////////////////////////
    // 가속도 데이터 변환
    accel_x_raw = (sint16) (accelData[0] << 8) | accelData[1];
    accel_y_raw = (sint16) (accelData[2] << 8) | accelData[3];
    accel_z_raw = (sint16) (accelData[4] << 8) | accelData[5];

    //감도 조정 2g
    accel_x = ((float) accel_x_raw) / ACCEL_SEN - imu_offset.accel_x;
    accel_y = ((float) accel_y_raw) / ACCEL_SEN - imu_offset.accel_y;
    accel_z = ((float) accel_z_raw) / ACCEL_SEN - imu_offset.accel_z;

    // 자이로 데이터 변환
    gyro_x_raw = (sint16) (gyroData[0] << 8) | gyroData[1];
    gyro_y_raw = (sint16) (gyroData[2] << 8) | gyroData[3];
    gyro_z_raw = (sint16) (gyroData[4] << 8) | gyroData[5];

    //감도 조정 250
    gyro_x = ((float) gyro_x_raw) / GYRO_SEN - imu_offset.gyro_x;//* (M_PI / 180.0f)
    gyro_y = ((float) gyro_y_raw) / GYRO_SEN - imu_offset.gyro_y;
    gyro_z = ((float) gyro_z_raw) / GYRO_SEN - imu_offset.gyro_z;

    // 지자기 데이터 변환
    mag_x_raw = (sint16) (magData[1] << 8) | magData[0];
    mag_y_raw = (sint16) (magData[3] << 8) | magData[2];
    mag_z_raw = (sint16) (magData[5] << 8) | magData[4];

    //감도 조정
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
    /////.h파일 784
    x_offset = (magx_max + magx_min) / 2;//아두이노랑 다르게 난 위에서 이미 구함
    y_offset = (magy_max + magy_min) / 2;
    z_offset = (magz_max + magz_min) / 2;
    float scale_x_diff = magx_max - magx_min;
    float scale_y_diff = magy_max - magy_min;
    float scale_z_diff = magz_max - magz_min;
    avg_rad = (scale_x_diff + scale_y_diff + scale_z_diff)/ 3.0f;
    scale_x = avg_rad / scale_x_diff;
    scale_y = avg_rad / scale_y_diff;
    scale_z = avg_rad / scale_z_diff;

    //heading -> 북쪽 0, 오른쪽으로 +
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


    //heading -> 북쪽 0, 오른쪽으로 +
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
    now13=9;
    IMU now_imu={0,0,0,0,0,0,0,0,0,0};
    // 가속도, 각속도, 지자기값 접근 레지스터
    uint8 accelAddr = ACCEL_REG;
    uint8 gyroAddr = GYRO_REG;

    // 센서 데이터 버퍼
    uint8 accelData[6] = {0};      // 가속도 데이터 버퍼
    uint8 gyroData[6] = {0};       // 자이로 데이터 버퍼

    // 측정 raw 값
    sint16 accel_x_raw, accel_y_raw, accel_z_raw;    // 가속도 값 (16비트)
    sint16 gyro_x_raw, gyro_y_raw, gyro_z_raw;       // 각속도 값 (16비트)

    // 결과 값
    float accel_x, accel_y, accel_z;    // 가속도 값 (16비트)
    float gyro_x, gyro_y, gyro_z;       // 각속도 값 (16비트)


    // 가속도 데이터
    i2cWrite(MPU9250_ADDRESS, &accelAddr, 1);
    i2cRead(MPU9250_ADDRESS, accelData, 6);

    // 자이로 데이터
    i2cWrite(MPU9250_ADDRESS, &gyroAddr, 1);
    i2cRead(MPU9250_ADDRESS, gyroData, 6);
    /// 시작 -> 단위 체크 //////////////////////////////////////////////////////////////////////////////////////
    // 가속도 데이터 변환
    accel_x_raw = (sint16) (accelData[0] << 8) | accelData[1];
    accel_y_raw = (sint16) (accelData[2] << 8) | accelData[3];
    accel_z_raw = (sint16) (accelData[4] << 8) | accelData[5];

    //감도 조정 2g
    accel_x = ((float) accel_x_raw) / ACCEL_SEN;
    accel_y = ((float) accel_y_raw) / ACCEL_SEN;
    accel_z = ((float) accel_z_raw) / ACCEL_SEN;
    //0

    // 자이로 데이터 변환
    gyro_x_raw = (sint16) (gyroData[0] << 8) | gyroData[1];
    gyro_y_raw = (sint16) (gyroData[2] << 8) | gyroData[3];
    gyro_z_raw = (sint16) (gyroData[4] << 8) | gyroData[5];

    //감도 조정 250
    gyro_x = ((float) gyro_x_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    gyro_y = ((float) gyro_y_raw) / GYRO_SEN;//* (M_PI / 180.0f);
    gyro_z = ((float) gyro_z_raw) / GYRO_SEN;//* (M_PI / 180.0f);
///0 -> 단위 조정은 나중에인가
    now_imu.accel_x = accel_x;
    now_imu.accel_y = accel_y;
    now_imu.accel_z = accel_z;

    now_imu.gyro_x = gyro_x;
    now_imu.gyro_y = gyro_y;
    now_imu.gyro_z = gyro_z;
////init함수는 문제 없는듯!!!!
    return now_imu;
}


/*
 initAK8963 함수
 AK8963(지자기) 세팅
 */

void initAK8963 (void)
{
    uint8 mag_mode_value = 0;
    uint8 mag_mode_valueoff = 1;
    uint8 status_valfir = 0;

    uint8 data[3];

    // I2C master 비활성화 -> MPU가 마스터가 되지 않게 설정-> 내가 자체적으로 통신할거라
    uint8 bypass_reg1[2] = {0x6A, 0x00};
    test1=i2cWrite(MPU9250_ADDRESS, bypass_reg1, 2);
    delay(10000);
    // MPU9250 I2C bypass enable
    uint8 bypass_reg2[2] = {0x37, 0x02};
    test2=i2cWrite(MPU9250_ADDRESS, bypass_reg2, 2);
    delay(10000);

    // AK8963 초기화
    // Power down ->  ROM에 접근하려고 계속 잠깐 끄기
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_power_down[2] = {AK_CNTL1_REG, 0x00};     // CNTL1 레지스터
    test3=i2cWrite(AK8963_ADDRESS, mag_power_down, 2);
    delay(10000);

    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_mode_check = AK_CNTL1_REG;                   //offmode 잘 들어갔는지
    test4=i2cWrite(AK8963_ADDRESS, &mag_mode_check, 1);
    i2cRead(AK8963_ADDRESS, &mag_mode_valueoff, 1);

    // Fuse ROM access mode-> ROM에 접근해서 초기 감도 알아내려고
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 mag_rom_access[2] = {AK_CNTL1_REG, 0x0F};
    test5=i2cWrite(AK8963_ADDRESS, mag_rom_access, 2);
    delay(10000);

    // 초기 감도 읽기
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    uint8 asa_reg = 0x10;  //감도 저장된거 시작 주소
    test6=i2cWrite(AK8963_ADDRESS, &asa_reg, 1);
    delay(100);
    i2cRead(AK8963_ADDRESS, data, 3);

    // 감도 조정값 저장
    asa_x = ((float) (data[0] - 128))/ 256.0f + 1.0f;
    asa_y = ((float) (data[1] - 128)) / 256.0f + 1.0f;
    asa_z = ((float) (data[2] - 128)) / 256.0f + 1.0f;

    // Power down -> ROM접근모드에서 나와서!!측정모드 전에 안정화하려고
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    test7=i2cWrite(AK8963_ADDRESS, mag_power_down, 2);
    delay(10000);

    // 연속측정모드 시작
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    uint8 mag_continuous[2] = {AK_CNTL1_REG, 0x06};  //

    ///1은 한번 하고 off
    test8=i2cWrite(AK8963_ADDRESS, mag_continuous, 2);
    //mag_mode_check = AK_CNTL1_REG;
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();
    i2cWrite(AK8963_ADDRESS, &mag_mode_check, 1);
    i2cRead(AK8963_ADDRESS, &mag_mode_value, 1);

    delay(10000);

    //업데이트 됐는지 확인
    i2cStopCondition();
    delay(1000);
    i2cStartCondition();

    //drdy확인 reg
    uint8 status_reg = 0x02;
    i2cWrite(AK8963_ADDRESS, &status_reg, 1);
    delay(10);
    i2cRead(AK8963_ADDRESS, &status_valfir, 1);

}

/*
 forceI2CBusReset 함수
 Bus 초기화
 */
void forceI2CBusReset (void)
{
    // I2C 모듈 비활성화
    IfxI2c_disableModule(&MODULE_I2C0);

    // SDA와 SCL을 GPIO
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // 둘 다 high로 설정
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

    // STOP condition -> scl이 high일 때 sda low에서 high
    IfxPort_setPinLow(&MODULE_P13, 1);   //sda
    delay(1000);
    IfxPort_setPinHigh(&MODULE_P13, 2);  //scl
    delay(1000);
    IfxPort_setPinHigh(&MODULE_P13, 1);  //sda
    delay(10000);

    // 다시 I2C 핀으로 설정
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 i2cStopCondition 함수
 i2c stop condition 생성
 */
void i2cStopCondition (void)
{
    // I2C 핀을 GPIO 모드로
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // STOP condition -> scl이 high일 때 sda low에서 high
    IfxPort_setPinLow(&MODULE_P13, 1);  // SDA LOW
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 2); // SCL HIGH
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 1); // SDA HIGH (STOP)
    delay(10);

    // 다시 I2C 모드
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 i2cStartCondition 함수
 i2c start condition 생성
 */
void i2cStartCondition (void)
{
    //  I2C 핀을 GPIO 모드로
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputPushPullGeneral); // SDA
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputPushPullGeneral); // SCL

    // Start Condition
    // SCL을 HIGH로 설정
    IfxPort_setPinHigh(&MODULE_P13, 2); // SCL HIGH
    delay(10);
    IfxPort_setPinHigh(&MODULE_P13, 1); // SDA HIGH
    delay(10);
    IfxPort_setPinLow(&MODULE_P13, 1);  // SDA LOW
    delay(10);

    // SCL을 LOW로 설정 (클럭 시작)
    IfxPort_setPinLow(&MODULE_P13, 2);  // SCL LOW
    delay(10);
    // I2C 모드로 복구
    IfxPort_setPinMode(&MODULE_P13, 1, IfxPort_Mode_outputOpenDrainAlt6);
    IfxPort_setPinMode(&MODULE_P13, 2, IfxPort_Mode_outputOpenDrainAlt6);
}

/*
 setDLPF 함수
 low pass filter 설정
 -> 수정해야함
 */
void setDLPF (void)
{
    uint8 gyro_dlpf[2] = {0x1A, 0x03};  // 자이로 DLPF 41Hz 설정
    i2cWrite(MPU9250_ADDRESS, gyro_dlpf, 2);
    delay(10);

    uint8 accel_dlpf[2] = {0x1D, 0x03};  // 가속도 DLPF 21.2Hz 설정
    i2cWrite(MPU9250_ADDRESS, accel_dlpf, 2);
    delay(10);
}

/*
 checkoffset 함수
 초기 오류 보정
 -> 수정해야함
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
    imu_offset.accel_z = now_statussum.accel_z/100-1;//중력 영향 제거

    imu_offset.gyro_x = now_statussum.gyro_x/100;
    imu_offset.gyro_y = now_statussum.gyro_y/100;
    imu_offset.gyro_z = now_statussum.gyro_z/100;
}
