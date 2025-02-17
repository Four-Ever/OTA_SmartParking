/*******************************************************************************
 * @file    IMU_Driver.h
 * @brief   IMU_accel, gyro,mag,heading without filter
 * @version 1.0
 * @date    2025-02-12
 ******************************************************************************/

#ifndef INC_IMU_DRIVER_H_
#define INC_IMU_DRIVER_H_

#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "IfxStm.h"
#include "IfxI2c_I2c.h"
#include <math.h>
#include "Ultra_Driver.h"
#include "MadgwickAHRS.h"


/***********************************************************************/
/*Typedef*/
/***********************************************************************/
typedef struct {
    float32 accel_x;
    float32 accel_y;
    float32 accel_z;
    float32 gyro_x;
    float32 gyro_y;
    float32 gyro_z;
    float32 mag_x;
    float32 mag_y;
    float32 mag_z;
    float32 heading;
} IMU;

/***********************************************************************/
/*Define*/
/***********************************************************************/
#define MPU9250_ADDRESS  0x68  //MPU I2C Slave Addr
#define AK8963_ADDRESS 0x0C    //AK8963 Slave Addr
#define ACCEL_CONFIG_REG 0x1C  // 가속도 init Reg
#define GYRO_CONFIG_REG  0x1B  // 각속도 init Reg
#define MAG_CONFIG_REG   //지자기 Reg

#define ACCEL_REG 0x3B    //엑셀 시작 reg주소
#define GYRO_REG  0x43    //자이로 시작 reg주소
#define MAG_REG  0x03  // AK8963 mag 시작 reg주소
#define WHOAMI_REG 0x75



#define ACCEL_SEN 16384.0f //2g -> Accel 감도 조정
#define GYRO_SEN 131.072f //250도/s  -> Gyro 감도 조정
#define MAG_SEN 5.997558f //uT/LSB -> 지자기 감도 조정10곱해서 mG단위

#define SCL_PIN  IfxI2c0_SCL_P13_1_INOUT
#define SDA_PIN  IfxI2c0_SDA_P13_2_INOUT

#define PWR_MGMT_1 0x6B   //MPU Power Reg
#define AK_UPDATE_REG 0x02 //AK Update drdy check reg
#define AK_CNTL1_REG 0x0A
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif



//#define scale_x 0.8435654f
//#define scale_y 1.0019939f
//#define scale_z 1.2246717f
//#define x_offset -24.517365f
//#define y_offset 193.28069f
//#define z_offset -20.312042f
extern IfxI2c_I2c_Status first;
extern IfxI2c_I2c_Status second;
extern IfxI2c_I2c_Status third;
extern IfxI2c_I2c_Status fourth;
extern IfxI2c_I2c_Status test1;
extern IfxI2c_I2c_Status test2;
extern IfxI2c_I2c_Status test3;
extern IfxI2c_I2c_Status test4;
extern IfxI2c_I2c_Status test5;
extern IfxI2c_I2c_Status test6;
extern IfxI2c_I2c_Status test7;
extern IfxI2c_I2c_Status test8;
extern IfxI2c_I2c_Status test9;




/***********************************************************************/
/*Global Function Prototype*/
/***********************************************************************/
void initIMU (void);
void initI2c (void);
IfxI2c_I2c_Status i2cWrite (uint8, uint8*, Ifx_SizeT);
void i2cRead (uint8, uint8*, Ifx_SizeT);
IMU imuRead (void);
IMU initimuRead (void);
void initAK8963 (void);
void forceI2CBusReset (void);
void i2cStopCondition (void);
void i2cStartCondition (void);
void setDLPF(void);
void checkoffset(void);


#endif /* INC_IMU_DRIVER_H_ */
