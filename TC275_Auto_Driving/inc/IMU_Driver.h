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
#define ACCEL_CONFIG_REG 0x1C  // 媛��냽�룄 init Reg
#define GYRO_CONFIG_REG  0x1B  // 媛곸냽�룄 init Reg
#define MAG_CONFIG_REG   //吏��옄湲� Reg

#define ACCEL_REG 0x3B    //�뿊�� �떆�옉 reg二쇱냼
#define GYRO_REG  0x43    //�옄�씠濡� �떆�옉 reg二쇱냼
#define MAG_REG  0x03  // AK8963 mag �떆�옉 reg二쇱냼
#define WHOAMI_REG 0x75



#define ACCEL_SEN 16384.0f //2g -> Accel 媛먮룄 議곗젙
#define GYRO_SEN 131.072f //250�룄/s  -> Gyro 媛먮룄 議곗젙
#define MAG_SEN 5.997558f //uT/LSB -> 吏��옄湲� 媛먮룄 議곗젙10怨깊빐�꽌 mG�떒�쐞

#define SCL_PIN  IfxI2c0_SCL_P13_1_INOUT
#define SDA_PIN  IfxI2c0_SDA_P13_2_INOUT

#define PWR_MGMT_1 0x6B   //MPU Power Reg
#define AK_UPDATE_REG 0x02 //AK Update drdy check reg
#define AK_CNTL1_REG 0x0A
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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
int Touch(void);
//void initGPIO(void);

#endif /* INC_IMU_DRIVER_H_ */
