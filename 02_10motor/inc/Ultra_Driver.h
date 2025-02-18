/*******************************************************************************
 * @file    Ultra_Driver.h
 * @brief   Ultra Sonic distance
 * @version 1.0
 * @date    2025-02-07
 ******************************************************************************/

#ifndef INC_Ultra_DRIVER_H_
#define INC_Ultra_DRIVER_H_

#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "IfxStm.h"
#define NUM_ULTRA 2
extern const IfxPort_Pin R_TRIG_PIN;
extern const IfxPort_Pin R_ECHO_PIN;
extern const IfxPort_Pin L_TRIG_PIN;
extern const IfxPort_Pin L_ECHO_PIN;
typedef enum
{
    R_ULTRA,
    L_ULTRA
}UltraID;


void initUltrasonic(void);
float getDistance(int);
void delay(uint32);
float R_getFilteredDistance(float);
float L_getFilteredDistance(float);
extern float Ultra_Distance[NUM_ULTRA];
#endif /* INC_Ultra_DRIVER_H_ */
