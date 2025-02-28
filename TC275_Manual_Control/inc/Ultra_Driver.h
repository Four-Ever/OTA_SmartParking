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



void initUltrasonic();
float getDistance();
void delay(uint32);

#endif /* INC_Ultra_DRIVER_H_ */
