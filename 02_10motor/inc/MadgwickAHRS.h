/*******************************************************************************
 * @file    MadgwickAHRS.h
 * @brief   calculate roll, pitch, yaw with Madgwick filter
 * @version 1.0
 * @date    2025-02-17
 ******************************************************************************/

#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include "IMU_Driver.h"
#include "IfxStm.h"
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;             // algorithm gain


typedef struct {
        float roll;
        float pitch;
        float yaw;
} Euler;

//---------------------------------------------------------------------------------------------------
// Function declarations

Euler MadgwickAHRSupdate(IMU);
Euler MadgwickAHRSupdateIMU(IMU);
Euler GetEulerAngles();
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
