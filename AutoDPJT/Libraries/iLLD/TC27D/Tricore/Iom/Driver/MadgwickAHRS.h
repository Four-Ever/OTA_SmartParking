//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
//
//=====================================================================================================
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
