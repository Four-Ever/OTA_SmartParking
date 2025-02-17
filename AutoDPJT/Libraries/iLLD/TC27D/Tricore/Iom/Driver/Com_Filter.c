///*******************************************************************************
// * @file    Com_Filter.c
// * @brief   find roll, pitch, yaw
// * @version 1.0
// * @date    2025-02-14
// ******************************************************************************/
//
//#include "Com_Filter.h"
//#include "IMU_Driver.h"
//
//void calculateAnglesFromAccel(IMU* now_status, float *roll_accel, float *pitch_accel)
//{
//    float ax = 0;//now_status.accel_x;
//    float ay = 0;//now_status.accel_y;
//    float az = 0;//now_status.accel_z;
//    *roll_accel = atan2(ay, az) * 180.0f / M_PI;
//    *pitch_accel = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;
//}
//
//void complementaryFilter(float roll_accel, float pitch_accel,
//                          float gx, float gy, float gz,
//                          float dt, float *roll, float *pitch, float *yaw) {
//
//    // 이전 값에 자이로 적분 더하기
//    *roll = *roll + gx * dt;
//    *pitch = *pitch + gy * dt;
//    *yaw = *yaw + gz * dt;
//
//    // Roll과 Pitch에만 상보필터 적용 (alpha = 0.98)
//    *roll = 0.98f * (*roll) + 0.02f * roll_accel;
//    *pitch = 0.98f * (*pitch) + 0.02f * pitch_accel;
//
//    // yaw는 자이로만으로 적분 (보정 불가)
//    // 필요시 여기에 yaw 범위 제한 (0~360도 또는 -180~180도)
//}
//
