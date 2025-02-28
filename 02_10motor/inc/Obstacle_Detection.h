/*******************************************************************************
 * @file    Obstacle_Detection.h
 * @brief   Ultra Sonic distance
 * @version 1.0
 * @date    2025-02-07
 ******************************************************************************/

#ifndef INC_Obstacle_Detection_H_
#define INC_Obstacle_Detection_H_

#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "IfxStm.h"
#include "Ultra_Driver.h"
#include "TOF.h"
#define OBSTACLE_NUM 4

#define PARKING_SPOT_WIDTH 13.0f
#define FBOBSTACLE_WARNING 12.0f
#define RLOBSTACLE_WARNING 4.0f
#define PARKING_SPOT_LENGTH 30.0f  //25f

typedef enum
{
    F_OBSTACLE,
    B_OBSTACLE,
    R_OBSTACLE,
    L_OBSTACLE

}OBSTACLE;

typedef enum
{
    R_PARKING,
    L_PARKING
}PARKINGSPOT;

void Obstacle_get_All_Distance(void);
extern int obstacle[OBSTACLE_NUM];
extern int parking_spot[NUM_ULTRA];
extern int detecting_spot[NUM_ULTRA];
extern double Cal_TTCD(double currvel);
extern double Cal_TTCR(double currvel);

//test
extern float spotdistacne[NUM_ULTRA];
extern int ultra[2];
#endif /* INC_Obstacle_Detection_H_ */
