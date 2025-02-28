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

#define FBOBSTACLE_WARNING 10.0f
#define RLOBSTACLE_WARNING 4.0f

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

#endif /* INC_Obstacle_Detection_H_ */
