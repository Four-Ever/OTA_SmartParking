/*******************************************************************************
 * @file    Obstacle_Detection.c
 * @brief   Ultra Sonic distance
 * @version 1.0
 * @date    2025-02-07
 ******************************************************************************/
#include "Obstacle_Detection.h"
#include "STM_Interrupt.h"
int obstacle[OBSTACLE_NUM] = {0}; //0 : no obstacle, 1+ : obstacle detected
int parking_spot[NUM_ULTRA] = {0}; //0 : no parking spot, 1+ : parking spot detected
//#define PARKING_SPOT_WIDTH 20.0f
//#define FBOBSTACLE_WARNING 10.0f
//#define RLOBSTACLE_WARNING 4.0f
//#define PARKING_SPOT_LENGTH 25.0f
//static uint32 R_startime = 0;
//static uint32 R_endtime = 0;
//static uint32 L_startime = 0;
//static uint32 L_endtime = 0;
int detecting_spot[NUM_ULTRA] = {0};
float spotdistacne[NUM_ULTRA] = {0};
//double U8Curr_vel = 10; //�굹以묒뿉 extern�빐�삤湲컈m/s
int ultra[2]={0};
void Obstacle_get_All_Distance (void)
{
    ToF_get_All_Distance(); // put this code to task code ( work for synchronize recent distance data )

    if ((Distance[TOF0]) < FBOBSTACLE_WARNING*10) // uart0 ToF Data-> cm
        obstacle[F_OBSTACLE] = Distance[TOF0]/10;  //cm
    else
        obstacle[F_OBSTACLE] = 0;
    if ((Distance[TOF0]) < FBOBSTACLE_WARNING*10) // uart2 ToF Data
        obstacle[B_OBSTACLE] = Distance[TOF1]/10;  //cm
    else
        obstacle[B_OBSTACLE] = 0;
}
