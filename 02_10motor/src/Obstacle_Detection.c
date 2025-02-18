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
static uint32 R_startime = 0;
static uint32 R_endtime = 0;
static uint32 L_startime = 0;
static uint32 L_endtime = 0;
int detecting_spot[NUM_ULTRA] = {0};
float spotdistacne[NUM_ULTRA] = {0};
//double U8Curr_vel = 10; //�굹以묒뿉 extern�빐�삤湲컈m/s
int ultra[2]={0};
void Obstacle_get_All_Distance (void)
{
    Ultra_Distance[R_ULTRA] = getDistance(R_ULTRA);
    ultra[0]=Ultra_Distance[R_ULTRA];
    if (Ultra_Distance[R_ULTRA] > PARKING_SPOT_LENGTH)
    {
        obstacle[R_OBSTACLE] = 0;///
        if (detecting_spot[R_ULTRA] == 0) //泥섏쓬 �깘吏�
        {
            R_startime = MODULE_STM0.TIM0.U;
            detecting_spot[R_ULTRA] = 1; //start
        }
        else if (detecting_spot[R_ULTRA] == 1)
        {

            R_endtime = MODULE_STM0.TIM0.U;
            spotdistacne[R_ULTRA] += (float) (R_endtime - R_startime) / IfxStm_getFrequency(&MODULE_STM0) * U8Curr_vel;//cm�떒�쐞
            if (spotdistacne[R_ULTRA] > PARKING_SPOT_WIDTH)
            {
                parking_spot[R_ULTRA] = 1;
            }
            R_startime = R_endtime;
        }
    }
    else
    {
        spotdistacne[R_ULTRA] = 0;
        detecting_spot[R_ULTRA] = 0;
        parking_spot[R_ULTRA] = 0;
        if (Ultra_Distance[R_ULTRA] < RLOBSTACLE_WARNING)
            obstacle[R_OBSTACLE] = (int)Ultra_Distance[R_ULTRA];
        else
            obstacle[R_OBSTACLE] = 0;

    }
    Ultra_Distance[L_ULTRA] = getDistance(L_ULTRA);
    ultra[1]=Ultra_Distance[L_ULTRA];
    if (Ultra_Distance[L_ULTRA] > PARKING_SPOT_LENGTH)
    {
        obstacle[L_OBSTACLE] = 0;
        if (detecting_spot[L_ULTRA] == 0) //泥섏쓬 �깘吏�
        {
            L_startime = MODULE_STM0.TIM0.U;
            detecting_spot[L_ULTRA] = 1; //start
        }
        else if (detecting_spot[L_ULTRA] == 1)
        {

            L_endtime = MODULE_STM0.TIM0.U;
            spotdistacne[L_ULTRA] += (float) (L_endtime - L_startime) / IfxStm_getFrequency(&MODULE_STM0) * U8Curr_vel;//cm�떒�쐞
            if (spotdistacne[L_ULTRA] > PARKING_SPOT_WIDTH)
            {
                parking_spot[L_ULTRA] = 1;
            }
            L_startime = L_endtime;
        }
    }
    else
    {
        spotdistacne[L_ULTRA] = 0;
        detecting_spot[L_ULTRA] = 0;
        parking_spot[L_ULTRA] = 0;
        if (Ultra_Distance[L_ULTRA] < RLOBSTACLE_WARNING)
            obstacle[L_OBSTACLE] = (int)Ultra_Distance[L_ULTRA];
        else
            obstacle[L_OBSTACLE] = 0;
    }
    ToF_get_All_Distance(); // put this code to task code ( work for synchronize recent distance data )
//    Distance[TOF0]=10;
//    Distance[TOF1]=20;
    //Distance[TOF0]/=10;//Distance[TOF0];
    //Distance[TOF1]/=10;//Distance[TOF1];
//    obstacle[F_OBSTACLE] = Distance[TOF0];
//    obstacle[B_OBSTACLE] = 0;
    if ((Distance[TOF0]/10) < FBOBSTACLE_WARNING*10) // uart0 ToF Data-> cm
        obstacle[F_OBSTACLE] = Distance[TOF0]/10;  //cm
    else
        obstacle[F_OBSTACLE] = 0;
    if ((Distance[TOF0]/10) < FBOBSTACLE_WARNING*10) // uart2 ToF Data
        obstacle[B_OBSTACLE] = Distance[TOF1]/10;  //cm
    else
        obstacle[B_OBSTACLE] = 0;

}

double Cal_TTCD(double currvel) {
    double TTC;
    if (currvel !=0) {
        TTC = ((double)obstacle[F_OBSTACLE]/1000)/(currvel);
    }
    else TTC=0;

    return TTC;
}

double Cal_TTCR(double currvel) {
    double TTC;
    if (currvel !=0) {
        TTC =((double)obstacle[F_OBSTACLE]/1000) / (currvel);
    }
    else TTC=0;

    return TTC;

}
