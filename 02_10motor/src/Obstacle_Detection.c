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
//static uint32 R_starttime = 0;
//static uint32 R_endtime = 0;
//static uint32 L_starttime = 0;
//static uint32 L_endtime = 0;

static float32 R_startdis = 0;
static float32 R_enddis = 0;
static float32 L_startdis = 0;
static float32 L_enddis = 0;

int detecting_spot[NUM_ULTRA] = {0};
float spotdistacne[NUM_ULTRA] = {0};
//double U8Curr_vel = 10; //�뜝�럡�룎繞벿살탳�굢占� extern�뜝�럥�돵�뜝�럩沅롧뼨�곹뜎m/s
int ultra[2]={0};
//void Obstacle_get_All_Distance (void)
//{
//    Ultra_Distance[R_ULTRA] = getDistance(R_ULTRA);
//    ultra[0]=Ultra_Distance[R_ULTRA];
//    if (Ultra_Distance[R_ULTRA] > PARKING_SPOT_LENGTH)
//    {
//        obstacle[R_OBSTACLE] = 0;///
//        if (detecting_spot[R_ULTRA] == 0) //嶺뚳퐣瑗뤄옙踰� �뜝�럡�뒄嶺뚯쉻�삕
//        {
//            R_starttime = MODULE_STM0.TIM0.U;
//            detecting_spot[R_ULTRA] = 1; //start
//        }
//        else if (detecting_spot[R_ULTRA] == 1)
//        {
//
//            R_endtime = MODULE_STM0.TIM0.U;
//            spotdistacne[R_ULTRA] += (float) (R_endtime - R_starttime) / IfxStm_getFrequency(&MODULE_STM0) * U8Curr_vel;//cm�뜝�럥堉듿뜝�럩留�
//            if (spotdistacne[R_ULTRA] > PARKING_SPOT_WIDTH)
//            {
//                parking_spot[R_ULTRA] = 1;
//            }
//            R_starttime = R_endtime;
//        }
//    }
//    else
//    {
//        spotdistacne[R_ULTRA] = 0;
//        detecting_spot[R_ULTRA] = 0;
//        parking_spot[R_ULTRA] = 0;
//        if (Ultra_Distance[R_ULTRA] < RLOBSTACLE_WARNING)
//            obstacle[R_OBSTACLE] = (int)Ultra_Distance[R_ULTRA];
//        else
//            obstacle[R_OBSTACLE] = 0;
//
//    }
//    Ultra_Distance[L_ULTRA] = getDistance(L_ULTRA);
//    ultra[1]=Ultra_Distance[L_ULTRA];
//    if (Ultra_Distance[L_ULTRA] > PARKING_SPOT_LENGTH)
//    {
//        obstacle[L_OBSTACLE] = 0;
//        if (detecting_spot[L_ULTRA] == 0) //嶺뚳퐣瑗뤄옙踰� �뜝�럡�뒄嶺뚯쉻�삕
//        {
//            L_starttime = MODULE_STM0.TIM0.U;
//            detecting_spot[L_ULTRA] = 1; //start
//        }
//        else if (detecting_spot[L_ULTRA] == 1)
//        {
//
//            L_endtime = MODULE_STM0.TIM0.U;
//            spotdistacne[L_ULTRA] += (float) (L_endtime - L_starttime) / IfxStm_getFrequency(&MODULE_STM0) * U8Curr_vel;//cm�뜝�럥堉듿뜝�럩留�
//            if (spotdistacne[L_ULTRA] > PARKING_SPOT_WIDTH)
//            {
//                parking_spot[L_ULTRA] = 1;
//            }
//            L_starttime = L_endtime;
//        }
//    }
//    else
//    {
//        spotdistacne[L_ULTRA] = 0;
//        detecting_spot[L_ULTRA] = 0;
//        parking_spot[L_ULTRA] = 0;
//        if (Ultra_Distance[L_ULTRA] < RLOBSTACLE_WARNING)
//            obstacle[L_OBSTACLE] = (int)Ultra_Distance[L_ULTRA];
//        else
//            obstacle[L_OBSTACLE] = 0;
//    }
//    ToF_get_All_Distance(); // put this code to task code ( work for synchronize recent distance data )
////    Distance[TOF0]=10;
////    Distance[TOF1]=20;
//    //Distance[TOF0]/=10;//Distance[TOF0];
//    //Distance[TOF1]/=10;//Distance[TOF1];
////    obstacle[F_OBSTACLE] = Distance[TOF0];
////    obstacle[B_OBSTACLE] = 0;
//    if ((Distance[TOF0]) < FBOBSTACLE_WARNING*10) // uart0 ToF Data-> cm
//        obstacle[F_OBSTACLE] = Distance[TOF0]/10;  //cm
//    else
//        obstacle[F_OBSTACLE] = 0;
//    if ((Distance[TOF0]) < FBOBSTACLE_WARNING*10) // uart2 ToF Data
//        obstacle[B_OBSTACLE] = Distance[TOF1]/10;  //cm
//    else
//        obstacle[B_OBSTACLE] = 0;
//
//}
void Obstacle_get_All_Distance (void)
{
    Ultra_Distance[R_ULTRA] = getDistance(R_ULTRA);
    ultra[0]=Ultra_Distance[R_ULTRA];
    if (Ultra_Distance[R_ULTRA] > PARKING_SPOT_LENGTH)
    {
        obstacle[R_OBSTACLE] = 0;///
        if (detecting_spot[R_ULTRA] == 0) //嶺뚳퐣瑗뤄옙踰� �뜝�럡�뒄嶺뚯쉻�삕
        {
            R_startdis=speed_pid.DisSum;
            detecting_spot[R_ULTRA] = 1; //start
        }
        else if (detecting_spot[R_ULTRA] == 1)
        {

            R_enddis = speed_pid.DisSum;
            spotdistacne[R_ULTRA] = (float) (R_enddis-R_startdis)/10;
            if (spotdistacne[R_ULTRA] > PARKING_SPOT_WIDTH)
            {
                parking_spot[R_ULTRA] = 1;
                R_startdis=0;
                R_enddis=0;
                //detecting_spot[R_ULTRA]=0;

            }
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
//    Ultra_Distance[L_ULTRA] = getDistance(L_ULTRA);
//    ultra[1]=Ultra_Distance[L_ULTRA];
//    if (Ultra_Distance[L_ULTRA] > PARKING_SPOT_LENGTH)
//    {
//        obstacle[L_OBSTACLE] = 0;
//        if (detecting_spot[L_ULTRA] == 0) //嶺뚳퐣瑗뤄옙踰� �뜝�럡�뒄嶺뚯쉻�삕
//        {
//            L_startdis=speed_pid.DisSum;
//            detecting_spot[L_ULTRA] = 1; //start
//        }
//        else if (detecting_spot[L_ULTRA] == 1)
//        {
//
//            L_enddis=speed_pid.DisSum;
//            spotdistacne[L_ULTRA] = (float) (L_enddis-L_startdis)/10;
//            if (spotdistacne[L_ULTRA] > PARKING_SPOT_WIDTH)
//            {
//                parking_spot[L_ULTRA] = 1;
//            }
//        }
//    }
//    else
//    {
//        spotdistacne[L_ULTRA] = 0;
//        detecting_spot[L_ULTRA] = 0;
//        parking_spot[L_ULTRA] = 0;
//        if (Ultra_Distance[L_ULTRA] < RLOBSTACLE_WARNING)
//            obstacle[L_OBSTACLE] = (int)Ultra_Distance[L_ULTRA];
//        else
//            obstacle[L_OBSTACLE] = 0;
//    }
    ToF_get_All_Distance(); // put this code to task code ( work for synchronize recent distance data )
//    Distance[TOF0]=10;
//    Distance[TOF1]=20;
    //Distance[TOF0]/=10;//Distance[TOF0];
    //Distance[TOF1]/=10;//Distance[TOF1];
//    obstacle[F_OBSTACLE] = Distance[TOF0];
//    obstacle[B_OBSTACLE] = 0;
    if ((Distance[TOF0]) < FBOBSTACLE_WARNING*10) // uart0 ToF Data-> cm
        obstacle[F_OBSTACLE] = Distance[TOF0]/10;  //cm
    else
        obstacle[F_OBSTACLE] = 0;
    if ((Distance[TOF1]) < FBOBSTACLE_WARNING*10) // uart2 ToF Data
        obstacle[B_OBSTACLE] = Distance[TOF1]/10;  //cm
    else
        obstacle[B_OBSTACLE] = 0;

}

double Cal_TTCD(double currvel) {
    double TTC;
    if (currvel !=0 && obstacle[F_OBSTACLE] !=0 ) {
        TTC = ((double)obstacle[F_OBSTACLE]*10)/(currvel);

        const double MAX_TTC_SECONDS = 5.0;  // 5초 이상은 1로 제한
                if (TTC > MAX_TTC_SECONDS) {
                    TTC = MAX_TTC_SECONDS;
                }
                TTC = TTC / MAX_TTC_SECONDS;  // 0~1 사이 값으로 정규화
    }

    else TTC=0;

    return TTC;
}

double Cal_TTCR(double currvel) {
    double TTC;
    if (currvel !=0 && obstacle[B_OBSTACLE] !=0 ) {
        TTC =((double)obstacle[B_OBSTACLE]*10) / (currvel);

        const double MAX_TTC_SECONDS = 5.0;  // 5초 이상은 1로 제한
                if (TTC > MAX_TTC_SECONDS) {
                    TTC = MAX_TTC_SECONDS;
                }
                TTC = TTC / MAX_TTC_SECONDS;  // 0~1 사이 값으로 정규화
    }
    else TTC=0;

    return TTC;

}
