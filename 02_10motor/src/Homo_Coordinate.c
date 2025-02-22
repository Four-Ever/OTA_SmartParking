/**********************************************************************************************************************
 * \file EncMotor.c
 * \copyright Copyright (C) Infineon Technologies AG 2019
 * 
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of 
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 * 
 * Boost Software License - Version 1.0 - August 17th, 2003
 * 
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and 
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 * 
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all 
 * derivative works of the Software, unless such copies or derivative works are solely in the form of 
 * machine-executable object code generated by a source language processor.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN 
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 * IN THE SOFTWARE.
 *********************************************************************************************************************/


/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
#include "Homo_Coordinate.h"

/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/


/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
double cam_points[4][2];
double world_points[4][2];
int data_ready_flag=0;
int transform_finished=0;
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
    double H[3][3]={    //변환행렬
            {-0.368036656685526, 0.00548074912323151, 0.460028801621360},
            {1.89101234653543e-16, -1.44092634176838e-18, -8.66458536875457e-16},
            {-0.739568464147702, -0.0417553076516824, 0.322780828929984}
    };
    double HR[3][3]={    //변환행렬
            {-0.0316236308901110,0.318563501871994, 0.0408641414594270},
            {-0.00390675036688200,0.0360707378204600,0.0172395596781960},
            {-0.0723287470257600,0.693770336447941,0.638502210303548}
    };
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/

// 호모그래피 행렬 H를 사용하여 여러 개의 카메라 좌표를 일괄 변환
    void transform_points(double H[3][3], double cam_points[4][2], double world_points[4][2]) {
        transform_finished = 0;  // 변환 시작

        for (int i = 0; i < 4; i++) {
            double x = cam_points[i][0];  // 카메라 픽셀 x좌표
            double y = cam_points[i][1];  // 카메라 픽셀 y좌표

            // 동차좌표 변환 (Homogeneous Transformation)
            double a = H[0][0] * x + H[0][1] * y + H[0][2];
            double b = H[1][0] * x + H[1][1] * y + H[1][2];
            double c = H[2][0] * x + H[2][1] * y + H[2][2];

            // 정규화 (0으로 나누는 오류 방지)
            if (fabs(c) > 1e-6) {  // 작은 값에 대한 안정성 확보
                world_points[i][0] = a / c;
                world_points[i][1] = b / c;
            } else {
                // 변환 실패 시 원본 좌표 그대로 유지
                world_points[i][0] = world_points[i-1][0] + 0.106 ;
                world_points[i][1] = world_points[i-1][1];
            }
        }

        transform_finished = 1;  // 변환 완료
    }
    void transform_points_HR(double HR[3][3], double cam_points[4][2], double world_points[4][2]) {
        transform_finished = 0;  // 변환 시작

        for (int i = 0; i < 4; i++) {
            double x = cam_points[i][0];  // 카메라 픽셀 x좌표
            double y = cam_points[i][1];  // 카메라 픽셀 y좌표

            // 동차좌표 변환 (Homogeneous Transformation)
            double a = HR[0][0] * x + HR[0][1] * y + HR[0][2];
            double b = HR[1][0] * x + HR[1][1] * y + HR[1][2];
            double c = HR[2][0] * x + HR[2][1] * y + HR[2][2];

            // 정규화 (0으로 나누는 오류 방지)
            if (fabs(c) > 1e-6) {  // 작은 값에 대한 안정성 확보
                world_points[i][0] = a / c;
                world_points[i][1] = b / c;
            } else {
                // 변환 실패 시 원본 좌표 그대로 유지
                world_points[i][0] = world_points[i-1][0] + 0.106 ;
                world_points[i][1] = world_points[i-1][1];
            }
        }

        transform_finished = 1;  // 변환 완료
    }

void TransformData_Init(void){
    cam_points[0][0]=0;
    cam_points[0][1]=0;
    cam_points[1][0]=0;
    cam_points[1][1]=0;

    cam_points[2][0]=0;
    cam_points[2][1]=0;
    cam_points[3][0]=0;
    cam_points[3][1]=0;

    world_points[0][0]=0;
    world_points[0][1]=0;
    world_points[1][0]=0;
    world_points[1][1]=0;

    world_points[2][0]=0;
    world_points[2][1]=0;
    world_points[3][0]=0;
    world_points[3][1]=0;
};

void InitCampoints(void){
    for (int i = 0; i < 4; i++) {
        cam_points[i][0] = 0;
        cam_points[i][1] = 0;
    }
}

void InitWorldpoints(void){
    for (int i = 0; i < 4; i++) {
        world_points[i][0] = 0;
        world_points[i][1] = 0;
    }
}


/*********************************************************************************************************************/
