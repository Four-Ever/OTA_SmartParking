/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gitstanley.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-02-10 21:14:29
 */

/* Include Files */
#include "gitstanley.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "wrapToPi.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>
#include "ASCLIN_Shell_UART.h"

/* Function Definitions */
/*
 * clear; clc; close all;
 *
 * Arguments    : void
 * Return Type  : float
 */


//static const double waypoints[][2] = {
//    {0.0, -0.3},
//    {0.2, -0.25},
//    {0.4, -0.2},
//    {0.6, -0.1},
//    {0.8, 0},
//    {0.9, 0.1}
//};

static const double waypoints2[][2] = {
    {0.1, 0.2},
    {0.2, 0.4},
    {0.3, 0.5},
    {0.4, 0.6}
};


//static int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);

/* 차량 상태 변수 */
double x, y, theta;
static int current_wp_idx;
static bool isReversing;
static bool exitg1;
static const double L = 0.135;  // 차량 축거
static const double max_steer = 0.6981317;  // 최대 조향각 (40도)
static const double waypoint_tolerance = 0.015; // Waypoint 도달 허용 오차
static const double max_error = 7.0; // 경로 이탈 허용 범위
static const double Kstanley = 0.6; // Stanley Controller 이득 값
static const double PI = 3.14159265358979323846; // 🚀 M_PI 대신 사용
extern float stanelytheta;
/* 전역변수 정의 */
double waypoints[4][2];
double x, y, theta;
double v;
double steering_output=0;
int IsWPTrackingFinish = 0;
int Update_finished=0;
int num_waypoints=3;
float refStanely_rpm=0;



/* 초기화 함수 */
void initStanley(void) {
    x = 0.0;
    y = 0.0;
    theta = PI/2;
    current_wp_idx = 0;
    isReversing = false;
    exitg1 = false;
    IsWPTrackingFinish = 0;
    Update_finished=0;
}

void updateWaypoints(float new_waypoints[][2]) {
    memcpy(waypoints, new_waypoints, sizeof(double) * 2 * 4);
    Update_finished=1;
}

/* Stanley Controller 적용 함수 */
float gitstanley()
{
    if (Update_finished==1) {
    v=(double)U8Curr_vel/1000; //현재 차속 m/s
    theta=stanelytheta;

    /*종료 조건: 경로 이탈 또는 모든 Waypoint 도달 */
    if (exitg1) {
        return 0.0f;
    }

    double target_x = waypoints[current_wp_idx][0];
    double target_y = waypoints[current_wp_idx][1];

   /* 현재 목표 Waypoint와의 거리 계산 */
    double dx = target_x - x;
    double dy = target_y - y;
    double distance_to_wp = sqrt(dx * dx + dy * dy);

    /* 경로 이탈 감지 */
    if (distance_to_wp > max_error) {
        exitg1 = true;
    }

    /* Waypoint 도달 여부 확인 후 다음 Waypoint로 이동 */
    if (distance_to_wp < waypoint_tolerance && current_wp_idx + 1 < num_waypoints) {
        current_wp_idx++;
        target_x = waypoints[current_wp_idx][0];
        target_y = waypoints[current_wp_idx][1];
    }

    /* CTE(횡방향 오차) 계산 (embeddedStanley.m 방식 적용) */
    double path_angle = atan2(target_y - y, target_x - x);
    double heading_error = path_angle - theta;
    wrapToPi(&heading_error);
    double cte = sin(heading_error) * distance_to_wp;

    /* Stanley Controller 조향각 계산 */
    double steering_angle = atan2(Kstanley * cte, fabs(v)) - heading_error;

    /* 조향각 제한 적용 */
    steering_angle = fmax(fmin(steering_angle, max_steer), -max_steer);

    /* 차량 위치 업데이트 */
    x += v * cos(theta) * 0.01;  // 100ms 간격 이동
    y += v * sin(theta) * 0.01;
    theta -= v / L * tan(steering_angle) * 0.01;
    wrapToPi(&theta);

    /* 종료 조건을 만족하면 조향 입력 0 */
    if (exitg1 || current_wp_idx >= num_waypoints) {
        steering_output = 0;
        IsWPTrackingFinish = 1;
    }
    else { //종료조건이 아니면 계산한 steering 값 넣어주기
        steering_output = round(steering_angle * (180.0 / PI));  // DEGREE 변환
        if (steering_output >0){
            steering_output=steering_output+2;
        }
        else if (steering_output<0) {
            steering_output=steering_output-2;
        }
    }

    }
    return steering_output ;
}

float gitstanleycheck(void)
{

    v=(double)U8Curr_vel/1000; //현재 차속 m/s
    theta=stanelytheta;  //degree
    wrapToPi(&theta);   //radian

    /*종료 조건: 경로 이탈 또는 모든 Waypoint 도달 */
    if (exitg1) {
        return 0.0f;
    }

    double target_x = waypoints2[current_wp_idx][0];
    double target_y = waypoints2[current_wp_idx][1];

   /* 현재 목표 Waypoint와의 거리 계산 */
    double dx = target_x - x;
    double dy = target_y - y;
    double distance_to_wp = sqrt(dx * dx + dy * dy);

    /* 경로 이탈 감지 */
    if (distance_to_wp > max_error) {
        exitg1 = true;
    }

    /* Waypoint 도달 여부 확인 후 다음 Waypoint로 이동 */
    if (distance_to_wp < waypoint_tolerance && current_wp_idx + 1 < num_waypoints) {
        current_wp_idx++;
        target_x = waypoints2[current_wp_idx][0];
        target_y = waypoints2[current_wp_idx][1];
    }

    /* CTE(횡방향 오차) 계산 (embeddedStanley.m 방식 적용) */
    double path_angle = atan2(target_y - y, target_x - x);
    double heading_error = path_angle - theta;
    wrapToPi(&heading_error);
    double cte = sin(heading_error) * distance_to_wp;

    /* Stanley Controller 조향각 계산 */
    double steering_angle = atan2(Kstanley * cte, fabs(v)) - heading_error;

    /* 조향각 제한 적용 */
    steering_angle = fmax(fmin(steering_angle, max_steer), -max_steer);

    /* 차량 위치 업데이트 */
    x += v * cos(theta) * 0.01;  // 100ms 간격 이동
    y += v * sin(theta) * 0.01;
    theta -= v / L * tan(steering_angle) * 0.01;
    wrapToPi(&theta);

    /* 종료 조건을 만족하면 조향 입력 0 */
    if (exitg1 || current_wp_idx >= num_waypoints) {
        steering_output = 0;
        IsWPTrackingFinish = 1;
        refStanely_rpm=0.0;
    }
    else { //종료조건이 아니면 계산한 steering 값 넣어주기
        steering_output = round(steering_angle * (180.0 / PI));  // DEGREE 변환
        if (steering_output >0){
            steering_output=steering_output+2;
        }
        else if (steering_output<0) {
            steering_output=steering_output-2;
        }
    }
    refStanely_rpm=(0.1*(60*gear_ratio*1000))/circumference;

    return steering_output ;
}
/*
 * File trailer for gitstanley.c
 *
 * [EOF]
 */
