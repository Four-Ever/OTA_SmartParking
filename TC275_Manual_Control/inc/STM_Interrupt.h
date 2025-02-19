/**********************************************************************************************************************
 * \file STM_Interrupt.h
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

#ifndef STM_INTERRUPT_H_
#define STM_INTERRUPT_H_

#include "Bsp.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "Ifx_Types.h"

#include "IfxGpt12_IncrEnc.h"
#include "EncMotor.h"

#include "PID_CON.h"
#include "GTM_ATOM_PWM.h"
#include "OurCan.h"

#define tick_dis                0.219911486f                    //엔코더 한 틱당 거리 (mm)
#define circumference           197.9203372f                    //원 둘레(mm)
#define gear_ratio              18.75f                          //기어비


extern sint32 Enc_count;
extern sint32 s32_motor_speed_rpm;
extern sint32 s32_DisSum;
/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void initPeripherals(void);
void initSTM(void);

void init_move_distance_control(float32 tarDis, float32 tarVel);
TargetDistanceStatus move_distance(float32 tarDis);

#endif /* STM_INTERRUPT_H_ */
