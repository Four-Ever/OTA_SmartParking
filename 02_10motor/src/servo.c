/**********************************************************************************************************************
 * \file servo.c
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
#include "servo.h"
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
#define ISR_PRIORITY_ATOM  20                                   /* Interrupt priority number                        */
#define SERVO              IfxGtm_ATOM0_4_TOUT4_P02_4_OUT     /* SERVO which will be driven by the PWM            */
#define PWM_PERIOD         5000                                 /* PWM period for the ATOM                          */
#define CLK_FREQ           1000000.0f                           /* CMU clock frequency, in Hertz                    */
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
IfxGtm_Atom_Pwm_Config g_atomConfig_servo;
IfxGtm_Atom_Pwm_Driver g_atomDriver_servo;
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
void IfxGtm_Servo_Atom_Pwm_initConfig(IfxGtm_Atom_Pwm_Config *config, Ifx_GTM *gtm)
{
    config->gtm                      = gtm;
    config->atom                     = SERVO.atom;
    config->atomChannel              = SERVO.channel;
    config->period                   = 20000;
    config->dutyCycle                = 1500;
    config->signalLevel              = Ifx_ActiveState_high;//Ifx_ActiveState_high;
    config->mode                     = IfxGtm_Atom_Mode_outputPwm;
    config->oneShotModeEnabled       = FALSE;
    config->synchronousUpdateEnabled = FALSE;
    config->immediateStartEnabled    = TRUE;
    config->interrupt.ccu0Enabled    = FALSE;
    config->interrupt.ccu1Enabled    = FALSE;
    config->interrupt.mode           = IfxGtm_IrqMode_pulseNotify;
    config->interrupt.isrProvider    = IfxSrc_Tos_cpu0;
    config->interrupt.isrPriority    = 0;
    config->pin.outputPin            = NULL_PTR;
    config->pin.outputMode           = IfxPort_OutputMode_pushPull;
    config->pin.padDriver            = IfxPort_PadDriver_cmosAutomotiveSpeed1;
}


void initServo(void)
{
    IfxGtm_enable(&MODULE_GTM); /* Enable GTM */
    // PWM 二쇳뙆�닔 �꽕�젙 (50Hz = 20ms)
    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_1, CLK_FREQ);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK1);                /* Enable the CMU clock 0           */
    IfxGtm_Servo_Atom_Pwm_initConfig(&g_atomConfig_servo, &MODULE_GTM);                     /* Initialize default parameters    */

    g_atomConfig_servo.atom = SERVO.atom;                                       /* Select the ATOM depending on the LED     */
    g_atomConfig_servo.atomChannel = SERVO.channel;                             /* Select the channel depending on the LED  */
    g_atomConfig_servo.period = 20000;//(unsigned int)(CLK_FREQ / 50);                                   /* Set timer period*/
    g_atomConfig_servo.dutyCycle = 1500;
    g_atomConfig_servo.pin.outputPin = &SERVO;                                  /* Set LED as output                        */
    g_atomConfig_servo.synchronousUpdateEnabled = TRUE;                       /* Enable synchronous update                */

    IfxGtm_Atom_Pwm_init(&g_atomDriver_servo, &g_atomConfig_servo);                 /* Initialize the PWM                       */
    IfxGtm_Atom_Pwm_start(&g_atomDriver_servo, TRUE);                         /* Start the PWM                            */
}

//void IfxGtm_Servo_Tom_Pwm_initConfig(IfxGtm_Tom_Pwm_Config *config, Ifx_GTM *gtm)
//{
//    config->gtm                      = gtm;
//    config->tom                     = SERVO.tom;
//    config->tomChannel              = SERVO.channel;
//    config->clock                   = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0;
//    config->period                   = 20000;
//    config->dutyCycle                = 1500;
//    config->signalLevel              = Ifx_ActiveState_high;
//    config->oneShotModeEnabled       = FALSE;
//    config->synchronousUpdateEnabled = FALSE;
//    config->interrupt.ccu0Enabled    = FALSE;
//    config->interrupt.ccu1Enabled    = FALSE;
//    config->interrupt.mode           = IfxGtm_IrqMode_pulseNotify;
//    config->interrupt.isrProvider    = IfxSrc_Tos_cpu0;
//    config->interrupt.isrPriority    = 0;
//    config->pin.outputPin            = NULL_PTR;
//    config->pin.outputMode           = IfxPort_OutputMode_pushPull;
//    config->pin.padDriver            = IfxPort_PadDriver_cmosAutomotiveSpeed1;
//}
//
//
//void initServo(void)
//{
//    IfxGtm_enable(&MODULE_GTM); /* Enable GTM */
//    // PWM 二쇳뙆�닔 �꽕�젙 (50Hz = 20ms)
//    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CLK_FREQ);
//    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);                /* Enable the CMU clock 0           */
//    //IfxGtm_Tom_Pwm_initConfig(&g_atomConfig_servo);
//    IfxGtm_Servo_Tom_Pwm_initConfig(&g_tomConfig_servo, &MODULE_GTM);                     /* Initialize default parameters    */
//
//    g_tomConfig_servo.tom = SERVO.tom;                                       /* Select the ATOM depending on the LED     */
//    g_tomConfig_servo.tomChannel = SERVO.channel;                             /* Select the channel depending on the LED  */
//    g_tomConfig_servo.period = 20000;//(unsigned int)(CLK_FREQ / 50);                                   /* Set timer period*/
//    g_tomConfig_servo.dutyCycle = 1500;
//    g_tomConfig_servo.pin.outputPin = &SERVO;                                  /* Set LED as output                        */
//    g_tomConfig_servo.synchronousUpdateEnabled = TRUE;                       /* Enable synchronous update                */
//
//    IfxGtm_Tom_Pwm_init(&g_tomDriver_servo, &g_tomConfig_servo);                 /* Initialize the PWM                       */
//    IfxGtm_Tom_Pwm_start(&g_tomDriver_servo, TRUE);                         /* Start the PWM                            */
//}

void setServoAngle(sint8 angle)
{
    // �꽌蹂� 以묎컙 �쐞移� (unsigned int)(CLK_FREQ / 50 * 0.075);

    if (angle < -MAX_ANGLE)
    {
        angle = -MAX_ANGLE;
    }
    else if (angle > MAX_ANGLE)
    {
        angle = MAX_ANGLE;
    }

    float pulseWidth;
    if (angle >= 0)
    {
        pulseWidth = SERVO_CENTER_MS + (angle / MAX_ANGLE) * (RIGHT_SERVO_RANGE_MS / 2.0f);
    }

    else
    {
        pulseWidth = SERVO_CENTER_MS + (angle / MAX_ANGLE) * (LEFT_SERVO_RANGE_MS / 2.0f);
    }

    float dutyCycle = pulseWidth / SERVO_PERIOD_MS;
    //g_atomConfig_servo.dutyCycle = (unsigned int)(CLK_FREQ / 50 * dutyCycle);                 /* Set duty cycle        */
    g_atomConfig_servo.dutyCycle = (unsigned int)(CLK_FREQ / 50 * dutyCycle);
    //g_atomConfig_servo.dutyCycle = (unsigned int)(CLK_FREQ / 50 * dutyCycle);                 /* Set duty cycle        */

    //g_atomConfig_servo.dutyCycle = dutyCycle;                 /* Set duty cycle        */
    IfxGtm_Atom_Pwm_init(&g_atomDriver_servo, &g_atomConfig_servo); /* Re-initialize the PWM */
}
/*********************************************************************************************************************/
