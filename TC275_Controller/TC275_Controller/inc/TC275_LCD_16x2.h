/**********************************************************************************************************************
 * \file TC275_LCD_16x2.h
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

#ifndef TC275_LCD_16X2_H_
#define TC275_LCD_16X2_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "Ifx_Types.h"

//#include "IfxCpu.h"
#include "Common_def.h"


/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/

#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

//***** List of commands Bitfields *****//
// 1) Entry mode Bitfields
#define LCD_ENTRY_SH 0x01
#define LCD_ENTRY_ID 0x02
// 2) Entry mode Bitfields
#define LCD_ENTRY_SH 0x01
#define LCD_ENTRY_ID 0x02
// 3) Display control
#define LCD_DISPLAY_B 0x01
#define LCD_DISPLAY_C 0x02
#define LCD_DISPLAY_D 0x04
// 4) Shift control
#define LCD_SHIFT_RL 0x04
#define LCD_SHIFT_SC 0x08
// 5) Function set control
#define LCD_FUNCTION_F 0x04
#define LCD_FUNCTION_N 0x08
#define LCD_FUNCTION_DL 0x10


/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/

/*********************************************************************************************************************/
 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/

// Public functions

// 1) LCD begin 8 bits function
void LCD1602_Begin8BIT(Ifx_P *PORT_RS_E, uint8 RS, uint8 E, Ifx_P *PORT_LSBs0to3, uint8 D0, uint8 D1, uint8 D2, uint8 D3, Ifx_P *PORT_MSBs4to7, uint8 D4, uint8 D5, uint8 D6, uint8 D7);
// 2) LCD begin 4 bits function
void LCD1602_Begin4BIT(Ifx_P *PORT_all, uint8 RS, uint8 E, Ifx_P *_PORT_D4, uint8 D4, uint8 D5, uint8 D6, uint8 D7);
// 3) LCD print string
void LCD1602_print( char string[]);
// 4) set cursor position
void LCD1602_setCursor(uint8 row, uint8 col);
void LCD1602_1stLine(void);
void LCD1602_2ndLine(void);
// 5) Enable two lines
void LCD1602_TwoLines(void);
void LCD1602_OneLine(void);
// 6) Cursor ON/OFF
void LCD1602_noCursor(void);
void LCD1602_cursor(void);
// 7) Clear display
void LCD1602_clear(void);
// 8) Blinking cursor
void LCD1602_noBlink(void);
void LCD1602_blink(void);
// 9) Display ON/OFF
void LCD1602_noDisplay(void);
void LCD1602_display(void);
// 10) Shift Display, right or left
void LCD1602_shiftToRight(uint8 num);
void LCD1602_shiftToLeft(uint8 num);

//********** Print numbers to LCD **********//
// 1. Integer
void LCD1602_PrintInt(int number);
// 2. Float
void LCD1602_PrintFloat(float number, int decimalPoints);

void LCD1602_set_CGRAM(uint8 address, uint8 *data);
void store_custom_img();
void LCD1602_print_picture(Picture p);

void init_LCD();
void init_GPIO_LCD();
void LCD1602_print_percent_img(int bat_val);
void LCD1602_loading();
void LCD1602_loading_nodelay();



#endif
