/**********************************************************************************************************************
 * \file ASCLIN_UART.c
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
/*********************************************************************************************************************/
#include "ASCLIN_UART.h"
#include "IfxCpu_Irq.h"
#include <string.h>
#include "Message.h"
#include "Data_process.h"
#include "Common_def.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define UART_BAUDRATE           921600                                  /* UART baud rate in bit/s                  */

#define UART_PIN_RX             IfxAsclin0_RXB_P15_3_IN                 /* UART receive port pin                    */
#define UART_PIN_TX             IfxAsclin0_TX_P15_2_OUT                 /* UART transmit port pin                   */

/* Definition of the interrupt priorities */
#define INTPRIO_ASCLIN0_RX      20
#define INTPRIO_ASCLIN0_TX      19

#define UART_RX_BUFFER_SIZE     1024                                      /* Definition of the receive buffer size    */
#define UART_TX_BUFFER_SIZE     1024                                      /* Definition of the transmit buffer size   */
#define SIZE                    32                                      /* Size of the string                       */



/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
/* Declaration of the ASC handle */
static IfxAsclin_Asc g_ascHandle;

/* Declaration of the FIFOs parameters */
uint8 g_ascTxBuffer[UART_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
uint8 g_ascRxBuffer[UART_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

/* Definition of txData and rxData */
uint8 g_txData[] = "Hello World!";
uint8 g_rxData[SIZE] = {''};
/* Size of the message */
uint8 g_count = 0;
Ifx_SizeT leng = 32;
DataReceivedFlag receive_flag = RECEIVE_WAIT;

uint8 g_rcv_size;

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
/* Adding of the interrupt service routines */
IFX_INTERRUPT(asclin0TxISR, 1, INTPRIO_ASCLIN0_TX);
void asclin0TxISR(void)
{
    IfxAsclin_Asc_isrTransmit(&g_ascHandle);
}

IFX_INTERRUPT(asclin0RxISR, 1, INTPRIO_ASCLIN0_RX);
void asclin0RxISR(void)
{
//    myprintf("I'm in RxISR\n\r");
    IfxAsclin_Asc_isrReceive(&g_ascHandle);


    g_count += IfxAsclin_Asc_read(&g_ascHandle, g_rxData + g_count, &leng, 0);
    if (g_rxData[g_count - 1] == 0xFF)  // 종료 문자 감지
    {

        g_rcv_size = g_count - 1;
        g_count = 0;
        receive_flag = RECEIVE_COMPLETED;
    }

}

/* This function initializes the ASCLIN UART module */
void init_ASCLIN_UART(void)
{
    /* Initialize an instance of IfxAsclin_Asc_Config with default values */
    IfxAsclin_Asc_Config ascConfig;
    IfxAsclin_Asc_initModuleConfig(&ascConfig, &MODULE_ASCLIN0);

    /* Set the desired baud rate */
    ascConfig.baudrate.baudrate = UART_BAUDRATE;

    /* ISR priorities and interrupt target */
    ascConfig.interrupt.txPriority = INTPRIO_ASCLIN0_TX;
    ascConfig.interrupt.rxPriority = INTPRIO_ASCLIN0_RX;
    ascConfig.interrupt.typeOfService = IfxSrc_Tos_cpu1;//IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());

    /* FIFO configuration */
    ascConfig.txBuffer = &g_ascTxBuffer;
    ascConfig.txBufferSize = UART_TX_BUFFER_SIZE;
    ascConfig.rxBuffer = &g_ascRxBuffer;
    ascConfig.rxBufferSize = UART_RX_BUFFER_SIZE;

    /* Pin configuration */
    const IfxAsclin_Asc_Pins pins =
    {
        NULL_PTR,       IfxPort_InputMode_pullUp,     /* CTS pin not used */
        &UART_PIN_RX,   IfxPort_InputMode_pullUp,     /* RX pin           */
        NULL_PTR,       IfxPort_OutputMode_pushPull,  /* RTS pin not used */
        &UART_PIN_TX,   IfxPort_OutputMode_pushPull,  /* TX pin           */
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConfig.pins = &pins;

    IfxAsclin_Asc_initModule(&g_ascHandle, &ascConfig); /* Initialize module with above parameters */
}

/* This function sends and receives the string "Hello World!" */

void Send_Message(uint8 *tx_Data, Ifx_SizeT size)
{

    IfxAsclin_Asc_write(&g_ascHandle, tx_Data, &size, TIME_INFINITE);
}


