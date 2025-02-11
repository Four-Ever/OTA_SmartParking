#ifndef __MY_UART_H__
#define __MY_UART_H__

#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "myMessage.h"
#include "myDataHandle.h"

// UART 번호 정의 (UART_NUM_1 사용)
#define UART_NUM UART_NUM_1
// 핀 번호 정의
#define TXD_PIN 17
#define RXD_PIN 16


void init_uart2(void);
// void uart_event_task(void *pvParameters);
void uart_event_task(void);


#endif 