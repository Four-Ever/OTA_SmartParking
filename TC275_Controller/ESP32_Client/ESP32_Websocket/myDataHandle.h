#ifndef __MY_DATA_HANDLE_H__
#define __MY_DATA_HANDLE_H__

#include <Arduino.h>


// #define DEBUG_PRINT
// #define SENT_DEBUG_PRINT
// #define LOCKING
// #define DUBUG_NANO_RECEIVE

void read_datas_from_TC275(uint8_t * rx_buffer);
void send_datas_to_TC275();

void read_datas_from_Nano(uint8_t * payload, size_t length);
void send_datas_to_Nano();


void byteArrayToStruct(uint8_t *buffer, void *certain_msg, size_t size);
void structToByteArray(void *certain_msg, uint8_t *buffer, size_t size);

void dummy_send_datas_to_TC275();

#endif 