#ifndef __MY_DATA_HANDLE_H__
#define __MY_DATA_HANDLE_H__

#include <Arduino.h>
#include <string.h>
#include "myMessage.h"
#include "myWebsocket.h"


void read_datas_from_TC275(uint8_t * rx_buffer);

void send_datas_to_Nano();


void byteArrayToStruct(uint8_t *buffer, void *certain_msg, size_t size);
void structToByteArray(void *certain_msg, uint8_t *buffer, size_t size);



#endif 