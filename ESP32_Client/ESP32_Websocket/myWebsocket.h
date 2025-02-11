#ifndef __MY_WEBSOCKET_H__
#define __MY_WEBSOCKET_H__

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>

#define PORT 8080


extern WebSocketsClient webSocket;

void init_wifi();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);

#endif 