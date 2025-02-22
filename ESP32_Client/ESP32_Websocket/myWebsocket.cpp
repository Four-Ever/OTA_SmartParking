#include <WiFi.h>
#include "myWebsocket.h"
#include "myDataHandle.h"

// const char* ssid = "AndroidAPc752";
// const char* password = "123321999";

const char* ssid = "rapa_meetingroom-1";
const char* password = "rapa6074";

// const char* ssid = "wonphil";
// const char* password = "1q2w3e4r";

// WiFi 설정
// const char* ssid = "ESP32_Fourever";
// const char* password = "1q2w3e4r";

const char* webSocketServer = "192.168.201.8"; // 예: ws://192.168.1.100:8080
WebSocketsClient webSocket;

void init_wifi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // 웹소켓 초기화 및 연결
  webSocket.begin(webSocketServer,PORT); // 서버 주소 설정
  webSocket.onEvent(webSocketEvent); // 이벤트 핸들러 등록

}

// 웹소켓 이벤트 핸들러

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      // Serial.printf("[WSc] Disconnected!\n");
      webSocket.begin(webSocketServer, PORT); // 웹소켓 재연결
      break;
    case WStype_CONNECTED:
      Serial.printf("[WSc] Connected to URL: %s\n", payload);
      // 서버에 메시지 전송
      // webSocket.sendTXT("Hello from ESP32!");
      break;
    case WStype_TEXT:
      // Serial.printf("[WSc] Received text: %s\n", payload);
      //서버에서 메시지 수신
      read_datas_from_Nano(payload,length);
      break;
    case WStype_ERROR:
      Serial.printf("[WSc] Error occurred\n");
      break;

  }
}
