
#include <WebSocketsClient.h>
#include <Arduino.h>
#include <string.h>

#include "myWebsocket.h"
#include "myMessage.h"
#include "myUART.h"
#include "myDataHandle.h"

// uint8_t tx_buffer[BUFFER_SIZE] = {0xAA, 0xBB, 0xCC, 0xDD}; // 송신 데이터 (예제)

void setup() {
  Serial.begin(115200);
  // SPI연결
  // init_spi();

  //UART2 연결 (With Tc275)
  init_uart2();

  // WiFi 연결 (With Jetson Nano)
  init_wifi();

  for(int i=0;i<MESSAGE_NUM;i++)
  {
    pthread_mutex_init(&lock[i], NULL);  //  뮤텍스 초기화
  }

}

void loop() {
  webSocket.loop(); // 웹소켓 루프 실행

  uart_event_task();
  if (webSocket.isConnected()) {
      send_datas_to_Nano();
      // uart_event_task();
  }
}

