
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Arduino.h>
#include <SPI.h>
#include "driver/spi_slave.h"

#define PORT 8080

// WiFi 설정
// const char* ssid = "ESP32_Fourever";
// const char* password = "1q2w3e4r";
const char* ssid = "wonphil";
const char* password = "1q2w3e4r";

const char* webSocketServer = "192.168.137.103"; // 예: ws://192.168.1.100:8080
WebSocketsClient webSocket;

struct DataPacket {
    unsigned char msgId;  // 메시지 ID (1바이트)
    struct {
        int value1;      // 4바이트
        float value2;    // 4바이트
    } data;
} __attribute__((packed));  // ESP32에서 패딩 제거


void init_wipi();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);

#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCLK 18
#define PIN_SS   5  // Slave Select (CS)

#define BUFFER_SIZE 32

uint8_t rx_buffer[BUFFER_SIZE] = {0}; // 수신 데이터 버퍼
uint8_t tx_buffer[BUFFER_SIZE] = {0xAA, 0xBB, 0xCC, 0xDD}; // 송신 데이터 (예제)

// #define SPI_CLK   1000000 // 1 MHz
#define SPI_INTERVAL 10  // 10ms


void my_post_setup_cb(spi_slave_transaction_t *trans);
void my_post_trans_cb(spi_slave_transaction_t *trans);
void spi_task(void *pvParameters);

void setup() {
  Serial.begin(115200);
  // SPI연결
  init_spi();
  // WiFi 연결
  init_wifi();



}


void loop() {
  webSocket.loop(); // 웹소켓 루프 실행

  static unsigned long lastTime = 0;
  static unsigned char msgCount = 0;
  
  if (millis() - lastTime > 1) {  // 1초마다 전송
      lastTime = millis();
      
      if (webSocket.isConnected()) {
          // 테스트 데이터 패킷 생성
          DataPacket packet;
          packet.msgId = msgCount++;          // 메시지 ID 증가
          packet.data.value1 = random(100);   // 테스트 값
          packet.data.value2 = random(100) / 10.0f;

          // 데이터 전송
          webSocket.sendBIN((uint8_t*)&packet, sizeof(DataPacket));
          
          // 전송한 데이터 출력
          Serial.printf("Sent - MsgID: 0x%02X, Value1: %d, Value2: %.1f\n", 
                      packet.msgId, packet.data.value1, packet.data.value2);
      }
  }
  // if (millis() - lastSendTime >= SPI_INTERVAL) {
  // // 주기적으로 SPI 데이터 송신
  //   lastSPISendTime = millis();

  //   // SPI 송신
  //   spiCommand(vspi);

  //   // txBuf1을 초기화 (0으로 채움)
  //   txBuf1 = 0;
  
  // }
}

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
      webSocket.sendTXT("Hello from ESP32!");
      break;
    case WStype_TEXT:
      Serial.printf("[WSc] Received text: %s\n", payload);
      //서버에서 메시지 수신
      // receive_send_spi(payload,length);
      break;
    case WStype_ERROR:
      Serial.printf("[WSc] Error occurred\n");
      break;

  }
}

void my_post_trans_cb(spi_slave_transaction_t *trans) {
    Serial.print("[SPI] Received Data: ");
    for (int i = 0; i < BUFFER_SIZE; i++) {
        Serial.printf("%02X ", rx_buffer[i]);
    }
    Serial.println();
}

void spi_task(void *pvParameters) {
    while (1) {
        spi_slave_transaction_t trans;
        trans.length = 8 * 32;  // 32 바이트
        trans.tx_buffer = tx_buffer;
        trans.rx_buffer = rx_buffer;

        if (spi_slave_transmit(VSPI_HOST, &trans, portMAX_DELAY) == ESP_OK) {
            // Serial.println("SPI Transaction Done");
        }
    }
}

void init_spi(){
  spi_bus_config_t buscfg = {
    .mosi_io_num = PIN_MOSI,
    .miso_io_num = PIN_MISO,
    .sclk_io_num = PIN_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1
  };

  spi_slave_interface_config_t slvcfg = {
    .spics_io_num = PIN_SS,
    .flags = 0,
    .queue_size = 3,
    .mode = 0, 
    .post_setup_cb = NULL,
    .post_trans_cb = my_post_trans_cb,
    // .dma_chan = 2 
  };

  // SPI Slave 초기화
  spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
  xTaskCreatePinnedToCore(spi_task, "spi_task", 4096, NULL, 1, NULL, 1);
  Serial.println("SPI Slave Initialized");

}
