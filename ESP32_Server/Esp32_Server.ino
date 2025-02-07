#include <WiFi.h>
#include <WebSocketsServer.h>


// Wi-Fi 설정
const char* ssid = "ESP32_Fourever";       // ESP32의 Wi-Fi SSID
const char* password = "1q2w3e4r";  // ESP32의 Wi-Fi 비밀번호




// WebSocket 서버 포트
WebSocketsServer webSocket(81);





// WebSocket 이벤트 처리 함수
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  // if (type == WStype_TEXT) {
  //   // JSON 메시지 파싱 준비
    
  // }
}

void setup() {
  Serial.begin(115200);

  // Wi-Fi SoftAP 모드 시작
  WiFi.softAP(ssid, password, 6, 0, 4);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // 최대 출력 설정
  Serial.print("SoftAP IP 주소: ");
  Serial.println(WiFi.softAPIP());

  // WebSocket 서버 시작
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket 서버가 시작되었습니다.");




}

void loop() {
  // WebSocket 클라이언트 관리
  webSocket.loop();


}