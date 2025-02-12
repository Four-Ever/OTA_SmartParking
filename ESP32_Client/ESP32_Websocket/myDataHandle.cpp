#include "myDataHandle.h"

// #define DEBUG_PRINT
// #define SENT_DEBUG_PRINT
// #define LOCKING

//  구조체 → 바이트 배열 변환 함수
void structToByteArray(void *certain_msg, uint8_t *buffer, size_t size) {
    memcpy(buffer, certain_msg, size);
}

// 바이트 배열 → 구조체 복원 함수
void byteArrayToStruct(uint8_t *buffer, void *certain_msg, size_t size) {
    memcpy(certain_msg, buffer, size);
}


void read_datas_from_TC275(uint8_t * rx_buffer)
{
  uint8_t sig_dlc = rx_buffer[0]; //sig dlc 1~8
  uint8_t id = rx_buffer[1]; // id
  // for(int i=0;i<32;i++)
  // {
  //   Serial.printf("%d - 0X%02X\t",i,rx_buffer[i]);
  //   if((i+1)%8 == 0)
  //   {
  //     Serial.println();
  //   }
  // }
  // delay(1000);
  switch (id)
  {
    case ID_ENGINE_MSG:
    {
      #ifdef LOCKING
      pthread_mutex_lock(&lock[LOCK_ENGINE]);
      #endif
      byteArrayToStruct(&rx_buffer[1],&msg.engine_msg,sig_dlc+1);
      #ifdef LOCKING
      pthread_mutex_unlock(&lock[LOCK_ENGINE]);
      #endif

      flag.engine_flag = 1;
      #ifdef DEBUG_PRINT
      Serial.printf("[UART_RECEIVE] MsgID: 0X%02X, control_engine:  %u  \n", 
                    msg.engine_msg.msgId, msg.engine_msg.signal.control_engine);
      Serial.println();
      #endif 
      break;
    }
    case ID_MOVE_MSG: // Move_Msg 
    {
      #ifdef LOCKING      
      pthread_mutex_lock(&lock[LOCK_MOVE]);
      #endif
      byteArrayToStruct(&rx_buffer[1],&msg.move_msg,sig_dlc+1);
      #ifdef LOCKING
      pthread_mutex_unlock(&lock[LOCK_MOVE]);
      #endif
      flag.move_flag = 1;

      #ifdef DEBUG_PRINT
      Serial.printf("[UART_RECEIVE] MsgID - 0X%02X control_accel: %u, control_brake : %u, control_steering_angle : %d, control_transmission : %u", 
                      msg.move_msg.msgId, msg.move_msg.signal.control_accel, msg.move_msg.signal.control_brake, 
                      msg.move_msg.signal.control_steering_angle, msg.move_msg.signal.control_transmission);
      Serial.println();
      #endif
      // Serial.printf(" Next msg.move_msg.signal.control_transmission : %u \n", 
      break;
    }

    case ID_AUTO_PARK_REQ_MSG:
    {

      #ifdef LOCKING
      pthread_mutex_lock(&lock[LOCK_APR]);
      #endif
      byteArrayToStruct(&rx_buffer[1],&msg.auto_park_req_msg,sig_dlc+1);
      #ifdef LOCKING
      pthread_mutex_unlock(&lock[LOCK_APR]);
      #endif
      flag.auto_park_req_flag = 1;

      #ifdef DEBUG_PRINT
      Serial.printf("[UART_RECEIVE] MsgID: 0X%02X, auto_parking: %u, \n", 
                    msg.auto_park_req_msg.msgId, msg.auto_park_req_msg.signal.auto_parking);
      Serial.println();
      #endif
      break;
    }

    case ID_OTA_UDT_CFM:
    {
      #ifdef LOCKING
      pthread_mutex_lock(&lock[LOCK_OTA_UDT_REQ]);
      #endif
      byteArrayToStruct(&rx_buffer[1],&msg.ota_udt_cfm_msg,sig_dlc+1);
      #ifdef LOCKING
      pthread_mutex_unlock(&lock[LOCK_OTA_UDT_REQ]);
      #endif
      flag.ota_udt_cfm_flag = 1;

      #ifdef DEBUG_PRINT
      Serial.printf("[UART_RECEIVE] MsgID: 0X%02X, ota_confirm: %u   \n", 
                      msg.ota_udt_cfm_msg.msgId, msg.ota_udt_cfm_msg.signal.ota_confirm);
      Serial.println();
      #endif
      break;
    }

    case ID_OFF_REQ:
    {
      #ifdef LOCKING
      pthread_mutex_lock(&lock[LOCK_OFF_REQ]);
      #endif
      byteArrayToStruct(&rx_buffer[1],&msg.off_req_msg,sig_dlc+1);
      #ifdef LOCKING
      pthread_mutex_unlock(&lock[LOCK_OFF_REQ]);
      #endif
      flag.off_req_flag = 1;
      
      #ifdef DEBUG_PRINT
      Serial.printf("[UART_RECEIVE] MsgID: 0X%02X, alert_request: %u, auto_exit_request : %u \n", 
                     msg.off_req_msg.msgId, msg.off_req_msg.signal.alert_request, msg.off_req_msg.signal.auto_exit_request);
      Serial.println();
      #endif
      break;
    }
    
  }
}

void send_datas_to_TC275(){
  // dlc = sizeof(msg.move_msg.signal);
  // TC275와 동일한 형태
  // send ()
}

void send_datas_to_Nano()
{
  if(flag.engine_flag == 1)
  {
    flag.engine_flag = 0;
    #ifdef LOCKING
    pthread_mutex_lock(&lock[LOCK_ENGINE]);
    #endif

    nano_msg.engine_msg.msgId = msg.engine_msg.msgId;
    nano_msg.engine_msg.signal.control_engine = msg.engine_msg.signal.control_engine;

    #ifdef LOCKING
    pthread_mutex_unlock(&lock[LOCK_ENGINE]);
    #endif
    
    webSocket.sendBIN((uint8_t*)&nano_msg.engine_msg, sizeof(nano_msg.engine_msg));
    #ifdef SENT_DEBUG_PRINT
    Serial.printf("Sent - MsgID: 0x%02X, control_engine:  %u  \n", 
              nano_msg.engine_msg.msgId, nano_msg.engine_msg.signal.control_engine);  
    #endif
  }

  if(flag.move_flag == 1)
  {
    flag.move_flag = 0;
    #ifdef LOCKING
    pthread_mutex_lock(&lock[LOCK_MOVE]);
    #endif

    nano_msg.move_msg.msgId = msg.move_msg.msgId;
    nano_msg.move_msg.signal.control_accel = msg.move_msg.signal.control_accel;
    nano_msg.move_msg.signal.control_brake = msg.move_msg.signal.control_brake;
    nano_msg.move_msg.signal.control_steering_angle = msg.move_msg.signal.control_steering_angle;
    nano_msg.move_msg.signal.control_transmission = msg.move_msg.signal.control_transmission;
    
    #ifdef LOCKING
    pthread_mutex_unlock(&lock[LOCK_MOVE]);
    #endif
    
    webSocket.sendBIN((uint8_t*)&nano_msg.move_msg, sizeof(nano_msg.move_msg));
    #ifdef SENT_DEBUG_PRINT
    Serial.printf("Sent - MsgID: 0x%02X, control_accel: %u, control_brake : %u, control_steering_angle : %d, control_transmission : %u nano-size: %u tc-size : %u   \n", 
              nano_msg.move_msg.msgId, nano_msg.move_msg.signal.control_accel, nano_msg.move_msg.signal.control_brake,
              nano_msg.move_msg.signal.control_steering_angle, nano_msg.move_msg.signal.control_transmission,sizeof(nano_msg.move_msg),sizeof(msg.move_msg)); 
    #endif
  }

  if(flag.auto_park_req_flag == 1)
  {
    flag.auto_park_req_flag = 0;
    #ifdef LOCKING
    pthread_mutex_lock(&lock[LOCK_APR]);
    #endif

    nano_msg.auto_park_req_msg.msgId = msg.auto_park_req_msg.msgId;
    nano_msg.auto_park_req_msg.signal.auto_parking = msg.auto_park_req_msg.signal.auto_parking;
    
    #ifdef LOCKING
    pthread_mutex_unlock(&lock[LOCK_APR]);
    #endif

    
    webSocket.sendBIN((uint8_t*)&nano_msg.auto_park_req_msg, sizeof(nano_msg.auto_park_req_msg));
    #ifdef SENT_DEBUG_PRINT
    Serial.printf("Sent - MsgID: 0x%02X, auto_parking: %u, \n", 
              nano_msg.auto_park_req_msg.msgId, nano_msg.auto_park_req_msg.signal.auto_parking);
    #endif  
  }

  if(flag.ota_udt_cfm_flag == 1)
  {
    flag.ota_udt_cfm_flag = 0;
    #ifdef LOCKING
    pthread_mutex_lock(&lock[LOCK_OTA_UDT_REQ]);
    #endif

    nano_msg.ota_udt_cfm_msg.msgId = msg.ota_udt_cfm_msg.msgId;
    nano_msg.ota_udt_cfm_msg.signal.ota_confirm = msg.ota_udt_cfm_msg.signal.ota_confirm;
    
    #ifdef LOCKING
    pthread_mutex_unlock(&lock[LOCK_OTA_UDT_REQ]);
    #endif
    
    webSocket.sendBIN((uint8_t*)&nano_msg.ota_udt_cfm_msg, sizeof(nano_msg.ota_udt_cfm_msg));
    #ifdef SENT_DEBUG_PRINT
    Serial.printf("Sent - MsgID: 0x%02X, ota_confirm: %u,   \n", 
              nano_msg.ota_udt_cfm_msg.msgId, nano_msg.ota_udt_cfm_msg.signal.ota_confirm); 
    #endif 
  }
  if(flag.off_req_flag == 1)
  {
    flag.off_req_flag = 0;
    #ifdef LOCKING
    pthread_mutex_lock(&lock[LOCK_OFF_REQ]);
    #endif

    nano_msg.off_req_msg.msgId = msg.off_req_msg.msgId;
    nano_msg.off_req_msg.signal.alert_request = msg.off_req_msg.signal.alert_request;
    nano_msg.off_req_msg.signal.auto_exit_request = msg.off_req_msg.signal.auto_exit_request;

    
    #ifdef LOCKING
    pthread_mutex_unlock(&lock[LOCK_OFF_REQ]);
    #endif
    
    webSocket.sendBIN((uint8_t*)&nano_msg.off_req_msg, sizeof(nano_msg.off_req_msg));

    #ifdef SENT_DEBUG_PRINT
    Serial.printf("Sent - MsgID: 0x%02X, alert_request: %u, auto_exit_request : %u, \n", 
              nano_msg.off_req_msg.msgId, nano_msg.off_req_msg.signal.alert_request, nano_msg.off_req_msg.signal.auto_exit_request);
    #endif
  }
}