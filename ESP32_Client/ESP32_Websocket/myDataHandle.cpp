#include <stdint.h>
#include "myDataHandle.h"
#include "myMessage.h"
#include "myWebsocket.h"
#include "myUART.h"
#include "driver/uart.h"
#include <string.h>

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

    case ID_OTA_UDT_CFM_MSG:
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

    case ID_OFF_REQ_MSG:
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

void read_datas_from_Nano(uint8_t * rx_buffer, size_t length){
  // length -> signal + 1 이 들어옴
  // dlc = sizeof(msg.move_msg.signal);
  // TC275와 동일한 형태
  uint8_t sig_dlc = length-1; //sig dlc 1~8
  uint8_t id = rx_buffer[0]; // id

  #ifdef DUBUG_NANO_RECEIVE
  for(int i=0;i<32;i++)
  {
    Serial.printf("%d - 0X%02X\t",i,rx_buffer[i]);
    if((i+1)%8 == 0)
    {
      Serial.println();
    }
  }
  #endif
  // delay(1000);
  switch (id)
  {
    case ID_CGW_OTA_UDT_REQ_MSG :
    {  
      #ifdef LOCKING
      pthread_mutex_lock(&lock[LOCK_CGW_OTA_UDT_REQ]);
      #endif
      byteArrayToStruct(&rx_buffer[0],&nano_msg.cgw_odt_udt_req_msg,length);
      #ifdef LOCKING
      pthread_mutex_unlock(&lock[LOCK_CGW_OTA_UDT_REQ]);
      #endif

      nano_flag.cgw_odt_udt_req_flag = 1;
      #ifdef DEBUG_PRINT
      Serial.printf("[WSC Received] MsgID: 0X%02X, ota_update_request:  %u  \n", 
                    nano_msg.cgw_odt_udt_req_msg, nano_msg.cgw_odt_udt_req_msg.signal.ota_update_request);
      Serial.println();
      #endif 
      break;
    }  
    case ID_CGW_OTA_UDT_STATE_MSG:
    {  
        #ifdef LOCKING
        pthread_mutex_lock(&lock[LOCK_CGW_OTA_UDT_STATUS]);
        #endif
        byteArrayToStruct(&rx_buffer[0], &nano_msg.cgw_odt_state_msg, length);
        #ifdef LOCKING
        pthread_mutex_unlock(&lock[LOCK_CGW_OTA_UDT_STATUS]);
        #endif

        nano_flag.cgw_odt_state_flag = 1;
        #ifdef DEBUG_PRINT
        Serial.printf("[WSC Received] MsgID: 0X%02X, ota_update_progress: %u\n", 
                      nano_msg.cgw_odt_state_msg.msgId, nano_msg.cgw_odt_state_msg.signal.ota_update_progress);
        Serial.println();
        #endif 
        break;
    }

    case ID_CGW_PRK_STATUS_MSG:
    {  
        #ifdef LOCKING
        pthread_mutex_lock(&lock[LOCK_CGW_PRK_STATUS]);
        #endif
        byteArrayToStruct(&rx_buffer[0], &nano_msg.cgw_park_status_msg, length);
        #ifdef LOCKING
        pthread_mutex_unlock(&lock[LOCK_CGW_PRK_STATUS]);
        #endif

        nano_flag.cgw_park_status_flag = 1;
        #ifdef DEBUG_PRINT
        Serial.printf("[WSC Received] MsgID: 0X%02X, parking_status: %u\n", 
                      nano_msg.cgw_park_status_msg.msgId, nano_msg.cgw_park_status_msg.signal.parking_status);
        Serial.println();
        #endif 
        break;
    }

    case ID_CGW_EXIT_STATUS_MSG:
    {  
        #ifdef LOCKING
        pthread_mutex_lock(&lock[LOCK_CGW_EXIT_STATUS_MSG]);
        #endif
        byteArrayToStruct(&rx_buffer[0], &nano_msg.cgw_exit_status_msg, length);
        #ifdef LOCKING
        pthread_mutex_unlock(&lock[LOCK_CGW_EXIT_STATUS_MSG]);
        #endif

        nano_flag.cgw_exit_status_flag = 1;
        #ifdef DEBUG_PRINT
        Serial.printf("[WSC Received] MsgID: 0X%02X, exiting_status: %u\n", 
                      nano_msg.cgw_exit_status_msg.msgId, nano_msg.cgw_exit_status_msg.signal.exiting_status);
        Serial.println();
        #endif 
        break;
    }

    case ID_CGW_VHC_STATUS_MSG:
    {  
        #ifdef LOCKING
        pthread_mutex_lock(&lock[LOCK_CGW_VHC_STATUS]);
        #endif
        byteArrayToStruct(&rx_buffer[0], &nano_msg.cgw_vhc_status_msg, length);
        #ifdef LOCKING
        pthread_mutex_unlock(&lock[LOCK_CGW_VHC_STATUS]);
        #endif

        nano_flag.cgw_vhc_status_flag = 1;
        #ifdef DEBUG_PRINT
        Serial.printf("[WSC Received] MsgID: 0X%02X, vehicle_velocity: %u, vehicle_steering_angle: %d, vehicle_transmission: %u\n", 
                      nano_msg.cgw_vhc_status_msg.msgId, 
                      nano_msg.cgw_vhc_status_msg.signal.vehicle_velocity, 
                      nano_msg.cgw_vhc_status_msg.signal.vehicle_steering_angle, 
                      nano_msg.cgw_vhc_status_msg.signal.vehicle_transmission);
        Serial.println();
        #endif 
        break;
    }

  }
}
void sent_uart_well(void * msg, size_t size)
{
  uint8_t rx_buffer[32] ={0};
  structToByteArray(msg, rx_buffer, size);
  rx_buffer[size] = 0xFF;
  uart_write_bytes(UART_NUM, &rx_buffer, size+1);
}

void dummy_send_datas_to_TC275(){
  msg.cgw_odt_udt_req_msg.msgId = ID_CGW_OTA_UDT_REQ_MSG;
  msg.cgw_odt_udt_req_msg.signal.ota_update_request = 1;

  sent_uart_well(&msg.cgw_odt_udt_req_msg,sizeof(msg.cgw_odt_udt_req_msg));

  #ifdef SENT_DEBUG_PRINT
  Serial.printf("[Sent to TC275] - MsgID: 0x%02X, ota_update_request : %u \n", 
            msg.cgw_odt_udt_req_msg.msgId,
            msg.cgw_odt_udt_req_msg.signal.ota_update_request

          );
  #endif
  delay(1);
  static uint8_t i =0;
  msg.cgw_odt_state_msg.msgId = ID_CGW_OTA_UDT_STATE_MSG;
  msg.cgw_odt_state_msg.signal.ota_update_progress = (i++)%101;

  sent_uart_well(&msg.cgw_odt_state_msg,sizeof(msg.cgw_odt_state_msg));

  #ifdef SENT_DEBUG_PRINT
  Serial.printf("[Sent to TC275] - MsgID: 0x%02X, ota_update_progress : %u \n", 
            msg.cgw_odt_state_msg.msgId,
            msg.cgw_odt_state_msg.signal.ota_update_progress

          );
  #endif
delay(1);
  msg.cgw_park_status_msg.msgId = ID_CGW_PRK_STATUS_MSG;
  msg.cgw_park_status_msg.signal.parking_status = 3;

  sent_uart_well(&msg.cgw_park_status_msg,sizeof(msg.cgw_park_status_msg));

  #ifdef SENT_DEBUG_PRINT
  Serial.printf("[Sent to TC275] - MsgID: 0x%02X, parking_status : %u \n", 
            msg.cgw_park_status_msg.msgId,
            msg.cgw_park_status_msg.signal.parking_status

          );
  #endif
delay(1);

  msg.cgw_exit_status_msg.msgId = ID_CGW_EXIT_STATUS_MSG;
  msg.cgw_exit_status_msg.signal.exiting_status = (i<50)?0:1;

  sent_uart_well(&msg.cgw_exit_status_msg,sizeof(msg.cgw_exit_status_msg));

  #ifdef SENT_DEBUG_PRINT
  Serial.printf("[Sent to TC275] - MsgID: 0x%02X, exiting_status : %u \n", 
            msg.cgw_exit_status_msg.msgId,
            msg.cgw_exit_status_msg.signal.exiting_status

          );
  #endif
delay(1);

 static int speed =10;
  msg.cgw_vhc_status_msg.msgId = ID_CGW_VHC_STATUS_MSG;
  msg.cgw_vhc_status_msg.signal.vehicle_velocity = speed++;
  msg.cgw_vhc_status_msg.signal.vehicle_steering_angle = -44;
  msg.cgw_vhc_status_msg.signal.vehicle_transmission = 3;

  sent_uart_well(&msg.cgw_vhc_status_msg,sizeof(msg.cgw_vhc_status_msg));

  #ifdef SENT_DEBUG_PRINT
  Serial.printf("[Sent to TC275] - MsgID: 0x%02X, vehicle_velocity : %u, vehicle_steering_angle:  %d, vehicle_transmission %u \n", 
            msg.cgw_vhc_status_msg.msgId,
            msg.cgw_vhc_status_msg.signal.vehicle_velocity,
            msg.cgw_vhc_status_msg.signal.vehicle_steering_angle,
            msg.cgw_vhc_status_msg.signal.vehicle_transmission

            );
  #endif
delay(1);


}

void send_datas_to_TC275(){
  if(nano_flag.cgw_odt_udt_req_flag == 1)
  {
    nano_flag.cgw_odt_udt_req_flag = 0;
    #ifdef LOCKING
    #endif

    msg.cgw_odt_udt_req_msg.msgId = nano_msg.cgw_odt_udt_req_msg.msgId;
    msg.cgw_odt_udt_req_msg.signal.ota_update_request = nano_msg.cgw_odt_udt_req_msg.signal.ota_update_request;

    #ifdef LOCKING
    #endif

    sent_uart_well(&msg.cgw_odt_udt_req_msg,sizeof(msg.cgw_odt_udt_req_msg));


  }
  if(nano_flag.cgw_odt_state_flag == 1)
  {
    nano_flag.cgw_odt_state_flag = 0;
    #ifdef LOCKING
    #endif

    msg.cgw_odt_state_msg.msgId = nano_msg.cgw_odt_state_msg.msgId;
    msg.cgw_odt_state_msg.signal.ota_update_progress = nano_msg.cgw_odt_state_msg.signal.ota_update_progress;

    #ifdef LOCKING
    #endif

    sent_uart_well(&msg.cgw_odt_state_msg,sizeof(msg.cgw_odt_state_msg));



  }

  if(nano_flag.cgw_park_status_flag == 1)
  {
    nano_flag.cgw_park_status_flag = 0;
    #ifdef LOCKING
    #endif

    msg.cgw_park_status_msg.msgId = nano_msg.cgw_park_status_msg.msgId;
    msg.cgw_park_status_msg.signal.parking_status = nano_msg.cgw_park_status_msg.signal.parking_status;

    #ifdef LOCKING
    #endif
    sent_uart_well(&msg.cgw_park_status_msg,sizeof(msg.cgw_park_status_msg));


  }

  if(nano_flag.cgw_exit_status_flag == 1)
  {
    nano_flag.cgw_exit_status_flag = 0;
    #ifdef LOCKING
    #endif

    msg.cgw_exit_status_msg.msgId = nano_msg.cgw_exit_status_msg.msgId;
    msg.cgw_exit_status_msg.signal.exiting_status = nano_msg.cgw_exit_status_msg.signal.exiting_status;

    #ifdef LOCKING
    #endif
    sent_uart_well(&msg.cgw_exit_status_msg,sizeof(msg.cgw_exit_status_msg));


  }

  if(nano_flag.cgw_vhc_status_flag == 1)
  {
    nano_flag.cgw_vhc_status_flag = 0;
    #ifdef LOCKING
    #endif

    msg.cgw_vhc_status_msg.msgId = nano_msg.cgw_vhc_status_msg.msgId;
    msg.cgw_vhc_status_msg.signal.vehicle_steering_angle = nano_msg.cgw_vhc_status_msg.signal.vehicle_steering_angle;
    msg.cgw_vhc_status_msg.signal.vehicle_transmission = nano_msg.cgw_vhc_status_msg.signal.vehicle_transmission;
    msg.cgw_vhc_status_msg.signal.vehicle_velocity = nano_msg.cgw_vhc_status_msg.signal.vehicle_velocity;

    #ifdef LOCKING
    #endif

    sent_uart_well(&msg.cgw_vhc_status_msg,sizeof(msg.cgw_vhc_status_msg));

    // uint8_t rx_buffer[32] ={0};

    // structToByteArray(&msg.cgw_vhc_status_msg, rx_buffer, sizeof(msg.cgw_vhc_status_msg));
    // rx_buffer[sizeof(msg.cgw_vhc_status_msg)] = 0xFF;
    // uart_write_bytes(UART_NUM, &rx_buffer, sizeof(msg.cgw_vhc_status_msg)+1);
    
    #ifdef SENT_DEBUG_PRINT
    Serial.printf("[Sent to TC275] - MsgID: 0x%02X, vehicle_steering_angle:  %d, vehicle_transmission %u, vehicle_velocity : %u\n", 
              msg.cgw_vhc_status_msg.msgId,
              msg.cgw_vhc_status_msg.signal.vehicle_steering_angle,
              msg.cgw_vhc_status_msg.signal.vehicle_transmission,
              msg.cgw_vhc_status_msg.signal.vehicle_velocity
              );  

    #endif
  }
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
    Serial.printf("[Sent to Nano] - MsgID: 0x%02X, control_engine:  %u  \n", 
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
    Serial.printf("[Sent to Nano] - MsgID: 0x%02X, control_accel: %u, control_brake : %u, control_steering_angle : %d, control_transmission : %u nano-size: %u tc-size : %u   \n", 
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
    Serial.printf("[Sent to Nano] - MsgID: 0x%02X, auto_parking: %u, \n", 
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
    Serial.printf("[Sent to Nano] - MsgID: 0x%02X, ota_confirm: %u,   \n", 
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
    Serial.printf("[Sent to Nano] - MsgID: 0x%02X, alert_request: %u, auto_exit_request : %u, \n", 
              nano_msg.off_req_msg.msgId, nano_msg.off_req_msg.signal.alert_request, nano_msg.off_req_msg.signal.auto_exit_request);
    #endif
  }
}