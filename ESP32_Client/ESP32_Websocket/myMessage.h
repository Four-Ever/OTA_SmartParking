#ifndef __MY_MESSAGE_H__
#define __MY_MESSAGE_H__

#include <Arduino.h>
#include <pthread.h>

//ID DB바뀜에 따라 수정해야함
#define ID_ENGINE_MSG               0xC9
#define ID_MOVE_MSG                 0xCA
#define ID_AUTO_PARK_REQ_MSG        0xD1
#define ID_OTA_UDT_CFM_MSG          0xD9
#define ID_OFF_REQ_MSG              0xCB
#define ID_CGW_OTA_UDT_REQ_MSG      0x5A
#define ID_CGW_OTA_UDT_STATE_MSG    0x5B
#define ID_CGW_PRK_STATUS_MSG       0x52
#define ID_CGW_VHC_STATUS_MSG       0x42

#define MESSAGE_NUM 9

#define BUFFER_SIZE 32


extern pthread_mutex_t lock[MESSAGE_NUM]; 

typedef enum{
  LOCK_ENGINE,
  LOCK_MOVE,
  LOCK_APR,
  LOCK_OTA_UDT_REQ,
  LOCK_OFF_REQ,
  LOCK_CGW_OTA_UDT_REQ,
  LOCK_CGW_OTA_UDT_STATUS,
  LOCK_CGW_PRK_STATUS,
  LOCK_CGW_VHC_STATUS

}CTRL;

#pragma pack(1)



struct Nano_Engine
{
  uint8_t msgId;  // 메시지 ID (1바이트)
  struct
  {
    uint8_t control_engine:1;
  }signal;
};


struct Nano_Move
{
  uint8_t msgId;  // 메시지 ID (1바이트)
  struct
  {
    uint8_t control_accel:1;
    uint8_t control_brake:1;
    int8_t control_steering_angle:7;
    uint8_t control_transmission:2;
  }signal;
};

struct Nano_Auto_Parking_Request
{
  uint8_t msgId;  // 메시지 ID (1바이트)
  struct
  {
    uint8_t auto_parking:1;
  }signal;
};

struct Nano_OTA_Update_Confirm
{
  uint8_t msgId;  // 메시지 ID (1바이트)
  struct
  {
    uint8_t ota_confirm:1;
  }signal;
};

struct Nano_Off_Request 
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t alert_request:1;
      uint8_t auto_exit_request:1;
    }signal;
    
}; 

struct Nano_CGW_OTA_Update_Request 
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t ota_update_request:1;
    }signal;
    
}; 

struct Nano_CGW_OTA_Update_State
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t ota_update_progress:8;
    }signal;
    
}; 

struct Nano_CGW_Parking_Status_Msg 
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t parking_status:2;
    }signal;
    
}; 

struct Nano_CGW_Vehicle_Status_Msg
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t vehicle_velocity:7;
      int8_t vehicle_steering_angle:7;
      uint8_t vehicle_transmission:2;
    }signal;
    
}; 

struct Nano_Message
{
  Nano_Engine engine_msg;
  Nano_Move move_msg;
  Nano_Auto_Parking_Request auto_park_req_msg;
  Nano_OTA_Update_Confirm ota_udt_cfm_msg;
  Nano_Off_Request off_req_msg;
  Nano_CGW_OTA_Update_Request cgw_odt_udt_req_msg;
  Nano_CGW_OTA_Update_State cgw_odt_state_msg;
  Nano_CGW_Parking_Status_Msg cgw_park_status_msg;
  Nano_CGW_Vehicle_Status_Msg cgw_vhc_status_msg;
};

struct Nano_Flag
{
 uint8_t cgw_odt_udt_req_flag:1;
 uint8_t cgw_odt_state_flag:1;
 uint8_t cgw_park_status_flag:1;
 uint8_t cgw_vhc_status_flag:1;
};

extern Nano_Message nano_msg;
extern Nano_Flag nano_flag;




#pragma pack()  // 패킹 설정 해제

struct CTRL_Engine
{
  uint8_t msgId;  // 메시지 ID (1바이트)
  struct
  {
    uint8_t control_engine:1;
  }signal;
};


struct CTRL_Move
{
  uint8_t msgId;  // 메시지 ID (1바이트)
  struct
  {
    uint8_t control_accel:1;
    uint8_t control_brake:1;
    int8_t control_steering_angle:7;
    uint8_t control_transmission:2;
  }signal;
};

struct CTRL_Auto_Parking_Request
{
  uint8_t msgId;  // 메시지 ID (1바이트)
  struct
  {
    uint8_t auto_parking:1;
  }signal;
};

struct CTRL_OTA_Update_Confirm
{
  uint8_t msgId;  // 메시지 ID (1바이트)
  struct
  {
    uint8_t ota_confirm:1;
  }signal;
};

struct CTRL_Off_Request 
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t alert_request:1;
      uint8_t auto_exit_request:1;
    }signal;
    
}; 

struct CGW_OTA_Update_Request 
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t ota_update_request:1;
    }signal;
    
}; 

struct CGW_OTA_Update_State
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t ota_update_progress:8;
    }signal;
    
}; 

struct CGW_Parking_Status_Msg 
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t parking_status:2;
    }signal;
    
}; 

struct CGW_Vehicle_Status_Msg
{
    uint8_t msgId;  // 메시지 ID (1바이트)
    struct 
    {
      uint8_t vehicle_velocity:7;
      int8_t vehicle_steering_angle:7;
      uint8_t vehicle_transmission:2;
    }signal;
    
}; 

struct Message
{
  CTRL_Engine engine_msg;
  CTRL_Move move_msg;
  CTRL_Auto_Parking_Request auto_park_req_msg;
  CTRL_OTA_Update_Confirm ota_udt_cfm_msg;
  CTRL_Off_Request off_req_msg;
  CGW_OTA_Update_Request cgw_odt_udt_req_msg;
  CGW_OTA_Update_State cgw_odt_state_msg;
  CGW_Parking_Status_Msg cgw_park_status_msg;
  CGW_Vehicle_Status_Msg cgw_vhc_status_msg;
};

struct Flag
{
 uint8_t engine_flag:1;
 uint8_t move_flag:1;
 uint8_t auto_park_req_flag:1;
 uint8_t ota_udt_cfm_flag:1;
 uint8_t off_req_flag:1;
};

extern Message msg;
extern Flag flag;





#endif 