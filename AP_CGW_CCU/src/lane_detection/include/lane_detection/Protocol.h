#pragma once

#include <cstdint>
#include <cstdlib>


// CANDB ID 정의
#define VCU_Vehicle_Status_ID 0x01 
#define VCU_Parking_Status_ID 0x12
#define VCU_Vehicle_Engine_Status_ID 0x02
#define VCU_ParkingLane_Request_ID 0x0C
#define VCU_Camera_ID 0x0B
#define VCU_Exiting_Status_ID 0x13
#define SCU_Obstacle_Detection_ID 0x28
#define CGW_OTA_File_Size_ID 0x5C
#define CGW_OTA_File_Data_ID 0x5D
#define CGW_OTA_Control_ID 0x59

#define CGW_OTA_Update_Request_ID 0x5A
#define CGW_OTA_Update_State_ID 0x5B
#define CGW_Vehicle_Status_ID 0x42
#define CGW_Parking_Status_ID 0x52
#define CGW_Exiting_Status_ID 0x53

#define CGW_Engine_ID 0x49
#define CGW_Move_ID 0x4A
#define CGW_Auto_Parking_Request_ID 0x51
#define CGW_Off_Request_ID 0x4B

//#define CTRL_OTA_Update_Confirm_ID 201

#define CCU_Cordi_data1_ID 0xA1
#define CCU_Cordi_data2_ID 0xA2
#define CCU_RightAngle_detect_ID 0xA0
#define CCU_ParkingAngle_detect_ID 0xA3
#define CCU_Parking_Complete_ID 0xA4
#define CTRL_Engine_ID 0xC9
#define CTRL_Move_ID 0xCA
#define CTRL_Auto_Parking_Request_ID 0xD1

#define CTRL_OTA_Update_Confirm_ID 0xD9

#define CTRL_Off_Request_ID 0xCB

// CANDB 크기 정의
#define VCU_Vehicle_Status_Size 2
#define VCU_Parking_Status_Size 1
#define VCU_Vehicle_Engine_Status_Size 1
#define VCU_ParkingLane_Request_Size 1
#define VCU_Camera_Size 1
#define VCU_Exiting_Status_Size 1
#define SCU_Obstacle_Detection_Size 1
#define CGW_OTA_File_Size_Size 4
#define CGW_OTA_File_Data_Size 8
#define CGW_OTA_Control_Size 8

#define CGW_OTA_Update_Request_Size 1
#define CGW_OTA_Update_State_Size 1
#define CGW_Vehicle_Status_Size 2
#define CGW_Parking_Status_Size 1
#define CGW_Exiting_Status_Size 1

#define CGW_Engine_Size 1
#define CGW_Move_Size 2
#define CGW_Auto_Parking_Request_Size 1
#define CGW_Off_Request_Size 1

#define CCU_Cordi_data1_Size 6
#define CCU_Cordi_data2_Size 6
#define CCU_RightAngle_detect_Size 1
#define CCU_ParkingAngle_detect_Size 1
#define CCU_Parking_Complete_Size 1
#define CTRL_Engine_Size 1
#define CTRL_Move_Size 2
#define CTRL_Auto_Parking_Request_Size 1

#define CTRL_OTA_Update_Confirm_Size 1

#define CTRL_Off_Request_Size 1



#pragma pack(push, 1)

struct BaseMsg {
    uint8_t msg_id;
};

// VCU TX Data Structures
struct VCU_Vehicle_Status_Data : public BaseMsg {
    struct {
        uint8_t vehicle_velocity : 7;
        int8_t vehicle_steering_angle : 7;
        uint8_t vehicle_transmission : 2;
    } data;
};

struct VCU_Parking_Status_Data : public BaseMsg {
    struct {
        uint8_t parking_status : 2;
    } data;
};

struct VCU_Vehicle_Engine_Status_Data : public BaseMsg {
    struct {
        uint8_t vehicle_engine_status : 1;
    } data;
};

struct VCU_ParkingLane_Request_Data : public BaseMsg {
    struct {
        uint8_t Lane_Request : 1;
    } data;
};

struct VCU_Camera_Data : public BaseMsg {
    struct {
        uint8_t camera_num : 2;
    } data;
};

struct VCU_Exiting_Status_Data : public BaseMsg{
    struct {
        uint8_t exiting_status : 1;
    } data;
};

struct SCU_Obstacle_Detection_Data : public BaseMsg {
    struct {
        uint8_t F_obstacle : 1;
        uint8_t B_obstacle : 1;
        uint8_t R_obstacle : 1;
        uint8_t L_obstacle : 1;
    } data;
};

// CGW TX Data Structures
struct CGW_OTA_File_Size_Data : public BaseMsg {
    struct {
        uint32_t ota_file_size;
    } data;
};

struct CGW_OTA_File_Data_Data : public BaseMsg {
    struct {
        uint64_t ota_file_data;
    } data;
};

struct CGW_OTA_Control_Data : public BaseMsg {
    struct {
        uint64_t ota_control;
    } data;
};

struct CGW_OTA_Update_Request_Data : public BaseMsg {
    struct {
        uint8_t ota_update_request : 1;
    } data;
};

struct CGW_OTA_Update_State_Data : public BaseMsg {
    struct {
        uint8_t ota_update_progress;
    } data;
};

struct CGW_Vehicle_Status_Data : public BaseMsg {
    struct {
        uint8_t vehicle_velocity : 7;
        int8_t vehicle_steering_angle : 7;
        uint8_t vehicle_transmission : 2;
    } data;
};

struct CGW_Parking_Status_Data : public BaseMsg {
    struct {
        uint8_t parking_status : 2;
    } data;
};

struct CGW_Exiting_Status_Data : public BaseMsg {
    struct {
        uint8_t exiting_status : 1;
    } data;
};

struct CGW_Engine_Data : public BaseMsg {
    struct {
        uint8_t control_engine : 1;
    } data;
};

struct CGW_Move_Data : public BaseMsg {
    struct {
        uint8_t control_accel : 1;
        uint8_t control_brake : 1;
        int8_t control_steering_angle : 7;
        uint8_t control_transmission : 2;
    } data;
};

struct CGW_Auto_Parking_Request_Data : public BaseMsg {
    struct {
        uint8_t auto_parking : 1;
    } data;
};

struct CGW_Off_Request_Data : public BaseMsg {
    struct {
        uint8_t alert_request : 1;
        uint8_t auto_exit_request : 1;
    } data;
};

struct CTRL_OTA_Update_Confirm_Data : public BaseMsg {
    struct {
        uint8_t ota_confirm : 1;
    } data;
};

struct CCU_Cordi_data1_Data : public BaseMsg {
    struct {
        int16_t cordi_data_y1 : 11;
        int16_t cordi_data_x1 : 10;
        int16_t cordi_data_y2 : 11;
        int16_t cordi_data_x2 : 10;
        //uint8_t using_camera : 2;
    } data;
};

struct CCU_Cordi_data2_Data : public BaseMsg {
    struct {
        int16_t cordi_data_y3 : 11;
        int16_t cordi_data_x3 : 10;
        int16_t cordi_data_y4 : 11;
        int16_t cordi_data_x4 : 10;
        uint8_t using_camera : 2;
        uint8_t trust_value : 6;
    } data;
};

struct CCU_RightAngle_detect_Data : public BaseMsg {
    struct {
        uint8_t right_angle_lane_detected : 1;
    } data;
};

struct CCU_ParkingAngle_detect_Data : public BaseMsg {
    struct {
        int8_t parking_back_lane_angle;
    } data;
};

struct CCU_Parking_Complete_Data : public BaseMsg {
    struct {
        uint8_t parking_back_lane_detected : 1;
    } data;
};

// CTRL TX Data Structures
struct CTRL_Engine_Data : public BaseMsg {
    struct {
        uint8_t control_engine : 1;
    } data;
};

struct CTRL_Move_Data : public BaseMsg {
    struct {
        uint8_t control_accel : 1;
        uint8_t control_brake : 1;
        int8_t control_steering_angle : 7;
        uint8_t control_transmission : 2;
    } data;
};

struct CTRL_Auto_Parking_Request_Data : public BaseMsg {
    struct {
        uint8_t auto_parking : 1;
    } data;
};

// struct CTRL_OTA_Update_Confirm_Data : public BaseMsg {
//     struct {
//         uint8_t ota_confirm : 1;
//     } data;
// };

struct CTRL_Off_Request_Data : public BaseMsg {
    struct {
        uint8_t alert_request : 1;
        uint8_t auto_exit_request : 1;
    } data;
};

#pragma pack(pop)

// Message Interface
class IMessage {
public:
    //IMessage(uint8_t msg_id) : msg_id_(msg_id) {}
    IMessage() = default;
    virtual ~IMessage() = default;
    virtual uint8_t* SerializeHttp()  = 0;
    virtual uint8_t* SerializeCan()  = 0;
    virtual size_t GetSizeHttp() const = 0;
    virtual size_t GetSizeCan() const = 0;
    virtual uint8_t GetMsgId() const = 0;
// public:
//     uint8_t msg_id_;
};
    
// VCU Messages
class VCU_Vehicle_Status_Msg : public IMessage {
public:
    VCU_Vehicle_Status_Msg() {
        data_.msg_id = VCU_Vehicle_Status_ID;
    }
    
    uint8_t* SerializeHttp()  override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan()  override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(VCU_Vehicle_Status_Data); }
    size_t GetSizeCan() const override { return sizeof(VCU_Vehicle_Status_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetVelocity(uint8_t velocity) { data_.data.vehicle_velocity = velocity; }
    void SetSteeringAngle(int8_t angle) { data_.data.vehicle_steering_angle = angle; }
    void SetTransmission(uint8_t transmission) { data_.data.vehicle_transmission = transmission; }

public:
    VCU_Vehicle_Status_Data data_;
};

class VCU_Parking_Status_Msg : public IMessage {
public:
    VCU_Parking_Status_Msg() {
        data_.msg_id = VCU_Parking_Status_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(VCU_Parking_Status_Data); }
    size_t GetSizeCan() const override { return sizeof(VCU_Parking_Status_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetParkingStatus(uint8_t status) { data_.data.parking_status = status; }

public:
    VCU_Parking_Status_Data data_;
};

class VCU_Vehicle_Engine_Status_Msg : public IMessage {
public:
    VCU_Vehicle_Engine_Status_Msg() {
        data_.msg_id = VCU_Vehicle_Engine_Status_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(VCU_Vehicle_Engine_Status_Data); }
    size_t GetSizeCan() const override { return sizeof(VCU_Vehicle_Engine_Status_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetEngineStatus(uint8_t status) { data_.data.vehicle_engine_status = status; }

private:
    VCU_Vehicle_Engine_Status_Data data_;
};

class VCU_ParkingLane_Request_Msg : public IMessage {
public:
    VCU_ParkingLane_Request_Msg() {
        data_.msg_id = VCU_ParkingLane_Request_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(VCU_ParkingLane_Request_Data); }
    size_t GetSizeCan() const override { return sizeof(VCU_ParkingLane_Request_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetLaneRequest(uint8_t request) { data_.data.Lane_Request = request; }

public:
    VCU_ParkingLane_Request_Data data_;
};

class VCU_Camera_Msg : public IMessage {
public:
    VCU_Camera_Msg() {
        data_.msg_id = VCU_Camera_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(VCU_Camera_Data); }
    size_t GetSizeCan() const override { return sizeof(VCU_Camera_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetCameraNum(uint8_t num) { data_.data.camera_num = num; }

public:
    VCU_Camera_Data data_;
};

class VCU_Exiting_Status_Msg : public IMessage {
public:
    VCU_Exiting_Status_Msg() {
        data_.msg_id = VCU_Exiting_Status_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(VCU_Exiting_Status_Data); }
    size_t GetSizeCan() const override { return sizeof(VCU_Exiting_Status_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetExitingStatus(uint8_t status) { data_.data.exiting_status = status; }

public:
    VCU_Exiting_Status_Data data_;
};

class SCU_Obstacle_Detection_Msg : public IMessage {
public:
    SCU_Obstacle_Detection_Msg() {
        data_.msg_id = SCU_Obstacle_Detection_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(SCU_Obstacle_Detection_Data); }
    size_t GetSizeCan() const override { return sizeof(SCU_Obstacle_Detection_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetFObstacle(uint8_t obstacle) { data_.data.F_obstacle = obstacle; }
    void SetBObstacle(uint8_t obstacle) { data_.data.B_obstacle = obstacle; }
    void SetRObstacle(uint8_t obstacle) { data_.data.R_obstacle = obstacle; }
    void SetLObstacle(uint8_t obstacle) { data_.data.L_obstacle = obstacle; }

public:
    SCU_Obstacle_Detection_Data data_;
};

// CGW Messages
class CGW_OTA_File_Size_Msg : public IMessage {
public:
    CGW_OTA_File_Size_Msg() {
        data_.msg_id = CGW_OTA_File_Size_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_OTA_File_Size_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_OTA_File_Size_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetOtaFileSize(uint32_t size) { data_.data.ota_file_size = size; }

public:
    CGW_OTA_File_Size_Data data_;
};

class CGW_OTA_File_Data_Msg : public IMessage {
public:
    CGW_OTA_File_Data_Msg() {
        data_.msg_id = CGW_OTA_File_Data_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_OTA_File_Data_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_OTA_File_Data_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetOtaFileData(uint64_t data) { data_.data.ota_file_data = data; }

public:
    CGW_OTA_File_Data_Data data_;
};

class CGW_OTA_Control_Msg : public IMessage {
public:
    CGW_OTA_Control_Msg() {
        data_.msg_id = CGW_OTA_Control_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_OTA_Control_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_OTA_Control_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetOtaControl(uint64_t control) { data_.data.ota_control = control; }

public:
    CGW_OTA_Control_Data data_;
};

class CGW_OTA_Update_Request_Msg : public IMessage {
public:
    CGW_OTA_Update_Request_Msg() {
        data_.msg_id = CGW_OTA_Update_Request_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_OTA_Update_Request_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_OTA_Update_Request_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetOtaUpdateRequest(uint8_t request) { data_.data.ota_update_request = request; }

public:
    CGW_OTA_Update_Request_Data data_;
};

class CGW_OTA_Update_State_Msg : public IMessage {
public:
    CGW_OTA_Update_State_Msg() {
        data_.msg_id = CGW_OTA_Update_State_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_OTA_Update_State_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_OTA_Update_State_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetOtaUpdateProgress(uint8_t progress) { data_.data.ota_update_progress = progress; }

public:
    CGW_OTA_Update_State_Data data_;
};

class CGW_Vehicle_Status_Msg : public IMessage {
public:
    CGW_Vehicle_Status_Msg() {
        data_.msg_id = CGW_Vehicle_Status_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_Vehicle_Status_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_Vehicle_Status_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetVelocity(uint8_t velocity) { data_.data.vehicle_velocity = velocity; }
    void SetSteeringAngle(int8_t angle) { data_.data.vehicle_steering_angle = angle; }
    void SetTransmission(uint8_t transmission) { data_.data.vehicle_transmission = transmission; }

public:
    CGW_Vehicle_Status_Data data_;
};

class CGW_Parking_Status_Msg : public IMessage {
public:
    CGW_Parking_Status_Msg() {
        data_.msg_id = CGW_Parking_Status_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_Parking_Status_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_Parking_Status_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetParkingStatus(uint8_t status) { data_.data.parking_status = status; }

public:
    CGW_Parking_Status_Data data_;
};

class CGW_Exiting_Status_Msg : public IMessage {
public:
    CGW_Exiting_Status_Msg() {
        data_.msg_id = CGW_Exiting_Status_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_Exiting_Status_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_Exiting_Status_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetExitingStatus(uint8_t status) { data_.data.exiting_status = status; }

public:
    CGW_Exiting_Status_Data data_;
};

class CGW_Engine_Msg : public IMessage {
public:
    CGW_Engine_Msg() {
        data_.msg_id = CGW_Engine_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_Engine_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_Engine_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetControlEngine(uint8_t control) { data_.data.control_engine = control; }

public:
    CGW_Engine_Data data_;
};

class CGW_Move_Msg : public IMessage {
public:
    CGW_Move_Msg() {
        data_.msg_id = CGW_Move_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_Move_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_Move_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetControlAccel(uint8_t accel) { data_.data.control_accel = accel; }
    void SetControlBrake(uint8_t brake) { data_.data.control_brake = brake; }
    void SetControlSteeringAngle(int8_t angle) { data_.data.control_steering_angle = angle; }
    void SetControlTransmission(uint8_t transmission) { data_.data.control_transmission = transmission; }

public:
    CGW_Move_Data data_;
};

class CGW_Auto_Parking_Request_Msg : public IMessage {
public:
    CGW_Auto_Parking_Request_Msg() {
        data_.msg_id = CGW_Auto_Parking_Request_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_Auto_Parking_Request_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_Auto_Parking_Request_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetAutoParking(uint8_t parking) { data_.data.auto_parking = parking; }

public:
    CGW_Auto_Parking_Request_Data data_;
};



class CGW_Off_Request_Msg : public IMessage {
public:
    CGW_Off_Request_Msg() {
        data_.msg_id = CGW_Off_Request_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CGW_Off_Request_Data); }
    size_t GetSizeCan() const override { return sizeof(CGW_Off_Request_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetAlertRequest(uint8_t alert) { data_.data.alert_request = alert; }
    void SetAutoExitRequest(uint8_t exit) { data_.data.auto_exit_request = exit; }

public:
    CGW_Off_Request_Data data_;
};

// class CTRL_OTA_Update_Confirm_Msg : public IMessage {
// public:
//     CTRL_OTA_Update_Confirm_Msg() {
//         data_.msg_id = CTRL_OTA_Update_Confirm_ID;
//     }
//     uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
//     uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
//     size_t GetSizeHttp() const override { return sizeof(CCU_Cordi_data1_Data); }
//     size_t GetSizeCan() const override { return sizeof(CCU_Cordi_data1_Data) - sizeof(BaseMsg); }
//     uint8_t GetMsgId() const override { return data_.msg_id; }

//     void SetOtaConfirm(uint8_t otaConfirm) {data_.data.ota_confirm = otaConfirm; }

// public:
//     CTRL_OTA_Update_Confirm_Data data_;
// };

class CCU_Cordi_data1_Msg : public IMessage {
public:
    CCU_Cordi_data1_Msg() {
        data_.msg_id = CCU_Cordi_data1_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CCU_Cordi_data1_Data); }
    size_t GetSizeCan() const override { return sizeof(CCU_Cordi_data1_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetCordiY1(int16_t y) { data_.data.cordi_data_y1 = y; }
    void SetCordiX1(int16_t x) { data_.data.cordi_data_x1 = x; }
    void SetCordiY2(int16_t y) { data_.data.cordi_data_y2 = y; }
    void SetCordiX2(int16_t x) { data_.data.cordi_data_x2 = x; }
    //void SetUsingCamera(uint8_t camera) { data_.data.using_camera = camera; }

public:
    CCU_Cordi_data1_Data data_;
};

class CCU_Cordi_data2_Msg : public IMessage {
public:
    CCU_Cordi_data2_Msg() {
        data_.msg_id = CCU_Cordi_data2_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CCU_Cordi_data2_Data); }
    size_t GetSizeCan() const override { return sizeof(CCU_Cordi_data2_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetCordiY3(int16_t y) { data_.data.cordi_data_y3 = y; }
    void SetCordiX3(int16_t x) { data_.data.cordi_data_x3 = x; }
    void SetCordiY4(int16_t y) { data_.data.cordi_data_y4 = y; }
    void SetCordiX4(int16_t x) { data_.data.cordi_data_x4 = x; }
    void SetUsingCamera(uint8_t camera) { data_.data.using_camera = camera; }
    void SetTrustValue(uint8_t value) { data_.data.trust_value = value; };

public:
    CCU_Cordi_data2_Data data_;
};

class CCU_RightAngle_detect_Msg : public IMessage {
public:
    CCU_RightAngle_detect_Msg() {
        data_.msg_id = CCU_RightAngle_detect_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CCU_RightAngle_detect_Data); }
    size_t GetSizeCan() const override { return sizeof(CCU_RightAngle_detect_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetRightAngleLaneDetected(uint8_t detected) { data_.data.right_angle_lane_detected = detected; }

public:
    CCU_RightAngle_detect_Data data_;
};

class CCU_ParkingAngle_detect_Msg : public IMessage {
public:
    CCU_ParkingAngle_detect_Msg() {
        data_.msg_id = CCU_ParkingAngle_detect_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CCU_ParkingAngle_detect_Data); }
    size_t GetSizeCan() const override { return sizeof(CCU_ParkingAngle_detect_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetParkingBackLaneAngle(int8_t angle) { data_.data.parking_back_lane_angle = angle; }

public:
    CCU_ParkingAngle_detect_Data data_;
};

class CCU_Parking_Complete_Msg : public IMessage {
public:
    CCU_Parking_Complete_Msg() {
        data_.msg_id = CCU_Parking_Complete_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CCU_Parking_Complete_Data); }
    size_t GetSizeCan() const override { return sizeof(CCU_Parking_Complete_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetParkingBackLaneDetected(uint8_t detected) { data_.data.parking_back_lane_detected = detected; }

public:
    CCU_Parking_Complete_Data data_;
};


// CTRL Messages
class CTRL_Engine_Msg : public IMessage {
public:
    CTRL_Engine_Msg() {
        data_.msg_id = CTRL_Engine_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CTRL_Engine_Data); }
    size_t GetSizeCan() const override { return sizeof(CTRL_Engine_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetControlEngine(uint8_t control) { data_.data.control_engine = control; }

public:
    CTRL_Engine_Data data_;
};

class CTRL_Move_Msg : public IMessage {
public:
    CTRL_Move_Msg() {
        data_.msg_id = CTRL_Move_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CTRL_Move_Data); }
    size_t GetSizeCan() const override { return sizeof(CTRL_Move_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetControlAccel(uint8_t accel) { data_.data.control_accel = accel; }
    void SetControlBrake(uint8_t brake) { data_.data.control_brake = brake; }
    void SetControlSteeringAngle(int8_t angle) { data_.data.control_steering_angle = angle; }
    void SetControlTransmission(uint8_t transmission) { data_.data.control_transmission = transmission; }

public:
    CTRL_Move_Data data_;
};

class CTRL_Auto_Parking_Request_Msg : public IMessage {
public:
    CTRL_Auto_Parking_Request_Msg() {
        data_.msg_id = CTRL_Auto_Parking_Request_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CTRL_Auto_Parking_Request_Data); }
    size_t GetSizeCan() const override { return sizeof(CTRL_Auto_Parking_Request_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetAutoParking(uint8_t parking) { data_.data.auto_parking = parking; }

public:
    CTRL_Auto_Parking_Request_Data data_;
};

class CTRL_OTA_Update_Confirm_Msg : public IMessage {
public:
    CTRL_OTA_Update_Confirm_Msg() {
        data_.msg_id = CTRL_OTA_Update_Confirm_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CTRL_OTA_Update_Confirm_Data); }
    size_t GetSizeCan() const override { return sizeof(CTRL_OTA_Update_Confirm_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetOtaConfirm(uint8_t confirm) { data_.data.ota_confirm = confirm; }

public:
    CTRL_OTA_Update_Confirm_Data data_;
};

class CTRL_Off_Request_Msg : public IMessage {
public:
    CTRL_Off_Request_Msg() {
        data_.msg_id = CTRL_Off_Request_ID;
    }
    
    uint8_t* SerializeHttp() override { return reinterpret_cast<uint8_t*>(&data_); }
    uint8_t* SerializeCan() override { return reinterpret_cast<uint8_t*>(&data_.data); }
    size_t GetSizeHttp() const override { return sizeof(CTRL_Off_Request_Data); }
    size_t GetSizeCan() const override { return sizeof(CTRL_Off_Request_Data) - sizeof(BaseMsg); }
    uint8_t GetMsgId() const override { return data_.msg_id; }
    
    void SetAlertRequest(uint8_t alert) { data_.data.alert_request = alert; }
    void SetAutoExitRequest(uint8_t exit) { data_.data.auto_exit_request = exit; }

public:
    CTRL_Off_Request_Data data_;
};


