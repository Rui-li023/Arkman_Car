#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#define HEADER_SOF 0xA5
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#pragma pack(push, 1)
//ZXX
typedef enum
{
  CHASSIS_ODOM_CMD_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,
} referee_data_cmd_id_type;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef struct
{
  float vx=0;
  float vy=0;
  float vw=0;
  float relative_yaw=0;
  float voltage=0;
}  chassis_odom_info_t;



typedef struct
{
    float vx=0;
    float vy=0;
    float vw=0;
}robot_ctrl_info_t;


#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H
