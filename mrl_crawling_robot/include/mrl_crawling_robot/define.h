#ifndef DEFINE
#define DEFINE

#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cstdlib>
#include <cmath>


// System value
#define DEVICENAME "/dev/ttyUSB0"
#define loop_hz 100


// Dynamixel setting (Custom)
#define BAUDRATE 115200
#define BASE_MOTOR_B 0 
#define BASE_MOTOR_A 1
#define XC330_DXL_ID 2
#define EE_MOTOR_A 3   
#define EE_MOTOR_B 4 
  

// Common Dynamixel values
#define PPR 4096
#define PROTOCOL_VERSION 2.0
#define ADDR_TORQUE_ENABLE 64  
#define ADDR_GOAL_POSITION 116  
#define ADDR_PRESENT_POSITION 132  
#define ADDR_OPERATING_MODE 11  
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0


// control value
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

double base_B_ref_pos = 0;
double base_A_ref_pos = 0;
double elbow_ref_pos = 0;
double ee_A_ref_pos = 0;
double ee_B_ref_pos = 0;

double base_B_now_pos = 0;
double base_A_now_pos = 0;
double elbow_now_pos = 0;
double ee_A_now_pos = 0;
double ee_B_now_pos = 0;

#endif // DEFINE