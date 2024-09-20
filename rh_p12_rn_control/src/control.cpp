#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cstdlib>
#include "std_msgs/msg/bool.hpp"

#define PROTOCOL_VERSION 2.0
#define DXL_ID 0
#define BAUDRATE 115200
#define DEVICENAME "/dev/ttyUSB0"

#define ADDR_TORQUE_ENABLE 512
#define ADDR_GOAL_POSITION 564
#define ADDR_PRESENT_POSITION 580
#define ADDR_OPERATING_MODE 11
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define goal_position_open 200
#define goal_position_grip 740
#define loop_hz 100

bool grip = false;

void grip_callback(const std_msgs::msg::Bool::SharedPtr msg) 
{
    grip = msg->data;
}

int main(int argc, char **argv)
{
    system(("sudo chmod 666 " + std::string(DEVICENAME)).c_str());

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rh_p12_rh_node");

    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);

    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);

    auto subscription = node->create_subscription<std_msgs::msg::Bool>("grip_command", 10, grip_callback);

    uint32_t present_position = 0;
    
    rclcpp::WallRate loop_rate(loop_hz);

    while (rclcpp::ok()) 
    {
        if (grip) packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position_grip, nullptr);
        else packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position_open, nullptr);

        packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, &present_position, nullptr);

        RCLCPP_INFO(node->get_logger(), "Goal position: %d, Present position: %d", grip ? goal_position_grip : goal_position_open, present_position);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
    portHandler->closePort();
    rclcpp::shutdown();
    return 0;
}
