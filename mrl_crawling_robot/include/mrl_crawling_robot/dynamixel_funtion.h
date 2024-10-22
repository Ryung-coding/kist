#ifndef DYNAMIXEL_FUNTION
#define DYNAMIXEL_FUNTION


void CONNECT_dynamixel()
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);  
    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);
}



void SET_dynamixel()
{
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Set up XC330 motor
    packetHandler->write1ByteTxRx(portHandler, XC330_DXL_ID, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, XC330_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);

    // Set up 2XC430-W250-T motors (base and end-effector with two IDs each)
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_B, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_B, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);

    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_A, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_A, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);

    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_A, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_A, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);

    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_B, ADDR_OPERATING_MODE, 3, nullptr);
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_B, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, nullptr);
}



void KILL_dynamixel()
{
    // Disable torque for all motors
    packetHandler->write1ByteTxRx(portHandler, XC330_DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_B, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
    packetHandler->write1ByteTxRx(portHandler, BASE_MOTOR_A, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_A, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
    packetHandler->write1ByteTxRx(portHandler, EE_MOTOR_B, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);

    portHandler->closePort();
}



    // 2XC430-W250-T position control (for base)
void BASE_position_control(double base_B_ref_pos, double base_A_ref_pos) 
{
    uint32_t base_B_now_pos_step = 0;
    uint32_t base_A_now_pos_step = 0;
    packetHandler->read4ByteTxRx(portHandler, BASE_MOTOR_B, ADDR_PRESENT_POSITION, &base_B_now_pos_step, nullptr); //output unit = step
    packetHandler->read4ByteTxRx(portHandler, BASE_MOTOR_A, ADDR_PRESENT_POSITION, &base_A_now_pos_step, nullptr); //output unit = step
    base_B_now_pos = static_cast<double>(base_B_now_pos_step) * (2 * M_PI) / PPR;  // step => rad
    base_A_now_pos = static_cast<double>(base_A_now_pos_step) * (2 * M_PI) / PPR;  // step => rad

    uint32_t base_B_ref_pos_step = static_cast<uint32_t>(base_B_ref_pos * PPR / (2 * M_PI));  // rad => step
    uint32_t base_A_ref_pos_step = static_cast<uint32_t>(base_A_ref_pos * PPR / (2 * M_PI));  // rad => step
    packetHandler->write4ByteTxRx(portHandler, BASE_MOTOR_B, ADDR_GOAL_POSITION, base_B_ref_pos_step, nullptr); //input unit = step
    packetHandler->write4ByteTxRx(portHandler, BASE_MOTOR_A, ADDR_GOAL_POSITION, base_A_ref_pos_step, nullptr); //input unit = step
}



    // XC330 position control
void ELBOW_position_control(double elbow_ref_pos) 
{   
    uint32_t elbow_now_pos_step = 0;
    packetHandler->read4ByteTxRx(portHandler, XC330_DXL_ID, ADDR_PRESENT_POSITION, &elbow_now_pos_step, nullptr); //output unit = step
    elbow_now_pos = static_cast<double>(elbow_now_pos_step) * (2 * M_PI) / PPR;  // step => rad


    uint32_t elbow_ref_pos_step = static_cast<uint32_t>(elbow_ref_pos * PPR / (2 * M_PI));  // rad => step
    packetHandler->write4ByteTxRx(portHandler, XC330_DXL_ID, ADDR_GOAL_POSITION, elbow_ref_pos_step, nullptr); //input unit = step
}



    // 2XC430-W250-T position control (for end-effector)
void EE_position_control(double ee_A_ref_pos, double ee_B_ref_pos) 
{
    uint32_t ee_B_now_pos_step = 0;
    uint32_t ee_A_now_pos_step = 0;
    packetHandler->read4ByteTxRx(portHandler, EE_MOTOR_A, ADDR_PRESENT_POSITION, &ee_A_now_pos_step, nullptr); //output unit = step
    packetHandler->read4ByteTxRx(portHandler, EE_MOTOR_B, ADDR_PRESENT_POSITION, &ee_B_now_pos_step, nullptr); //output unit = step
    ee_A_now_pos = static_cast<double>(ee_A_now_pos_step) * (2 * M_PI) / PPR;  // step => rad
    ee_B_now_pos = static_cast<double>(ee_B_now_pos_step) * (2 * M_PI) / PPR;  // step => rad

    uint32_t ee_A_ref_pos_step = static_cast<uint32_t>(ee_A_ref_pos * PPR / (2 * M_PI));  // rad => step
    uint32_t ee_B_ref_pos_step = static_cast<uint32_t>(ee_B_ref_pos * PPR / (2 * M_PI));  // rad => step
    packetHandler->write4ByteTxRx(portHandler, EE_MOTOR_A, ADDR_GOAL_POSITION, ee_A_ref_pos_step, nullptr); //input unit = step
    packetHandler->write4ByteTxRx(portHandler, EE_MOTOR_B, ADDR_GOAL_POSITION, ee_B_ref_pos_step, nullptr); //input unit = step   
}




#endif // DYNAMIXEL_FUNTION