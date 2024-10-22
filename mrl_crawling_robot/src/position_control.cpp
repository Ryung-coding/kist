#include "define.h"
#include "dynamixel_funtion.h"


int main(int argc, char **argv)
{
        //Port setting
    system(("sudo chmod 666 " + std::string(DEVICENAME)).c_str());

        //ROS2 setting
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("position_control");
    rclcpp::WallRate loop_rate(loop_hz);
    rclcpp::on_shutdown([]() { KILL_dynamixel(); });

        //Dynamixel setting 
    CONNECT_dynamixel();
    SET_dynamixel();

    base_B_ref_pos = M_PI;
    base_A_ref_pos = M_PI;
    elbow_ref_pos = M_PI;
    ee_A_ref_pos = M_PI;
    ee_B_ref_pos = M_PI;
    
    while (rclcpp::ok()) 
    {
        BASE_position_control(base_B_ref_pos, base_A_ref_pos);
        ELBOW_position_control(elbow_ref_pos);
        EE_position_control(ee_A_ref_pos, ee_B_ref_pos);

        RCLCPP_INFO(rclcpp::get_logger("position_control"), "ID0 : %f | ID1 : %f | ID2 : %f | ID3 : %f | ID4 : %f | ", 
        base_B_now_pos*180/M_PI,
        base_A_now_pos*180/M_PI,
        elbow_now_pos*180/M_PI,
        ee_A_now_pos*180/M_PI,
        ee_B_now_pos*180/M_PI);
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    KILL_dynamixel(); 
    return 0;
}
