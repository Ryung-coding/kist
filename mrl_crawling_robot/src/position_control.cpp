#include "define.h"
#include "dynamixel_funtion.h"

void IK_2dim(double x, double y)
{
    double theta2 = -acos((x * x + y * y - 2 * L * L) / (2 * L * L));
    double theta1 = atan2(y, x) + asin(L * sin(-theta2) / sqrt(x * x + y * y));
    double theta3 = -(theta1 + theta2);

    base_B_ref_pos = M_PI/2;
    base_A_ref_pos = M_PI + theta1;
    elbow_ref_pos = M_PI + theta2;
    ee_A_ref_pos = M_PI + theta3;
    ee_B_ref_pos = M_PI/2;
}

void FK_2dim(double theta1, double theta2, double &x_ee, double &y_ee)
{
    x_ee = L * cos(theta1) + L * cos(theta1 + theta2);
    y_ee = L * sin(theta1) + L * sin(theta1 + theta2);
}

void path()
{
    if (target_index < points.size()) 
    {
        x_target = points[target_index].first;
        y_target = points[target_index].second;
        distance = sqrt(pow(x_target - x_ee, 2) + pow(y_target - y_ee, 2));

        if (distance > step_size) 
        {
            x_ee += step_size * (x_target - x_ee) / distance;
            y_ee += step_size * (y_target - y_ee) / distance;
        } 
        else target_index = (target_index + 1) % points.size();
    }
}

int main(int argc, char **argv)
{
    // Port setting
    system(("sudo chmod 666 " + std::string(DEVICENAME)).c_str());

    // ROS2 setting
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("position_control");
    rclcpp::WallRate loop_rate(loop_hz);
    rclcpp::on_shutdown([]() { KILL_dynamixel(); });

    // Dynamixel setting 
    CONNECT_dynamixel();
    SET_dynamixel();
    
    
    while (rclcpp::ok()) 
    {


        path();
        IK_2dim(x_ee, y_ee);
        BASE_position_control(base_B_ref_pos, base_A_ref_pos);
        ELBOW_position_control(elbow_ref_pos);
        EE_position_control(ee_A_ref_pos, ee_B_ref_pos);

        RCLCPP_INFO(rclcpp::get_logger("position_control"), "ID0 : %f | ID1 : %f | ID2 : %f | ID3 : %f | ID4 : %f | ", 
        base_B_now_pos * 180 / M_PI,
        base_A_now_pos * 180 / M_PI,
        elbow_now_pos * 180 / M_PI,
        ee_A_now_pos * 180 / M_PI,
        ee_B_now_pos * 180 / M_PI);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    KILL_dynamixel(); 
    return 0;
}