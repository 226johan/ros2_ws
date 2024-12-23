#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<geometry_msgs/msg/twist.hpp>

class ImuData : public rclcpp::Node{
public:
    ImuData() : Node("imu_node") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data",10,std::bind(&ImuData::ImuCallback,this,std::placeholders::_1));
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

    }

private:
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
        tf2::Quaternion tf2_quaternion;
        tf2_quaternion.setX(msg->orientation.x);
        tf2_quaternion.setY(msg->orientation.y);
        tf2_quaternion.setZ(msg->orientation.z);
        tf2_quaternion.setW(msg->orientation.w);

        //将四元数转换为旋转矩阵
        tf2::Matrix3x3 matrix(tf2_quaternion);
        //将旋转矩阵转换为欧拉角（弧度）
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        //将欧拉角（弧度）转换为角度（角度制）
        roll = roll * 180 / M_PI;
        pitch = pitch * 180 / M_PI;
        yaw = yaw * 180 / M_PI;
        RCLCPP_INFO(this->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

        double targe_yaw = 90;

        double error_yaw = targe_yaw - yaw;
        twist_msg_.angular.z = error_yaw * 0.01;
        twist_msg_.linear.x = 0.1;
        twist_pub_->publish(twist_msg_);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    geometry_msgs::msg::Twist twist_msg_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuData>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}