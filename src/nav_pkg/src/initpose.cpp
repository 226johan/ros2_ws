#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

class PoseInitializer : public rclcpp::Node
{
public:
    PoseInitializer()
        : Node("pose_initializer")
    {
        // 创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        
        // 初始化位置
        publish_initial_pose();
    }

private:
    void publish_initial_pose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();

        // 设置时间戳和帧id
        msg.header.stamp = this->now();
        msg.header.frame_id = "map"; // 或者您所使用的其他坐标系名

        // 设置位置（x, y)和方向（四元数）
        msg.pose.pose.position.x = 0.0; // 您想要设置的x坐标
        msg.pose.pose.position.y = 0.0; // 您想要设置的y坐标
        msg.pose.pose.position.z = 0.0; // Z坐标一般为0

        // 假设方向是0（朝向x轴），可以使用四元数进行设置
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;

        // 设置协方差（这里可以是一个零矩阵，表示没有不确定性）
        msg.pose.covariance.fill(0.0);
        // msg.pose.covariance[0] = 0.1; // x协方差
        // msg.pose.covariance[7] = 0.1; // y协方差
        // msg.pose.covariance[14] = 0.1; // 方向协方差

        // 发布消息
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseInitializer>());
    rclcpp::shutdown();
    return 0;
}
