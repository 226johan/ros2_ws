#include"rclcpp/rclcpp.hpp"
#include"sensor_msgs/msg/laser_scan.hpp"
#include"geometry_msgs/msg/twist.hpp"

class LidarObstacle : public rclcpp::Node
{
    public:
    LidarObstacle() : Node("lidar_node")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&LidarObstacle::LidarCallback,this,std::placeholders::_1));
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    }

    
    private:
    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        int nMid=msg->ranges.size()/2;
        int nNum=0;
        
        float fMidDist=msg->ranges[nMid];
        RCLCPP_INFO(this->get_logger(),"ranges[%d] = %f m",nMid,fMidDist);

        if(nCount_>0){
            nCount_--;
            RCLCPP_INFO(this->get_logger(),"nCount = %d m",nCount_);
            return;
        }

        if(fMidDist<1.5f){
            twist_msg_.angular.z=0.3;
            // twist_msg_.linear.x=0.05;
            nCount_=100;
            
        }
        else{
            twist_msg_.angular.z=0.0;
            twist_msg_.linear.x=0.2;
        }

        twist_pub_->publish(twist_msg_);

        
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    geometry_msgs::msg::Twist twist_msg_;
    int nCount_=0;    //旋转计时器
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarObstacle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}