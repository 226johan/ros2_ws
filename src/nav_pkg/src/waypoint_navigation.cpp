#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<map>
#include<cstdlib>

class WaypointNavigation : public rclcpp::Node
{
public:
    WaypointNavigation() : Node("waypoint_navigation_node")
    {
        navigation_pub_ = this->create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10);
        result_sub_ = this->create_subscription<std_msgs::msg::String>("/waterplus/navi_result", 10, std::bind(&WaypointNavigation::ResultCallback, this, std::placeholders::_1));
    
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        waypoint_msg_.data="1";
        navigation_pub_->publish(waypoint_msg_);
        speak("开始导航,正在前往目标点1");


    }

private:

    void speak(const std::string &message){
        // std::string command = "pico2wave -w /tmp/tts.wav \"" + message + "\" && aplay /tmp/tts.wav";
        // std::string command = "espeak \"" + message + "\"";
        std::string command = "espeak-ng -v zh \"" + message + "\"";

        std::system(command.c_str());
    }

    void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
    {

        // point_msg_->data=msg->data;
        if(msg->data == "navi done")
        {
            if(waypoint_msg_.data == "1")
            {
                RCLCPP_INFO(this->get_logger(), "Arrived Waypoint 1 !");
                // speak("Arrived Waypoint 1 !");
                speak("到达目标点1,正在前往目标点2");
                waypoint_msg_.data = "2";
                navigation_pub_->publish(waypoint_msg_);
                return;
            }

            if(waypoint_msg_.data == "2")
            {
                RCLCPP_INFO(this->get_logger(), "Arrived Waypoint 2 !");
                // speak("Arrived Waypoint 2 !");
                speak("到达目标点2,正在前往目标点3");
                waypoint_msg_.data = "3";
                navigation_pub_->publish(waypoint_msg_);
                return;
            }

            if(waypoint_msg_.data == "3")
            {
                RCLCPP_INFO(this->get_logger(), "Arrived Waypoint 3 !");
                // speak("Arrived Waypoint 3 !");
                speak("到达目标点3,正在前往目标点4");
                waypoint_msg_.data = "4";
                navigation_pub_->publish(waypoint_msg_);
                return;
            }

            if(waypoint_msg_.data == "4")
            {
                RCLCPP_INFO(this->get_logger(), "Arrived Waypoint 4 !");    
                // speak("Arrived Waypoint 4 !");
                speak("到达目标点4,正在前往目标点1");
                waypoint_msg_.data = "1";
                navigation_pub_->publish(waypoint_msg_);
                return;

            }
            // if(waypoint_msg_.data == "5")
            // {
            //     RCLCPP_INFO(this->get_logger(), "Arrived Waypoint 4 !");    
            //     // speak("Arrived Waypoint 4 !");
            //     speak("到达目标点5,正在前往目标点6");
            //     waypoint_msg_.data = "6";
            //     navigation_pub_->publish(waypoint_msg_);
            //     return;

            // }
            // if(waypoint_msg_.data == "6")
            // {
            //     RCLCPP_INFO(this->get_logger(), "Arrived Waypoint 4 !");    
            //     // speak("Arrived Waypoint 4 !");
            //     speak("到达目标点6,正在前往目标点7");
            //     waypoint_msg_.data = "7";
            //     navigation_pub_->publish(waypoint_msg_);
            //     return;

            // }
            // if(waypoint_msg_.data == "7")
            // {
            //     RCLCPP_INFO(this->get_logger(), "Arrived Waypoint 4 !");    
            //     // speak("Arrived Waypoint 4 !");
            //     speak("到达目标点7,正在前往目标点8");
            //     waypoint_msg_.data = "8";
            //     navigation_pub_->publish(waypoint_msg_);
            //     return;

            // }
            // if(waypoint_msg_.data == "8")
            // {
            //     RCLCPP_INFO(this->get_logger(), "Arrived Waypoint 4 !");    
            //     // speak("Arrived Waypoint 4 !");
            //     speak("到达目标点8,正在前往目标点1");
            //     waypoint_msg_.data = "1";
            //     navigation_pub_->publish(waypoint_msg_);
            //     return;

            // }
        }



    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;

    std_msgs::msg::String waypoint_msg_;


};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}