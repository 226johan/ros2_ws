#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/joint_state.hpp>

class ManiControl : public rclcpp::Node
{

/**
 * lift  机械臂基座的升降高度
 * gripper  手抓的指间宽度
*/
public:
  ManiControl() : Node("mani_control")
  {
    mani_msg_.name.resize(2);
    mani_msg_.name[0] = "lift";
    mani_msg_.name[1] = "gripper";

    mani_msg_.position.resize(2);
    mani_msg_.position[0] = 0.0;
    mani_msg_.position[1] = 0.0;

    pose_id_ = 1;

    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/wpb_home/mani_ctrl", 
      10
    );
    
    timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&ManiControl::TimerCallback,this)
    );
  }

  void TimerCallback()
  {
    if(pose_id_ == 1)
    {
      RCLCPP_WARN(this->get_logger(), "Pose 1");
      mani_msg_.position[0] = 0.0;
      mani_msg_.position[1] = 0.01;
      pub_joint_states_->publish(mani_msg_);
      pose_id_ = 2;
      return;
    }
 
    if(pose_id_ == 2)
    {
      RCLCPP_WARN(this->get_logger(), "Pose 2");
      mani_msg_.position[0] = 1.0;
      mani_msg_.position[1] = 0.1;
      pub_joint_states_->publish(mani_msg_);
      pose_id_ = 1;
      return;
    }
  }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    sensor_msgs::msg::JointState mani_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    int pose_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManiControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}