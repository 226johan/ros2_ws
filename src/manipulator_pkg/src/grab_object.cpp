#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include<wpr_simulation2/msg/object.hpp>


class GrabObject : public rclcpp::Node
{
public:
    GrabObject() : Node("grab_object")
    {
        cmd_pub_=this->create_publisher<std_msgs::msg::String>("/wpb_home/behavior",10);
        vel_pub_=this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
        mani_pub_=this->create_publisher<sensor_msgs::msg::JointState>("/wpb_home/mani_ctrl",10);
        obj_sub_=this->create_subscription<wpr_simulation2::msg::Object>("/wpb_home/objects_3d",10,std::bind(&GrabObject::ObjectCallback,this,std::placeholders::_1));

        align_x_=1.0;
        align_y_=0.0;
        object_x_=0.0;
        object_y_=0.0;
        object_z_=0.0;
        grab_step_=STEP_WAIT;

        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&GrabObject::TimerCallback, this));
    }
private:
    void ObjectCallback(const wpr_simulation2::msg::Object::SharedPtr msg)
    {
        if(grab_step_==STEP_WAIT)
        {
            grab_step_=STEP_ALIGN_OBJ;
        }
        if(grab_step_==STEP_ALIGN_OBJ)
        {
            object_x_=msg->x[0];
            object_y_=msg->y[0];
            object_z_=msg->z[0];
        }

    }

    void TimerCallback()
    {
        if(grab_step_==STEP_WAIT)
        {
            std_msgs::msg::String start_msg;
            start_msg.data="start objects";
            cmd_pub_->publish(start_msg);
            RCLCPP_INFO(this->get_logger(),"[STEP_WAIT]");
            return;
        }
        if(grab_step_==STEP_ALIGN_OBJ)
        {
            float x_diff=object_x_-align_x_;
            float y_diff=object_y_-align_y_;
            geometry_msgs::msg::Twist vel_msg;
            if(std::fabs(x_diff)>0.02 || std::fabs(y_diff)>0.01)
            {
                vel_msg.linear.x=x_diff*0.8;
                vel_msg.linear.y=y_diff*0.8;
            }
            else
            {
                vel_msg.linear.x=0.0;
                vel_msg.linear.y=0.0;
                std_msgs::msg::String start_msg;
                start_msg.data="stop objects";
                cmd_pub_->publish(start_msg);
                grab_step_=STEP_HAND_UP;
            }
            RCLCPP_INFO(this->get_logger(),"[STEP_ALIGN_OBJ] vel = (%.2f, %.2f)",vel_msg.linear.x,vel_msg.linear.y);
            vel_pub_->publish(vel_msg);
            return;
        }
        if(grab_step_==STEP_HAND_UP)
        {
            RCLCPP_INFO(this->get_logger(),"[STEP_HAND_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0]="lift";
            mani_msg.name[1]="gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0]=object_z_;
            mani_msg.position[1]=0.15;
            mani_pub_->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(8000));
            grab_step_=STEP_FORWORD;
            return;
        }
        if(grab_step_==STEP_FORWORD)
        {
            RCLCPP_INFO(this->get_logger(),"[STEP_FORWORD] objecr_x = %.2f",object_x_);
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x=0.1;
            vel_msg.linear.y=0.0;
            vel_pub_->publish(vel_msg);
            int forword_duration = (object_x_-0.65)*20000;
            RCLCPP_INFO(this->get_logger(),"[STEP_FORWORD] vel_x = %.2f",vel_msg.linear.x);
            rclcpp::sleep_for(std::chrono::milliseconds(forword_duration));
            RCLCPP_INFO(this->get_logger(),"[STEP_FORWORD1] vel_x = %.2f",vel_msg.linear.x);
            grab_step_=STEP_GRAB;
            RCLCPP_INFO(this->get_logger(),"[STEP_FORWORD2] vel_x = %.2f",vel_msg.linear.x);
            return;
        }
        if (grab_step_==STEP_GRAB)
        {
            RCLCPP_INFO(get_logger(), "[STEP_GRAB]");
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_pub_->publish(vel_msg);
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z_;
            mani_msg.position[1] = 0.07;
            mani_pub_->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            grab_step_ = STEP_OBJ_UP;
            return;
        }
        if(grab_step_==STEP_OBJ_UP)
        {
            RCLCPP_INFO(get_logger(), "[STEP_OBJ_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z_ + 0.05;
            mani_msg.position[1] = 0.07;
            mani_pub_->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            grab_step_ = STEP_BACKWARD;
            return;
        }
        if(grab_step_==STEP_BACKWARD)
        {
            RCLCPP_INFO(get_logger(), "[STEP_BACKWARD]");
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = -0.1;
            vel_msg.linear.y = 0;
            vel_pub_->publish(vel_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(10000));
            grab_step_ = STEP_DONE;
            RCLCPP_INFO(get_logger(), "[STEP_DONE]");
            return;
        }
        if (grab_step_ == STEP_DONE)
        {
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_pub_->publish(vel_msg);
            return;
        }
    }

    static constexpr int STEP_WAIT =0;          // 初始状态，发布”stat object“ 激活pcl_object检测功能，接受到检测结果切换到STEP_ALIGN_OBJ状态
    static constexpr int STEP_ALIGN_OBJ =1;     // 左右平移，使正前方对准物体，对准后切换到STEP_HAND_UP状态
    static constexpr int STEP_HAND_UP =2;       // 手抬起，准备抓取切换到STEP_FORWORD状态
    static constexpr int STEP_FORWORD =3;       // 根据距离前进，使物体进入夹爪手中，切换到STEP_GRAB状态
    static constexpr int STEP_GRAB =4;          // 闭合手臂，完成夹取，切换到STEP_OBJ_UP状态
    static constexpr int STEP_OBJ_UP =5;        // 向上抬升，离开桌面，切换到STEP_BACKWORD状态
    static constexpr int STEP_BACKWARD =6;      // 后退，远离桌面切换到STEP_DOWN状态
    static constexpr int STEP_DONE =7;          // 完成抓取
    
    // 状态机变量
    int grab_step_;
    // 物体位置
    float object_x_;
    float object_y_;
    float object_z_;
    // 平移目标点
    float align_x_;
    float align_y_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub_;
    rclcpp::Subscription<wpr_simulation2::msg::Object>::SharedPtr obj_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GrabObject>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
