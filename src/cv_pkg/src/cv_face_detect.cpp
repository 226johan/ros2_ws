#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<sensor_msgs/msg/region_of_interest.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <geometry_msgs/msg/twist.hpp>


class FaceDetect :public rclcpp::Node{
public:
    FaceDetect() : Node("cv_face_detect"){
        cv::namedWindow("Face",cv::WINDOW_NORMAL);
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>("/kinect2/qhd/image_raw",10,std::bind(&FaceDetect::CamRGBCallback,this,std::placeholders::_1));
        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/face_detector_input",10);
        pub_face_ = this->create_subscription<sensor_msgs::msg::RegionOfInterest>("/face_position",10,std::bind(&FaceDetect::FaceCallback,this,std::placeholders::_1));
        pub_vel_= this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }


private:
    void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 获取图像发布到人连检测节点，
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        img_face_=cv_ptr->image;
        target_x_ = img_face_.cols * 0.5;
        target_y_ = img_face_.rows * 0.5;
        pub_image_->publish(*msg);

    }

    void FaceCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
    {
        if(msg.get() != nullptr)
        {
            // 获取到的是人脸的roi区域，注意坐标计算方式
            int face_pose_x = msg->x_offset + (msg->width * 0.5);
            int face_pose_y = msg->y_offset + (msg->height * 0.5);
            float fVelForward = (target_y_ - face_pose_y) * 0.003;
            float fVelTurn = (target_x_ - face_pose_x) * 0.003;    
            vel_cmd_.linear.x = 0;
            vel_cmd_.linear.y = 0;
            vel_cmd_.linear.z = 0;
            vel_cmd_.angular.x = 0;
            vel_cmd_.angular.y = 0;
            vel_cmd_.angular.z = fVelTurn;
            pub_vel_->publish(vel_cmd_);

            cv::rectangle(img_face_,cv::Point(msg->x_offset,msg->y_offset),cv::Point(msg->x_offset+msg->width,msg->y_offset+msg->height),cv::Scalar(0,0,255),2);
            cv::imshow("Face",img_face_);
            cv::waitKey(1);

        }
        else{
            RCLCPP_INFO(this->get_logger(),"No face detected!");
            vel_cmd_.linear.x = 0;
            vel_cmd_.linear.y = 0;
            vel_cmd_.linear.z = 0;
            vel_cmd_.angular.x = 0;
            vel_cmd_.angular.y = 0;
            vel_cmd_.angular.z = 0;
            pub_vel_->publish(vel_cmd_);


        }


    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr pub_face_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    geometry_msgs::msg::Twist vel_cmd_;
    cv::Mat img_face_;

    int target_x_;
    int target_y_;
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceDetect>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}