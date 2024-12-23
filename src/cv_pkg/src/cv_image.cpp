#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

class CVRGBCamera : public rclcpp::Node
{
public:
    CVRGBCamera() : Node("cv_camerargb")
    {
        cv::namedWindow("RGB", cv::WINDOW_AUTOSIZE);
        image_sub_  = this->create_subscription<sensor_msgs::msg::Image>("/kinect2/qhd/image_raw",10,std::bind(&CVRGBCamera::CamRGBCallback, this, std::placeholders::_1));
    }


private:
    void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat imgOriginal = cv_ptr->image;
        RCLCPP_INFO(this->get_logger(), "IMG %d",imgOriginal.rows);
        cv::imshow("RGB", imgOriginal);
        cv::waitKey(10);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CVRGBCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    // cv::destroyAllWindows();
    return 0;
}