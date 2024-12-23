#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<geometry_msgs/msg/twist.hpp>

using namespace std;
using namespace cv;

class ObjectFollow : public rclcpp::Node{
public:
    ObjectFollow() : Node("object_follower") {
    namedWindow("RGB",WINDOW_AUTOSIZE);
    namedWindow("Result",WINDOW_AUTOSIZE);
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>("/kinect2/qhd/image_raw",10,std::bind(&ObjectFollow::image_callback,this,std::placeholders::_1));
    pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat imgOriginal = cv_ptr->image;
        Mat imgHSV;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

        vector<Mat> hsvSplit;
        split(imgHSV, hsvSplit);
        // 直方图均值化提高对比度和亮度分布，提升图片质量
        equalizeHist(hsvSplit[2], hsvSplit[2]);
        merge(hsvSplit, imgHSV);

        Mat imgThresholded;
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

        Mat element=getStructuringElement(MORPH_RECT, Size(5,5));
        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

        nImageWidth=imgThresholded.cols;
        nImageHeight=imgThresholded.rows;

        int nTargetX=0;
        int nTargetY=0;
        int nPixCount=0;

        geometry_msgs::msg::Twist cmd_vel_;
        float fVelFoward;
        float fVelTurn;

        for(int y=0;y<nImageHeight;y++)
        {
            for(int x=0;x<nImageWidth;x++)
            {
                if(imgThresholded.data[y*nImageWidth+x]==255){
                    nTargetX+=x;
                    nTargetY+=y;
                    nPixCount++;
                }
            }
        }
        if(nPixCount>0){
            nTargetX=nTargetX/nPixCount;
            nTargetY=nTargetY/nPixCount;
            printf("Target(%d %d) pixelCount = %d \n",nTargetX,nTargetY,nPixCount);
            Point line_begin=Point(nTargetX-10,nTargetY);
            Point line_end=Point(nTargetX+10,nTargetY);
            line(imgOriginal,line_begin,line_end,Scalar(255,0,0));
            line_begin.x=nTargetX;
            line_end.y=nTargetY+10;
            line(imgOriginal,line_begin,line_end,Scalar(255,0,0));

            // 控制距离和角度
            fVelFoward=(nImageHeight/2-nTargetY)*0.02;
            fVelTurn=(nImageHeight/2-nTargetX)*0.003;

            cmd_vel_.linear.x=fVelFoward;
            printf("VelFoward = %f \n",fVelFoward);
            cmd_vel_.linear.y=0;
            cmd_vel_.linear.z=0;
            cmd_vel_.angular.x=0;
            cmd_vel_.angular.y=0;
            cmd_vel_.angular.z=fVelTurn;


        }
        else{
            printf("No target found \n");
            cmd_vel_.linear.x=0;
            cmd_vel_.linear.y=0;
            cmd_vel_.linear.z=0;
            cmd_vel_.angular.x=0;
            cmd_vel_.angular.y=0;
            cmd_vel_.angular.z=0;
        }

        pub_cmd_vel->publish(cmd_vel_);
        imshow("RGB", imgOriginal);
        // imshow("HSV", imgHSV);
        imshow("Result", imgThresholded);

        waitKey(10);

    
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    

    int iLowH=10;
    int iHighH=40;

    int iLowS=90;
    int iHighS=255;
    
    int iLowV=1;
    int iHighV=255;


    int nImageWidth=0;
    int nImageHeight=0;

};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectFollow>();
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;       
}
