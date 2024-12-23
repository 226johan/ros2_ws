#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>

// 是否保存点云
bool pcl_save_flag=false;


class PCLData : public rclcpp::Node{
public:
    PCLData() : Node("pcl_data_node") {
        sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/kinect2/sd/points", 10, std::bind(&PCLData::PCLCallback, this, std::placeholders::_1));
        pointCloudData_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

private:
    void PCLCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        
        pcl::fromROSMsg(*msg, *pointCloudData_);

        int cloudSize = pointCloudData_->points.size();
        for(int i=0;i<cloudSize;i++)
        {
            RCLCPP_INFO(this->get_logger(), "[i=%d]:(%.2f %.2f %.2f)", i, pointCloudData_->points[i].x, pointCloudData_->points[i].y, pointCloudData_->points[i].z);
        }
        if(pcl_save_flag == true)
        {
            pcl::io::savePCDFileASCII("pointcloud_data.pcd", *pointCloudData_);
            RCLCPP_INFO(this->get_logger(), "Point cloud data saved as pointcloud_data.pcd");
            pcl_save_flag = false;
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudData_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLData>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}