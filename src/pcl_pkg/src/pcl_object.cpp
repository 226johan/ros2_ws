#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


class PCLObject : public rclcpp::Node{
public:
    PCLObject() : Node("pcl_object") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        "/kinect2/sd/points", 
                        10,
                        std::bind(&PCLObject::PointCloudCallback, this, std::placeholders::_1));

    }

private:
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 查询是否存在机器人到相机变换的坐标系tf，不存在则直接返回
        bool result = tf_buffer_->canTransform("base_footprint",msg->header.frame_id,msg->header.stamp);
        if(!result)
        {
            return;
        }
        // 将msg中的点云消息msg从相机坐标系转换到机器人坐标系
        sensor_msgs::msg::PointCloud2 pcl_footprint;
        pcl_ros::transformPointCloud("base_footprint",
                                     *msg,
                                     pcl_footprint,
                                     *tf_buffer_);
        // 将点云转换成pcl格式
        pcl::PointCloud<pcl::PointXYZ> cloud_src;
        pcl::fromROSMsg(pcl_footprint, cloud_src);
        
        // 构造直通滤波器裁减点云对象
        // 保留x 0.5-1.5 y -0.5-0.5 z 0.5-1.5
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_src.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.5, 1.5);
        pass.filter(cloud_src);
        pass.setInputCloud(cloud_src.makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.5, 0.5);
        pass.filter(cloud_src);
        pass.setInputCloud(cloud_src.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.5, 1.5);
        pass.filter(cloud_src);
        
        // 构造平面分割对象
        // 模型::SACMODEL_PLANE 平面模型
        // 方法::SAC_RANSAC 随机采样一致性算法
        // 平面连通阈值0.05
        // 是否优化 true
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setInputCloud(cloud_src.makeShared());
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setDistanceThreshold(0.05);
        segmentation.setOptimizeCoefficients(true);
        pcl::PointIndices::Ptr planeinliers(new pcl::PointIndices);
        segmentation.segment(*planeinliers, *coefficients);


        // 构造聚类对象
        // 计算桌面高度
        int point_num=planeinliers->indices.size();
        float points_z_nums=0;
        for(int i=0;i<point_num;i++){
            int point_index=planeinliers->indices[i];
            points_z_nums+=cloud_src.points[point_index].z;
        }
        float plane_height=points_z_nums/point_num;
        RCLCPP_INFO(this->get_logger(),"plane height is %.2f",plane_height);

        // 构造直通滤波器裁减点云对象
        // 保留桌面高度0.2-1.5的点集
        pass.setInputCloud(cloud_src.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(plane_height+0.2, 1.5);
        pass.filter(cloud_src);
        // 构造KD树搜索结构tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_src.makeShared());

        // 使用KD tree进行聚类分割
        // 使用欧几里德区类分割其
        // param
        // 最小分割尺寸 100
        // 最大分割尺寸 25000
        // 聚类距离阈值 0.1
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setInputCloud(cloud_src.makeShared());
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setClusterTolerance(0.1);
        ec.setSearchMethod(tree);
        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);
        int object_num=cluster_indices.size();
        RCLCPP_INFO(this->get_logger(),"object num is %d",object_num);
        // 计算每个对象的质心坐标
        for(int i=0;i<object_num;i++){
            int point_num=cluster_indices[i].indices.size();
            float points_x_nums=0;
            float points_y_nums=0;
            float points_z_nums=0;
            for(int j=0;j<point_num;j++){
                int point_index=cluster_indices[i].indices[j];
                points_x_nums+=cloud_src.points[point_index].x;
                points_y_nums+=cloud_src.points[point_index].y;
                points_z_nums+=cloud_src.points[point_index].z;
            }
            float object_x=points_x_nums/point_num;
            float object_y=points_y_nums/point_num;
            float object_z=points_z_nums/point_num;
            RCLCPP_INFO(this->get_logger(),"object %d pos = (%.2f, %.2f, %.2f)",i,object_x,object_y,object_z);
            
        }
        RCLCPP_INFO(this->get_logger(),"-------------------------");


    }
    tf2_ros::Buffer::SharedPtr tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLObject>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}