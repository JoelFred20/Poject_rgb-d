#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher voxelized_cloud_publisher;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Convert PointCloud2 message to PCL PointCloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Voxelization (Downsampling)
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.01, 0.01, 0.01); // Set your desired voxel size
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*voxelized_cloud);

    // Publish the voxelized cloud
    sensor_msgs::PointCloud2 voxelized_cloud_msg;
    pcl::toROSMsg(*voxelized_cloud, voxelized_cloud_msg);
    voxelized_cloud_msg.header = cloud_msg->header;
    voxelized_cloud_publisher.publish(voxelized_cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_layer_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/your/point_cloud_topic", 1, pointCloudCallback);
    voxelized_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/voxelized_cloud_topic", 1);

    ros::spin();

    return 0;
}
