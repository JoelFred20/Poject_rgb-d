#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher cmd_vel_pub;

// Variables for obstacle detection and avoidance
bool obstacleDetected = false;
bool turning = false; // Indicates if the robot is currently turning to avoid an obstacle

void depthCameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // If an obstacle has already been detected or if the robot is turning, do nothing
    if (obstacleDetected || turning) {
        return;
    }

    // Convert the PointCloud2 message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Create a voxel grid filter to downsample the point cloud
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.05, 0.05, 0.05); // Adjust voxel size as needed
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*filtered_cloud);

    // Implement obstacle detection logic
    for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
        pcl::PointXYZ point = filtered_cloud->points[i];
        // Check if the voxel is occupied based on a threshold
        if (point.z > 0.1) {  // Adjust the threshold as needed
            obstacleDetected = true;
            break; // Exit the loop on the first detected obstacle
        }
    }

    // Publish collision avoidance commands
    geometry_msgs::Twist cmd_vel;
    if (obstacleDetected) {
        // If an obstacle is detected, stop or change the robot's trajectory
        cmd_vel.linear.x = 0.0;

        // Perform obstacle avoidance maneuver (turn right)
        cmd_vel.angular.z = -0.2; // Rotate to avoid the obstacle (adjust as needed)
        turning = true; // Set the turning flag
    } else {
        // If no obstacle is detected, proceed forward
        cmd_vel.linear.x = 0.2; // Adjust linear velocity as needed
        cmd_vel.angular.z = 0.0;
        turning = false; // Reset the turning flag
    }

    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection_node");
    ros::NodeHandle nh;

    ros::Subscriber depthCameraSub = nh.subscribe("/head/depth/points", 1, depthCameraCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();

    return 0;
}
