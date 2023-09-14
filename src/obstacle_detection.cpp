#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <termios.h>

ros::Publisher frontLeftWheelPub;
ros::Publisher frontRightWheelPub;
ros::Publisher backRightWheelPub;
ros::Publisher backLeftWheelPub;

ros::Publisher frontLeftWheelPubTurn;
ros::Publisher frontRightWheelPubTurn;
ros::Publisher backRightWheelPubTurn;
ros::Publisher backLeftWheelPubTurn;

ros::Publisher statusPub; // Publisher for status messages

bool obstacleDetected = false;

int getKey() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void depthCameraCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (obstacleDetected) {
        return; // If an obstacle has already been detected, do nothing
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
        if (point.z > 0.7) {  // Adjust the threshold as needed
            obstacleDetected = true;
            return; // Exit the function on the first detected obstacle
        }
    }
}

void moveRobot(double linearVel, double angularVel) {
    // Check for obstacle detection
    if (obstacleDetected) {
        // Stop the robot completely
        linearVel = 0.0;
        angularVel = 0.0;
    }

    // Publish velocity commands to control the wheels
    std_msgs::Float64 velocityMsg;

    if (!obstacleDetected) {
        velocityMsg.data = linearVel; // Publish the desired linear velocity
    } else {
        velocityMsg.data = 0.0; // Publish a linear velocity of 0.0 when an obstacle is detected
    }

    frontLeftWheelPub.publish(velocityMsg);
    frontRightWheelPub.publish(velocityMsg);
    backRightWheelPub.publish(velocityMsg);
    backLeftWheelPub.publish(velocityMsg);

    // Publish position commands to turn the wheels (if needed)
    std_msgs::Float64 positionMsg;
    positionMsg.data = angularVel;
    frontRightWheelPubTurn.publish(positionMsg);
    frontLeftWheelPubTurn.publish(positionMsg);
    backLeftWheelPubTurn.publish(positionMsg);
    backRightWheelPubTurn.publish(positionMsg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "epic_node");
    ros::NodeHandle n;

    // Define publishers for wheel controllers
    frontLeftWheelPub = n.advertise<std_msgs::Float64>("front_left_arm_controller/command", 1000);
    frontRightWheelPub = n.advertise<std_msgs::Float64>("front_right_arm_controller/command", 1000);
    backRightWheelPub = n.advertise<std_msgs::Float64>("back_right_arm_controller/command", 1000);
    backLeftWheelPub = n.advertise<std_msgs::Float64>("back_left_arm_controller/command", 1000);

    frontLeftWheelPubTurn = n.advertise<std_msgs::Float64>("front_left_arm_controller_turn/command", 1000);
    frontRightWheelPubTurn = n.advertise<std_msgs::Float64>("front_right_arm_controller_turn/command", 1000);
    backRightWheelPubTurn = n.advertise<std_msgs::Float64>("back_right_arm_controller_turn/command", 1000);
    backLeftWheelPubTurn = n.advertise<std_msgs::Float64>("back_left_arm_controller_turn/command", 1000);

    // Publisher for status messages
    statusPub = n.advertise<std_msgs::String>("robot_status", 1000);

    ros::Subscriber depthCameraSub = n.subscribe("/head/depth/points", 1, depthCameraCallback);

    std_msgs::Float64 initialPos;
    initialPos.data = 0.0;
    frontRightWheelPubTurn.publish(initialPos);
    frontLeftWheelPubTurn.publish(initialPos);
    backLeftWheelPubTurn.publish(initialPos);
    backRightWheelPubTurn.publish(initialPos);

    double linearVel = 0.0; // Initial linear velocity
    double angularVel = 0.0;

    // Publish a status message indicating that the robot is starting
    std_msgs::String statusMsg;
    statusMsg.data = "Robot started";
    statusPub.publish(statusMsg);

    bool startMoving = false;

    while (ros::ok()) {
        if (!startMoving) {
            ROS_INFO("Press 'w' to start moving the robot...");
            int key = getKey();
            if (key == 'w') {
                startMoving = true;
                linearVel = 5.0; // Set linear velocity to start moving
            }
        } else {
            // Move the robot forward (linear motion)
            moveRobot(linearVel, angularVel);
        }

        // Sleep for a short duration to control update rate
        ros::Duration(0.1).sleep();
        ros::spinOnce(); // Process any callbacks

        // Reset obstacle detection flag after moving a certain distance
        if (linearVel > 0.0) {
            static double distanceTraveled = 0.0;
            distanceTraveled += linearVel * 0.1; // Assuming a constant velocity for 0.1 seconds
            if (distanceTraveled > 1.0) { // Adjust the distance threshold as needed
                obstacleDetected = false;
                distanceTraveled = 0.0;
            }
        }

        // Publish a status message at each step
        statusMsg.data = "Robot moving";
        statusPub.publish(statusMsg);
    }

    return 0;
}
