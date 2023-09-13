#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgbd_sensor_publisher");
    ros::NodeHandle nh;

    // Create publishers for RGB and depth images
    ros::Publisher rgb_pub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_raw", 1);
    ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);

    // Create ROS image messages
    sensor_msgs::Image rgb_msg;
    sensor_msgs::Image depth_msg;

    // Fill in header information for the images
    rgb_msg.header.frame_id = "rgbd_sensor_link"; // Adjust frame_id as needed
    rgb_msg.encoding = "rgb8"; // RGB encoding
    depth_msg.header.frame_id = "rgbd_sensor_link"; // Adjust frame_id as needed
    depth_msg.encoding = "32FC1"; // 32-bit float encoding for depth data

    // Create a counter to simulate changing data
    int counter = 0;

    ros::Rate loop_rate(10); // Publish at 10 Hz

    while (ros::ok())
    {
        // Generate synthetic RGB image with changing colors
        cv::Mat rgb_image(480, 640, CV_8UC3);
        cv::Scalar color(255, 0, 0); // Initialize color to red
        color[0] = static_cast<int>((static_cast<int>(color[0]) + counter) % 256); // Change the color with counter
        cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGR2RGB); // Convert to RGB format
        rgb_image.setTo(color);

        // Generate synthetic depth image with changing depth values
        cv::Mat depth_image(480, 640, CV_32FC1);
        depth_image.setTo(static_cast<float>(counter) / 100.0); // Change depth value with counter

        // Convert OpenCV images to ROS image messages
        cv_bridge::CvImage(rgb_msg.header, rgb_msg.encoding, rgb_image).toImageMsg(rgb_msg);
        cv_bridge::CvImage(depth_msg.header, depth_msg.encoding, depth_image).toImageMsg(depth_msg);

        // Publish the RGB and depth images
        rgb_pub.publish(rgb_msg);
        depth_pub.publish(depth_msg);

        // Increment the counter for changing data
        counter++;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
