#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>

// Callback function for image data
void imageCallback(const sensor_msgs::ImageConstPtr& msg, ros::Publisher& image_pub) {
    try {
        // Convert the ROS image message to an OpenCV image
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Publish the image for visualization in RViz
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub.publish(img_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// Callback function for point cloud data
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, ros::Publisher& cloud_pub) {
    try {
        // Publish the point cloud for visualization in RViz
        cloud_pub.publish(*msg);
    } catch (std::runtime_error& e) {
        ROS_ERROR("Error processing point cloud data: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_visualizer");
    ros::NodeHandle nh;

    // Create publishers for image and point cloud topics
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image_topic", 1);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);

    // Subscribe to the camera image topic
    ros::Subscriber image_sub = nh.subscribe("/head/color/image_raw", 1, boost::bind(imageCallback, _1, image_pub));

    // Subscribe to the point cloud topic
    ros::Subscriber point_cloud_sub = nh.subscribe("/head/depth/points", 1, boost::bind(pointCloudCallback, _1, cloud_pub));

    // Initialize ROS image transport
    image_transport::ImageTransport it(nh);

    // Initialize OpenCV window for image visualization (optional)
    // cv::namedWindow("Image Viewer", cv::WINDOW_AUTOSIZE);

    // Create a visualization marker (optional)
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    visualization_msgs::Marker marker;
    // Configure the marker here (e.g., for visualization in RViz)
    marker.header.frame_id = "base_link"; // Replace with the appropriate frame_id
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // Loop to update RViz visualization
    ros::Rate loop_rate(10);  // Adjust the loop rate as needed
    while (ros::ok()) {
        // Publish the marker for visualization in RViz (optional)
        marker_pub.publish(marker);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
