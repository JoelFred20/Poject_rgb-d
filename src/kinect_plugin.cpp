// kinect_plugin.cpp
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

namespace gazebo {

class KinectPlugin : public SensorPlugin {
public:
  KinectPlugin() : SensorPlugin() {}

  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {
    // Check if the sensor is a camera sensor
    if (_sensor->Type() != sensors::CameraSensor::Type) {
      gzerr << "This plugin only works with camera sensors\n";
      return;
    }

    // Get the camera sensor
    this->cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

    if (!this->cameraSensor) {
      gzerr << "Failed to cast to CameraSensor\n";
      return;
    }

    // Connect to the camera sensor's update event
    this->updateConnection = this->cameraSensor->ConnectUpdated(
        std::bind(&KinectPlugin::OnCameraUpdate, this));

    // Initialize ROS node and publisher for point cloud
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "kinect_plugin");
    this->rosNode.reset(new ros::NodeHandle("kinect_plugin"));
    this->pointCloudPub = this->rosNode->advertise<sensor_msgs::PointCloud2>("/kinect/point_cloud", 1);
  }

  void OnCameraUpdate() {
    // Access camera data (image and depth)
    const auto imageMsg = this->cameraSensor->CaptureCamera();
    const auto depthMsg = this->cameraSensor->DepthMsg();

    // Check if the depth image is available
    if (!depthMsg) {
      gzerr << "Depth data not available\n";
      return;
    }

    // Process depth data and create a point cloud
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

    // Get the depth image dimensions
    int width = depthMsg->width;
    int height = depthMsg->height;

    // Loop through the depth image
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        float depthValue = depthMsg->data[y * width + x] / 1000.0; // Convert from millimeters to meters

        // Skip invalid depth values
        if (depthValue <= 0.0 || depthValue > 10.0) {
          continue;
        }

        // Calculate 3D point in camera frame
        double fx = this->cameraSensor->Camera()->ImageElement(0)->FocalLength();
        double fy = this->cameraSensor->Camera()->ImageElement(1)->FocalLength();
        double cx = this->cameraSensor->Camera()->ImageElement(0)->Center();
        double cy = this->cameraSensor->Camera()->ImageElement(1)->Center();
        
        double x3d = (x - cx) * depthValue / fx;
        double y3d = (y - cy) * depthValue / fy;
        double z3d = depthValue;

        // Add point to point cloud
        pcl::PointXYZRGB point;
        point.x = x3d;
        point.y = y3d;
        point.z = z3d;
        point.r = 255; // Placeholder for RGB color values
        point.g = 0;
        point.b = 0;

        pointCloud.push_back(point);
      }
    }

    // Create a ROS PointCloud2 message
    sensor_msgs::PointCloud2 pointCloudMsg;
    pcl::toROSMsg(pointCloud, pointCloudMsg);
    pointCloudMsg.header.frame_id = "kinect_rgb_optical_frame"; // Adjust the frame ID

    // Publish the point cloud to ROS
    this->pointCloudPub.publish(pointCloudMsg);

    // Implement your processing logic here, e.g., accessing camera info
    const auto cameraInfo = this->cameraSensor->CameraInfo();
    gzmsg << "Camera Resolution: " << cameraInfo.width << " x " << cameraInfo.height << "\n";
  }

private:
  sensors::CameraSensorPtr cameraSensor;
  event::ConnectionPtr updateConnection;

  ros::NodeHandlePtr rosNode;
  ros::Publisher pointCloudPub;
};

GZ_REGISTER_SENSOR_PLUGIN(KinectPlugin)
}  // namespace gazebo
