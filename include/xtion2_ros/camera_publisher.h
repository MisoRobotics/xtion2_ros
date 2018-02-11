// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#ifndef XTION2_ROS_CAMERA_PUBLISHER_H
#define XTION2_ROS_CAMERA_PUBLISHER_H

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

namespace cv
{
class Mat;
}

namespace xtion2_ros
{
class IOInterface;

class CameraPublisher
{
  unsigned int counter_;

  ros::NodeHandle color_nh_;
  ros::NodeHandle depth_nh_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport color_it_;
  image_transport::ImageTransport depth_it_;
  camera_info_manager::CameraInfoManager color_manager_;
  camera_info_manager::CameraInfoManager depth_manager_;
  image_transport::Publisher color_pub_;
  image_transport::Publisher depth_pub_;

  void publishImage(const cv::Mat img, const std::string& type, image_transport::Publisher& publisher);

  cv_bridge::CvImage img_bridge_;

public:
  explicit CameraPublisher(const std::string& camera_name);

  void publish(IOInterface& xtion_interface);
};
}  // namespace xtion2_ros

#endif  // XTION2_ROS_CAMERA_PUBLISHER_H
