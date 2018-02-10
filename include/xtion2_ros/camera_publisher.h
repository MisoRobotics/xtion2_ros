// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#ifndef XTION2_ROS_CAMERA_PUBLISHER_H
#define XTION2_ROS_CAMERA_PUBLISHER_H

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

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

  ros::NodeHandle nh_;
  ros::Publisher color_image_;
  ros::Publisher depth_image_;

  void publishImage(const cv::Mat img, const std::string& type, ros::Publisher& publisher);

  cv_bridge::CvImage img_bridge_;

public:
  explicit CameraPublisher(const std::string& camera_name);

  void publish(IOInterface& xtion_interface);
};
}  // namespace xtion2_ros

#endif  // XTION2_ROS_CAMERA_PUBLISHER_H
