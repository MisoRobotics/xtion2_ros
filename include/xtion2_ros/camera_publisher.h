// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#ifndef XTION2_ROS_CAMERA_PUBLISHER_H
#define XTION2_ROS_CAMERA_PUBLISHER_H

#include <ros/ros.h>

namespace cv
{
class Mat;
}

namespace xtion2_ros
{
class CameraPublisher
{
  unsigned int counter_;

  ros::NodeHandle nh_;
  ros::Publisher color_image_;

public:
  CameraPublisher();

  void publish(const cv::Mat& color);
};
}  // namespace xtion2_ros

#endif  // XTION2_ROS_CAMERA_PUBLISHER_H
