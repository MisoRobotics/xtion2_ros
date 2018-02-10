// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <xtion2_ros/camera_publisher.h>

namespace xtion2_ros
{
CameraPublisher::CameraPublisher() : color_image_(nh_.advertise<sensor_msgs::Image>("/color", 1))
{
}

void CameraPublisher::publish(const cv::Mat& img)
{
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;  // >> message to be sent

  std_msgs::Header header;
  header.seq = counter_++;
  header.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
  img_bridge.toImageMsg(img_msg);  // from cv_bridge to sensor_msgs::Image
  color_image_.publish(img_msg);
}
}  // namespace xtion2_ros
