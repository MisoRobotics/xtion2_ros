// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <xtion2_ros/camera_publisher.h>
#include <xtion2_ros/io_interface.h>

namespace xtion2_ros
{
CameraPublisher::CameraPublisher(const std::string& camera_name)
  : color_image_(nh_.advertise<sensor_msgs::Image>(camera_name + "/image", 1))
  , depth_image_(nh_.advertise<sensor_msgs::Image>(camera_name + "/depth/image", 1))
{
}

void CameraPublisher::publishImage(const cv::Mat img, const std::string& type, ros::Publisher& publisher)
{
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.seq = counter_;
  header.stamp = ros::Time::now();
  img_bridge_ = cv_bridge::CvImage(header, type, img);
  img_bridge_.toImageMsg(img_msg);
  publisher.publish(img_msg);
}

void CameraPublisher::publish(IOInterface& xtion_interface)
{
  if (!xtion_interface.framesValid())
    return;

  ++counter_;
  publishImage(xtion_interface.getColorFrame(), sensor_msgs::image_encodings::RGB8, color_image_);
  publishImage(xtion_interface.getDepthFrame(), sensor_msgs::image_encodings::TYPE_16UC1, depth_image_);
}
}  // namespace xtion2_ros
