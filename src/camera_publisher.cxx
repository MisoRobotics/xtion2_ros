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
  : color_nh_("rgb")
  , depth_nh_("depth")
  , color_it_(color_nh_)
  , depth_it_(depth_nh_)
  , color_manager_(color_nh_)
  , depth_manager_(depth_nh_)
  , color_pub_(color_it_.advertise("image_raw", 1))
  , depth_pub_(depth_it_.advertise("image_raw", 1))
{
}

void CameraPublisher::publishImage(const cv::Mat img, const std::string& type, image_transport::Publisher& publisher)
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

  if (xtion_interface.color_new)
  {
    publishImage(xtion_interface.getColorFrame(), sensor_msgs::image_encodings::RGB8, color_pub_);
    xtion_interface.color_new = false;
  }
  if (xtion_interface.depth_new)
  {
    publishImage(xtion_interface.getDepthFrame(), sensor_msgs::image_encodings::TYPE_16UC1, depth_pub_);
    xtion_interface.depth_new = false;
  }
}
}  // namespace xtion2_ros
