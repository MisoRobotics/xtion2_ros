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
  : camera_name_(camera_name)
  , color_nh_(camera_name + "/rgb")
  , depth_nh_(camera_name + "/depth")
  , color_it_(color_nh_)
  , depth_it_(depth_nh_)
  , color_manager_(color_nh_)
  , depth_manager_(depth_nh_)
  , color_pub_(color_it_.advertise("image_raw", 1))
  , depth_pub_(depth_it_.advertise("image_raw", 1))
  , color_info_pub_(color_nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1))
  , depth_info_pub_(depth_nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1))
{
}

void CameraPublisher::initialize()
{
  if (!color_manager_.setCameraName(camera_name_))
  {
    ROS_WARN_STREAM("[" << camera_name_ << "] name not valid"
                        << " for camera_info_manger");
  }
  if (!depth_manager_.setCameraName(camera_name_))
  {
    ROS_WARN_STREAM("[" << camera_name_ << "] name not valid"
                        << " for camera_info_manger");
  }
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

void CameraPublisher::publishInfo(sensor_msgs::CameraInfo& msg, ros::Publisher& publisher, const std::string& type)
{
  msg.header.seq = counter_;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = camera_name_ + "/" + type + "/optical_frame";
  msg.height = 240;
  msg.width = 320;
  msg.distortion_model = "plumb_bob";
  msg.D = { { 0., 0., 0., 0., 0. } };
  msg.K = { { 831.3820367867805, 0.0, 480.5, 0.0, 831.3820367867805, 270.5, 0.0, 0.0, 1.0 } };
  msg.R = { { 0., 0., 0., 0., 0., 0., 0., 0., 0. } };
  msg.P = { { 831.3820367867805, 0.0, 480.5, -0.0, 0.0, 831.3820367867805, 270.5, 0.0, 0.0, 0.0, 1.0, 0.0 } };
  publisher.publish(msg);
}

void CameraPublisher::publish(IOInterface& xtion_interface)
{
  if (!xtion_interface.framesValid())
    return;

  ++counter_;

  if (xtion_interface.color_new)
  {
    publishImage(xtion_interface.getColorFrame(), sensor_msgs::image_encodings::RGB8, color_pub_);
    publishInfo(color_msg_, color_info_pub_, "rgb");
    xtion_interface.color_new = false;
  }
  if (xtion_interface.depth_new)
  {
    publishImage(xtion_interface.getDepthFrame(), sensor_msgs::image_encodings::TYPE_16UC1, depth_pub_);
    publishInfo(depth_msg_, depth_info_pub_, "depth");
    xtion_interface.depth_new = false;
  }
}
}  // namespace xtion2_ros
