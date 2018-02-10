// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#ifndef XTION2_ROS_IO_INTERFACE_H
#define XTION2_ROS_IO_INTERFACE_H

#include <array>
#include <vector>

#include <opencv2/highgui.hpp>

#include <OpenNI.h>

namespace xtion2_ros
{
class IOInterface
{
  openni::VideoFrameRef depth_frame_;
  openni::VideoFrameRef color_frame_;

  openni::Device& device_;
  openni::VideoStream& depth_stream_;
  openni::VideoStream& color_stream_;
  std::array<openni::VideoStream*, 2> streams_;

  int width_;
  int height_;

  static std::array<int, 2> initialize_stream(openni::VideoStream& stream);

  const cv::Mat getFrame(openni::VideoFrameRef& frame, const int data_type);

public:
  bool color_new;
  bool depth_new;

  IOInterface(openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color);

  bool initialize();
  void spinOnce();

  inline int getWidth() const
  {
    return width_;
  }

  inline int getHeight() const
  {
    return height_;
  }

  inline const cv::Mat getColorFrame()
  {
    return getFrame(color_frame_, CV_8UC3);
  }

  inline const cv::Mat getDepthFrame()
  {
    return getFrame(depth_frame_, CV_16U);
  }

  inline bool framesValid()
  {
    return color_frame_.isValid() && depth_frame_.isValid();
  }
};
}  // namespace xtion2_ros

#endif  // XTION2_ROS_IO_INTERFACE_H
