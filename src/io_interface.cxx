// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#include <opencv2/highgui.hpp>

#include <xtion2_ros/io_interface.h>
#include <ros/ros.h>

namespace xtion2_ros
{
IOInterface::IOInterface(openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color)
  : device_(device), depth_stream_(depth), color_stream_(color)
{
}

std::array<int, 2> IOInterface::initialize_stream(const openni::VideoStream& stream)
{
  auto video_mode = stream.getVideoMode();
  return { { video_mode.getResolutionX(), video_mode.getResolutionY() } };
}

void IOInterface::initialize_texture_map()
{
  texture_map_x_ = MIN_CHUNKS_SIZE(width_, TEXTURE_SIZE);
  texture_map_y_ = MIN_CHUNKS_SIZE(height_, TEXTURE_SIZE);
  texture_map_ = std::vector<openni::RGB888Pixel>(texture_map_x_ * texture_map_y_);
}

bool IOInterface::initialize()
{
  const auto depth_resolution = initialize_stream(depth_stream_);
  if (!depth_stream_.isValid())
  {
    ROS_ERROR("Depth stream was invalid.");
    return false;
  }

  const auto color_resolution = initialize_stream(color_stream_);
  initialize_stream(color_stream_);
  if (!color_stream_.isValid())
  {
    ROS_ERROR("Color stream was invalid.");
    return false;
  }

  if (color_resolution != depth_resolution)
  {
    ROS_ERROR("Expect color and depth to be in same resolution: D: %dx%d, C: %dx%d", depth_resolution[0],
              depth_resolution[1], color_resolution[0], color_resolution[1]);
    return false;
  }

  width_ = color_resolution[0];
  height_ = color_resolution[1];

  streams_[0] = &depth_stream_;
  streams_[1] = &color_stream_;

  initialize_texture_map();

  return true;
}

void IOInterface::convertColorFrame()
{
  // In a show of poor form, const_cast the data to avoid a memcpy. Hopefully no one tries to use them afterward.
  cv::Mat mat(color_frame_.getHeight(), color_frame_.getWidth(), CV_8UC3, const_cast<void*>(color_frame_.getData()));
  publisher_.publish(mat);
}

void IOInterface::spinOnce()
{
  int changedIndex;
  openni::Status rc = openni::OpenNI::waitForAnyStream(streams_.data(), 2, &changedIndex);
  if (rc != openni::STATUS_OK)
  {
    ROS_ERROR("Wait failed");
    return;
  }

  switch (changedIndex)
  {
    case 0:
      depth_stream_.readFrame(&depth_frame_);
      break;
    case 1:
      color_stream_.readFrame(&color_frame_);
      convertColorFrame();
      break;
    default:
      ROS_ERROR("Failed waiting for next frame.");
  }
}
}
