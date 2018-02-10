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

bool IOInterface::initialize_streams()
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

  return true;
}

void IOInterface::initialize_texture_map()
{
  texture_map_x_ = MIN_CHUNKS_SIZE(width_, TEXTURE_SIZE);
  texture_map_y_ = MIN_CHUNKS_SIZE(height_, TEXTURE_SIZE);
  texture_map_ = std::vector<openni::RGB888Pixel>(texture_map_x_ * texture_map_y_);
}

bool IOInterface::initialize()
{
  if (!initialize_streams())
    return false;

  initialize_texture_map();

  return true;
}

void IOInterface::display()
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
      break;
    default:
      ROS_ERROR("Failed waiting for next frame.");
  }
}
}
