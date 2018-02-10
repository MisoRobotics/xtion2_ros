// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#ifndef XTION2_ROS_IO_INTERFACE_H
#define XTION2_ROS_IO_INTERFACE_H

#include <array>
#include <vector>

#include <OpenNI.h>

#define MIN_NUM_CHUNKS(data_size, chunk_size) ((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size) (MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

namespace xtion2_ros
{
class IOInterface
{
  static constexpr size_t TEXTURE_SIZE = 512;
  static constexpr size_t MAX_DEPTH = 10000;

  openni::VideoFrameRef depth_frame_;
  openni::VideoFrameRef color_frame_;

  openni::Device& device_;
  openni::VideoStream& depth_stream_;
  openni::VideoStream& color_stream_;
  std::array<openni::VideoStream*, 2> streams_;

  std::array<float, MAX_DEPTH> depth_history_;
  unsigned int texture_map_x_;
  unsigned int texture_map_y_;
  std::vector<openni::RGB888Pixel> texture_map_;
  int width_;
  int height_;

  static std::array<int, 2> initialize_stream(const openni::VideoStream& stream);
  bool initialize_streams();
  void initialize_texture_map();

public:
  IOInterface(openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color);

  bool initialize();
  void display();
};
}  // namespace xtion2_ros

#endif  // XTION2_ROS_IO_INTERFACE_H
