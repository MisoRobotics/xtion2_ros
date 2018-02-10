// Copyright 2018 Miso Robotics, Inc.
// Author: Ryan W. Sinnet (ryan@rwsinnet.com)

#include <OpenNI.h>

#include <ros/ros.h>
#include <xtion2_ros/camera_publisher.h>
#include <xtion2_ros/io_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xtion2_ros_node");
  ros::start();

  std::string camera_name;
  ros::param::param("camera_name", camera_name, std::string("xtion2"));

  std::string device_uri;
  ros::param::param("device_uri", device_uri, std::string());

  double fps;
  ros::param::param("fps", fps, 30.);

  openni::Status rc = openni::STATUS_OK;
  openni::Device device;
  openni::VideoStream depth, color;

  rc = openni::OpenNI::initialize();

  ROS_WARN("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

  rc = device.open(openni::ANY_DEVICE);
  if (rc != openni::STATUS_OK)
  {
    ROS_ERROR("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
    openni::OpenNI::shutdown();
    return 1;
  }

  rc = depth.create(device, openni::SENSOR_DEPTH);
  if (rc == openni::STATUS_OK)
  {
    rc = depth.start();
    if (rc != openni::STATUS_OK)
    {
      ROS_WARN("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
      depth.destroy();
    }
  }
  else
  {
    ROS_WARN("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
  }

  rc = color.create(device, openni::SENSOR_COLOR);
  if (rc == openni::STATUS_OK)
  {
    rc = color.start();
    if (rc != openni::STATUS_OK)
    {
      ROS_WARN("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
      color.destroy();
    }
  }
  else
  {
    ROS_WARN("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
  }

  if (!depth.isValid() || !color.isValid())
  {
    ROS_ERROR("SimpleViewer: No valid streams. Exiting\n");
    openni::OpenNI::shutdown();
    return 2;
  }

  xtion2_ros::IOInterface iface(device, depth, color);
  xtion2_ros::CameraPublisher publisher(camera_name);
  iface.initialize();

  ROS_INFO("Connected to Xtion2 with resolution %dx%d.", iface.getWidth(), iface.getHeight());

  auto i = 0;
  ros::Rate r(fps);
  while (ros::ok())
  {
    iface.spinOnce();
    publisher.publish(iface);
    r.sleep();
  }

  openni::OpenNI::shutdown();
  return 0;
}
