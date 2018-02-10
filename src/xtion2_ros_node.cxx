#include <OpenNI.h>

#include <ros/ros.h>
#include <xtion2_ros/io_interface.h>

int main(int argc, char** argv)
{
  openni::Status rc = openni::STATUS_OK;
  openni::Device device;
  openni::VideoStream depth, color;
  const char* device_uri = openni::ANY_DEVICE;
  if (argc > 1)
  {
    device_uri = argv[1];
  }

  rc = openni::OpenNI::initialize();

  ROS_WARN("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

  rc = device.open(device_uri);
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
  iface.initialize();

  for (int i = 0; i < 5; ++i)
  {
    ROS_INFO_STREAM("i: " << i);
    iface.display();
  }

  openni::OpenNI::shutdown();
  return 0;
}
