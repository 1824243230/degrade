// SA-LIVO ROS1 entry point. Inherited FAST-LIVO2 code is GPL-2.0-only.
#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try
  {
    LIVMapper mapper(nh, pnh);
    mapper.initializeSubscribersAndPublishers();
    mapper.run();
  }
  catch (const std::exception &error)
  {
    ROS_FATAL("SA-LIVO failed: %s", error.what());
    return 1;
  }
  ros::shutdown();
  return 0;
}
