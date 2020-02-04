/**********************************************************
 * Author        : Allen
 * Email         : 1059074199@qq.com
 * Last modified : 2019-02-23 18:46
 * File Name      : driver_node.cpp
 * Description   : 
 * *******************************************************/

#include <ros/ros.h>
#include "edvs_ros_simple/driver.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv,"edvs_ros_simple");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  EdvsDriver* driver = new EdvsDriver(nh, nh_private);
  ros::spin();
  ros::shutdown();
  return 0;
}
