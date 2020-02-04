/**********************************************************
 * Author        : Allen
 * Email         : 1059074199@qq.com
 * Last modified : 2019-02-23 18:46
 * File Name      : driver_node.cpp
 * Description   : 
 * *******************************************************/

#include <ros/ros.h>
#include "clustering/clustering.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv,"clustering");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

	Clustering* cluster= new Clustering(nh, nh_private);
  ros::spin();
  ros::shutdown();
  return 0;
}
