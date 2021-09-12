

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
