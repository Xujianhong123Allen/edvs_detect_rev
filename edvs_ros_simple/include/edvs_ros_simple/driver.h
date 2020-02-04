/**********************************************************
 * Author        : Allen
 * Email         : 1059074199@qq.com
 * Last modified : 2019-02-23 18:45
 * File Name      : driver.h
 * Description   : 
 * *******************************************************/

#include <libcaer/devices/edvs.h>
#include <libcaer/devices/serial.h>
#include <libcaer/libcaer.h>
#include <libcaer/filters/dvs_noise.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <chrono>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Time.h>
#include <boost/thread.hpp>
#include <thread>
#include <string>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;

class EdvsDriver
{
private:
    ros::NodeHandle nh_;
    caerDeviceHandle edvs_handle_;
    ros::Time reset_time;

    boost::posix_time::time_duration delta_;

    ros::Publisher event_pub_;
    ros::Publisher event_array_pub_;
    ros::Publisher camera_info_pub_;
    volatile bool running_;

    std::shared_ptr<std::thread> readout_thread_;
    void readout();
    void resetTimestamps();

    void changeEDvsParameters();

    camera_info_manager::CameraInfoManager* camera_info_manager_;

    std::string device_id_;
    
    // caerFilterDVSNoise state ;

    // void caerDVSNoiseFilterInit();
    
    struct caer_edvs_info edvs_info_;
public:
    EdvsDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private);
    ~EdvsDriver();
};
