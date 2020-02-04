/**********************************************************
 * Author        : Allen
 * Email         : 1059074199@qq.com
 * Last modified : 2019-01-16 14:47
 * File Name      : driver.cpp
 * Description   : EDVS ros drive.
 * *******************************************************/

#include "edvs_ros_simple/driver.h"

EdvsDriver::EdvsDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private)
{
//	caerDeviceHandle edvs_handle
//		= caerDeviceOpenSerial(1, CAER_DEVICE_EDVS, "/dev/ttyUSB0", CAER_HOST_CONFIG_SERIAL_BAUD_RATE_12M);
//	if (edvs_handle == NULL) {
////		return (EXIT_FAILURE);
//	}

//	// Let's take a look at the information we have on the device.
//	struct caer_edvs_info edvs_info = caerEDVSInfoGet(edvs_handle);

//	printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d.\n", edvs_info.deviceString, edvs_info.deviceID,
//		edvs_info.deviceIsMaster, edvs_info.dvsSizeX, edvs_info.dvsSizeY);
		
    bool device_is_running = false;
    while (!device_is_running)
    {

        edvs_handle_ = caerDeviceOpenSerial(1, CAER_DEVICE_EDVS, "/dev/ttyUSB0", CAER_HOST_CONFIG_SERIAL_BAUD_RATE_12M);

        device_is_running = !(edvs_handle_ == NULL);

        if (!device_is_running)
        {
            ROS_WARN("Could not find DVS. Will retry every second.");

            ros::Duration(1.0).sleep();
        }
        else
        {
            caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PR, 695);
            caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_FOLL, 867);
        }
        if (!ros::ok())
        {
            return;
        }
    }

    edvs_info_ = caerEDVSInfoGet(edvs_handle_);

    device_id_ = "EDVS-V1-" + std::string(edvs_info_.deviceString);

    // ROS_INFO("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, serialPortName: %s, serialBaudRate: %d.\n", edvs_info_.deviceString,
    //             edvs_info_.deviceID, edvs_info_.deviceIsMaster, edvs_info_.dvsSizeX, edvs_info_.dvsSizeY,
    //             edvs_info_.serialPortName, edvs_info_.serialBaudRate);
    ROS_INFO("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d.\n", edvs_info_.deviceString,
       edvs_info_.deviceID, edvs_info_.deviceIsMaster, edvs_info_.dvsSizeX, edvs_info_.dvsSizeY);

    delta_ = boost::posix_time::microseconds(1e6/30);
    std::string ns = ros::this_node::getNamespace();

    if (ns == "/")
        ns = "/edvs";

    event_pub_ = nh_.advertise<dvs_msgs::Event>(ns + "/events",1);
    event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>(ns + "/event_array",1);
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(ns + "/camera_info", 1);

// camera info handling
    ros::NodeHandle nh_ns(ns);
    camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh_ns, device_id_);

    resetTimestamps();

    running_ = true;
    // changeEDvsParameters();
    readout_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&EdvsDriver::readout, this)));

    // caerDVSNoiseFilterInit();
}

EdvsDriver::~EdvsDriver()
{
    if (running_)
    {
        ROS_INFO("shutting down threads");
        running_ = false;
        readout_thread_->join();
        ROS_INFO("threads stopped");

        caerDeviceClose(&edvs_handle_);
    }
    // caerFilterDVSNoiseDestroy(state);
}

void EdvsDriver::resetTimestamps()
{
    if (caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_DVS, EDVS_CONFIG_DVS_TIMESTAMP_RESET, 1))
    {
        ROS_INFO("Reset timestamps");
        reset_time =ros::Time::now();
    }
    else
    {
        ROS_ERROR("Failed to reset timestamps");
    }
}

void EdvsDriver::changeEDvsParameters()
{
    // fast
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_CAS, 1992);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_INJGND, 1108364);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REQPD, 16777215);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PUX, 8159221);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFFOFF, 132);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REQ, 309590);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REFR, 969);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PUY, 16777215);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFFON, 209996);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFF, 13125);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_FOLL, 271);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PR, 217);

    // standard
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_CAS, 54);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_INJGND, 1108364);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REQPD, 16777215);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PUX, 8159221);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFFOFF, 132);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REQ, 159147);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REFR, 6);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PUY, 16777215);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFFON, 262144);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFF, 30153);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_FOLL, 51);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PR, 3);

    // slow
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_CAS, 54);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_INJGND, 1108364);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REQPD, 16777215);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PUX, 8159221);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFFOFF, 132);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REQ, 159147);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_REFR, 6);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PUY, 16777215);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFFON, 482443);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_DIFF, 30153);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_FOLL, 51);
    // caerDeviceConfigSet(edvs_handle_, EDVS_CONFIG_BIAS, EDVS_CONFIG_BIAS_PR, 3);


}

void EdvsDriver::readout()
{
    caerDeviceConfigSet(edvs_handle_, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    caerDeviceDataStart(edvs_handle_, NULL, NULL, NULL, NULL, NULL);

    boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();

    dvs_msgs::EventArrayPtr event_array_msg(new dvs_msgs::EventArray());
    // dvs_msgs::EventArray::SharedPtr event_array_msg = std::make_shared<dvs_msgs::EventArray>();
    event_array_msg->height = edvs_info_.dvsSizeY;
    event_array_msg->width = edvs_info_.dvsSizeX;

    while (running_)
    {
        try
        {
            caerEventPacketContainer packetContainer = caerDeviceDataGet(edvs_handle_);
            if (packetContainer == NULL)
            {
                continue;
            }

            int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

            for (int32_t i = 0; i < packetNum; i++)
            {
                caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
                if (packetHeader == NULL)
                {
                    continue;
                }

                const int type = caerEventPacketHeaderGetEventType(packetHeader);

                if (type == POLARITY_EVENT)
                {
                    caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;

                    // caerFilterDVSNoiseApply(state, polarity);

                    // caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_RESET, true);

                    const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);

                    for (int j = 0; j < numEvents; j++)
                    {
                        caerPolarityEvent event = caerPolarityEventPacketGetEvent(polarity, j);

                        dvs_msgs::Event e;
                        e.x = caerPolarityEventGetX(event);
                        e.y = caerPolarityEventGetY(event);
                        e.ts = reset_time +
                        ros::Duration().fromNSec(caerPolarityEventGetTimestamp64(event, polarity) * 1000);

                        e.polarity = caerPolarityEventGetPolarity(event);
                        event_array_msg->events.push_back(e);
                        event_pub_.publish(e);
                    }

                    if (boost::posix_time::microsec_clock::local_time() > next_send_time)
                    {
                        event_array_pub_.publish(event_array_msg);
                        event_array_msg->events.clear();

                        next_send_time += delta_;
                    }

                    if (camera_info_manager_->isCalibrated())
                    {
                        sensor_msgs::CameraInfoPtr camera_info_msg(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
                        camera_info_pub_.publish(camera_info_msg);
                        std::cout<<"K:\n"<<camera_info_msg->K[0]<<"\t"<<camera_info_msg->K[1]<<"\t"<<camera_info_msg->K[2]<<std::endl;
                        std::cout<<camera_info_msg->K[3]<<"\t"<<camera_info_msg->K[4]<<"\t"<<camera_info_msg->K[5]<<std::endl;
                        std::cout<<camera_info_msg->K[6]<<"\t"<<camera_info_msg->K[7]<<"\t"<<camera_info_msg->K[8]<<std::endl;
                        std::cout<<"D:\n"<<camera_info_msg->D[0]<<"\t"<<camera_info_msg->D[1]<<"\t"<<camera_info_msg->D[2]
                        <<"\t"<<camera_info_msg->D[3]<<"\t"<<camera_info_msg->D[4]<<std::endl;
                        // std::cout<<camera_info_msg<<std::endl;
                    }

                }

                caerEventPacketContainerFree(packetContainer);
                // caerEventPacketContainerFree(packetContainer);
            }


        }
        catch(boost::thread_interrupted&)
        {
            return;
        }
    }

    caerDeviceDataStop(edvs_handle_);
}

// void EdvsDriver::caerDVSNoiseFilterInit()
// {
//     state = caerFilterDVSNoiseInitialize(U16T(edvs_info_.dvsSizeY), U16T(edvs_info_.dvsSizeX));

//     if (state == NULL)
//     {
//         ROS_ERROR("Failed to initialize DVS Noise filter.");
//     }

// //     bool caerFilterDVSNoiseConfigSet (caerFilterDVSNoise noiseFilter,uint8_t paramAddr,uint64_t param )
//     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_HOTPIXEL_TIME, U32T(1000000) );
//     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_HOTPIXEL_COUNT, U32T(10000) );
//     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_HOTPIXEL_ENABLE, false );
// //     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_BACKGROUND_ACTIVITY_ENABLE,true);
// //     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TWO_LEVELS,false);
// //     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_BACKGROUND_ACTIVITY_CHECK_POLARITY,false);
// //     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_BACKGROUND_ACTIVITY_SUPPORT_MIN,1);
// //     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_BACKGROUND_ACTIVITY_SUPPORT_MAX,8);
// //     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TIME,2000);
//     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_REFRACTORY_PERIOD_ENABLE,true);
//     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_REFRACTORY_PERIOD_TIME,100);
// //     caerFilterDVSNoiseConfigSet(state, CAER_FILTER_DVS_LOG_LEVEL, 5);

// }
