/**********************************************************
 * Author        : Jianhong XU
 * Email         : jianhongxu1006@gmail.com
 * Last modified : 2020年01月13日 星期一 21时21分48秒 
 * Description   : 对edvs数据滤波、聚类处理、用于测量电机轴转速
**********************************************************/

#include "clustering/clustering.h"


Clustering::Clustering(ros::NodeHandle & nh, ros::NodeHandle nh_private)
{
	std::cout << "clustering node runs." <<std::endl;
	
	EventArray_sub_ = nh.subscribe("/edvs/event_array", 1, &Clustering::Clustering_generateCb, this);
	Image_pub_ = nh.advertise<sensor_msgs::Image>("/dvs/clustering_image", 1, true); 
	
	rect_xy.x_min = 128;
	rect_xy.x_max = 0;
	rect_xy.y_min = 128;
	rect_xy.y_max = 0;
	
	detect_vec = cv::Vec3f(1,1,0);
	
	//fout.open("/home/allen/data/area.txt");
	
}

Clustering:: ~Clustering()
{
	//fout.close();
}

void Clustering::Clustering_generateCb(const dvs_msgs::EventArray::ConstPtr& msg)
{
    struct event_xy event_key
    {
       0,0
    };
        
    struct Ts_pol event_value
    {
        0, false
    };
    
    event_map.clear();

    for(int i = 0;i < msg->events.size(); i++)
    {
    	event_key.x = msg->events[i].x;
        event_key.y = msg->events[i].y;
        event_value.ts = msg->events[i].ts.toNSec();
        event_value.polarity = msg->events[i].polarity;
        	
        event_map[event_key] = event_value;
     }
     
     int num_pre = event_map.size();
     for(auto i:event_map)
     {
        int num_points = 0;
        for(int j = (i.first.x - 1); j <= (i.first.x +1); j++)
        {
            for(int k = (i.first.y -1); k <= (i.first.y +1); k++)
            {
                event_key.x = j;
                event_key.y = k;
                if(event_map.find(event_key) != event_map.end())
                {
                    num_points++;
                }
            }
        }
        event_key.x = i.first.x;
        event_key.y = i.first.y;
        if(num_points <= 2)
        {
            event_map.erase(event_key);
        }
     }
     int num_now = event_map.size();
//     std::cout << "滤波前点的数量：" << num_pre << "\t滤波后点的数量：" << num_now << "\t滤去点的数量：" 
//     	<< (num_pre - num_now)<<std::endl;
    
    std::map<event_xy, Ts_pol>::iterator it;
    it = event_map.begin();
    struct Rect_xy area_xy;
    area_xy.x_min = 128;
    area_xy.x_max = 0;
	area_xy.y_min = 128;
	area_xy.y_max = 0;
	
	double x_average = 0, y_average = 0;

	int positive_num = 0;
	for(auto i:event_map)
	{
		event_key.x = i.first.x;
		event_key.y = i.first.y;
		if(event_accumulation_map.find(event_key) != event_accumulation_map.end())
		{
			event_accumulation_map[event_key]++;
		}	
		else
		{
			event_accumulation_map[event_key] = 0;
		}
		if(i.second.polarity == true)
		{
			x_average += i.first.x;
			y_average += i.first.y;
			positive_num++;
		}
	}
	
	x_average = x_average / positive_num;
	y_average = y_average / positive_num;
	
	//搜索旋转的区域最大最小值
	if(generate_num > 150)
	{
		generate_num = 0;
		generate_num_2++;
		for(auto i : event_accumulation_map)
		{
			if(i.second > 15)
			{
				if(rect_xy.x_min > i.first.x)
				{
					rect_xy.x_min = i.first.x;
				}
				if(rect_xy.x_max < i.first.x)
				{
					rect_xy.x_max = i.first.x;
				}
				if(rect_xy.y_min > i.first.y)
				{
					rect_xy.y_min = i.first.y;
				}
				if(rect_xy.y_max < i.first.y)
				{
					rect_xy.y_max = i.first.y;
				}
				
				//fout << i.first.x << "\t" << i.first.y << "\t" << i.second << std::endl;
			}
		}
		event_accumulation_map.clear();
	}
	
	//更新旋转区域
	if(generate_num_2 > 3)
	{
		generate_num_2 = 0;
		rect_xy.x_min = 128;
		rect_xy.x_max = 0;
		rect_xy.y_min = 128;
		rect_xy.y_max = 0;
		
		detect_num = 0;
	}
	
    // 绘制滤波后的渲染
	cv_bridge::CvImage cv_image;
	if(msg->events.size() > 0)
	{
		cv_image.header.stamp = msg->events[msg->events.size() / 2].ts;
	}
	cv_image.encoding = "bgr8";
	
	cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
    cv_image.image = cv::Scalar(0,0,0);

    for(auto i:event_map)
    {
    	const int x = i.first.x;
    	const int y = i.first.y;

      cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = 
      		(i.second.polarity == true ? cv::Vec3b(180, 0, 0) : cv::Vec3b(0, 0, 180) );
    }
    
		//绘制旋转区域
//	std::cout << "x(min): " << rect_xy.x_min << "\t" << "x(max): " << rect_xy.x_max << "\t" << "y(min): " << rect_xy.y_min
//		 << "\t" << "y(max): " << rect_xy.y_max << std::endl; 
	
	if((rect_xy.x_max > rect_xy.x_min) && (rect_xy.y_max > rect_xy.y_min))
	{
		detect_area = cv::Rect(rect_xy.x_min, rect_xy.y_min, (rect_xy.x_max - rect_xy.x_min), 
			(rect_xy.y_max - rect_xy.y_min));
		
		if(abs(detect_area.width - detect_area.height) < 10)
		{
			//绘制旋转区域
			cv::rectangle(cv_image.image, detect_area, cv::Scalar(0,255,0), 1, 1, 0);
			
			//绘制旋转中心点
			rotation_center =  cv::Point2f((detect_area.x + detect_area.width/2), (detect_area.y + detect_area.height/2));
			cv::circle(cv_image.image, rotation_center, 1, cv::Scalar(0, 255, 0), 0);
			
			//绘制旋转中心点与中值点的向量连线
			cv::line(cv_image.image, rotation_center, cv::Point2f(x_average, y_average), cv::Scalar(255,255,255));
			
			//旋转向量
			rotation_vec = cv::Vec3f((x_average - rotation_center.x), (y_average - rotation_center.y), 0);
			//std::cout << rotation_vec <<std::endl;
			

			double rotation_angle = get_vector_angle(rotation_vec, rotation_vec_last);
			rotation_vec_last = rotation_vec;
			if(rotation_angle != 0)
			{
				rotation_angle_average_1 += rotation_angle;
				vec_num++;
			}
			if(vec_num == 10)
			{
				rotation_angle_average_1 = rotation_angle_average_1 / vec_num;
				rotation_angle_average_2 = rotation_angle_average_1;
				rotation_angle_average_1 = 0;
				vec_num = 0;
			}
			
			cv::Vec3f Vn = rotation_vec.cross(detect_vec);  //旋转向量与检测向量(1,1,0)的叉积为相似度向量

			if(rotation_angle_average_2 != 0)
			{
//				std::cout << (rotation_angle_average_2 / 2) << std::endl;
				if(get_vector_norm(Vn) < ( get_vector_norm(rotation_vec) * sin(rotation_angle_average_2 / 2)))
				{
//					std::cout << "相似度Vn: " << Vn << std::endl;
					if(detect_num == 0)
					{
						last_time = msg->events[msg->events.size()/ 2].ts;
					}
					detect_num++;
					if(last_if)
					{
						last_time = msg->events[msg->events.size()/ 2].ts;
						last_if = false;
					}
				}	
			}
			
			if(detect_num >= 10)
			{
				detect_num = 0;
				ros::Duration delta_t = msg->events[msg->events.size() / 2].ts - last_time;
				rotation_velocity = 1.0 * 60 * 5 / (delta_t.toSec());
				//std::cout << "时间间隔delta: " << delta_t <<std::endl;
								
				rotation_velocity = rotation_velocity * (1 + sin(rotation_angle_average_2 / 2));
				
				std::cout << "电机转速: " << rotation_velocity << std::endl;
				last_time = msg->events[msg->events.size() / 2].ts;
				
				rotation_velocity_aver += rotation_velocity;
				vel_num++;
				if(vel_num == 10)
				{
					rotation_velocity_aver = rotation_velocity_aver / vel_num;
					std:: cout << "平均转速：" << rotation_velocity_aver << std::endl;
					rotation_velocity_aver = 0;
					vel_num = 0;
				}
			}
		}
	} 
	  
    Image_pub_.publish(cv_image.toImageMsg());
    
    generate_num++;
    
}

/*
 * name: get_vector_angle
 * @param: (cv::Vec3f a, cv::Vec3f b)
 * @return: 返回两个空间向量的夹角
 */
double get_vector_angle(cv::Vec3f a, cv::Vec3f b)
{
	double xita = acos(a.dot(b) / (get_vector_norm(a) * get_vector_norm(b)));
	return abs(xita);
}


/*
 * name: get_vector_norm
 * @param: (cv::Vec3f a)
 * @return: 返回向量的模长
 */
double get_vector_norm(cv::Vec3f a)
{
	return sqrt(a(0)*a(0) + a(1)*a(1) + a(2)*a(2));
}
