/**********************************************************
 * Author        : Jianhong XU
 * Email         : jianhongxu1006@gmail.com
 * Last modified : 2020年01月13日 星期一 21时21分48秒 
 * Description   : 对edvs数据滤波、聚类处理、用于测量电机轴转速
**********************************************************/

#include "clustering/clustering.h"

Clustering::Clustering(ros::NodeHandle &nh, ros::NodeHandle nh_private)
{
	std::cout << "clustering node runs." << std::endl;

	// EventArray_sub_ = nh.subscribe("/edvs/event_array", 1, &Clustering::Clustering_generateCb, this);
	EventArray_sub_ = nh.subscribe("/dvs/events", 1, &Clustering::Clustering_generateCb, this);
	Image_pub_ = nh.advertise<sensor_msgs::Image>("/dvs/clustering_image", 1, true);
	Rev_pub_ = nh.advertise<std_msgs::Float32>("/dvs/rev_detect", 1, true);

	rect_xy.x_min = 128;
	rect_xy.x_max = 0;
	rect_xy.y_min = 128;
	rect_xy.y_max = 0;

	detect_vec = cv::Vec3f(1, 1, 0);

	// fout.open("/home/jh/area.txt");
}

Clustering::~Clustering()
{
	// fout.close();
}

void Clustering::Clustering_generateCb(const dvs_msgs::EventArray::ConstPtr &msg)
{
	// std::cout << "generate" << std::endl;
	struct event_xy event_key
	{
		0, 0
	};

	struct Ts_pol event_value
	{
		0, false
	};

	event_map.clear();

	for (int i = 0; i < msg->events.size(); i++)
	{
		event_key.x = msg->events[i].x;
		event_key.y = msg->events[i].y;
		event_value.ts = msg->events[i].ts.toNSec();
		event_value.polarity = msg->events[i].polarity;

		event_map[event_key] = event_value;
	}

	//滤波：将多与的噪声数据滤去
	int num_pre = event_map.size();
	std::map<event_xy, Ts_pol>::iterator it;
	for (it = event_map.begin(); it != event_map.end();)
	{
		int num_points = 0;
		for (uint j = it->first.x - 1; j <= (it->first.x + 2); j++)
		{
			for (uint k = it->first.y - 1; k <= (it->first.y + 2); k++)
			{
				event_key.x = j;
				event_key.y = k;
				// if ((event_map.find(event_key) != event_map.end()) && it->second.polarity == true)
				if (event_map.find(event_key) != event_map.end())
				{

					num_points++;
				}
			}
		}
		event_key.x = it->first.x;
		event_key.y = it->first.y;
		if (num_points <= filtering_area_point_num)
		{
			event_map.erase(it++);
		}
		else
		{
			it++;
		}
	}
	int num_now = event_map.size();
	// std::cout << "滤波前点的数量：" << num_pre << "\t滤波后点的数量：" << num_now << "\t滤去点的数量：" << (num_pre - num_now) << std::endl;
	num_total = num_total + num_now;

	struct Rect_xy area_xy;
	area_xy.x_min = 128;
	area_xy.x_max = 0;
	area_xy.y_min = 128;
	area_xy.y_max = 0;

	double x_average = 0, y_average = 0;

	int positive_num = 0;
	for (auto i : event_map)
	{
		event_key.x = i.first.x;
		event_key.y = i.first.y;
		if (event_accumulation_map.find(event_key) != event_accumulation_map.end())
		{
			event_accumulation_map[event_key]++;
		}
		else
		{
			event_accumulation_map[event_key] = 0;
		}
		// if (i.second.polarity == true)
		// {
		// 	x_average += i.first.x;
		// 	y_average += i.first.y;
		// 	positive_num++;
		// }
		x_average += i.first.x;
		y_average += i.first.y;
		positive_num++;
	}

	x_average = x_average / positive_num;
	y_average = y_average / positive_num;

	//搜索旋转的区域最大最小值,generate_num为处理图像帧的次数
	if (count_points >= 10)
	{
		num_total = num_total / count_points;
		if (num_total < 200)
		{
			rev_class = 50;
		}
		else if (num_total < 500)
		{
			rev_class = 100;
		}
		else if (num_total < 1000)
		{
			rev_class = 200;
		}
		else
		{
			rev_class = 300;
		}
		count_points = 0;
		num_total = 0;
		// std::cout << "rev_class: " << rev_class << std::endl;
	}
	if (generate_num > rev_class)
	{
		generate_num = 0;
		generate_num_2++;
		// if (area_once == true)
		// {
		// 	for (auto i : event_accumulation_map)
		// 	{

		// 		if (i.second > 1)
		// 		{
		// 			fout << i.first.x << "\t" << i.first.y << "\t" << i.second << std::endl;
		// 		}
		// 	}
		// 	area_once = false;
		// }

		for (auto i : event_accumulation_map)
		{
			if (i.second > (rev_class / 20))
			{
				if (rect_xy.x_min > i.first.x)
				{
					rect_xy.x_min = i.first.x;
				}
				if (rect_xy.x_max < i.first.x)
				{
					rect_xy.x_max = i.first.x;
				}
				if (rect_xy.y_min > i.first.y)
				{
					rect_xy.y_min = i.first.y;
				}
				if (rect_xy.y_max < i.first.y)
				{
					rect_xy.y_max = i.first.y;
				}
			}
		}
		event_accumulation_map.clear();
	}

	//更新旋转区域
	if (generate_num_2 > 20)
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
	if (msg->events.size() > 0)
	{
		cv_image.header.stamp = msg->events[msg->events.size() / 2].ts;
	}
	cv_image.encoding = "bgr8";

	cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
	cv_image.image = cv::Scalar(0, 0, 0);

	for (auto i : event_map)
	{
		const int x = i.first.x;
		const int y = i.first.y;

		cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) =
			(i.second.polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
	}

	//绘制旋转区域
	//	std::cout << "旋转区域：\tx(min): " << rect_xy.x_min << "\tx(max): " << rect_xy.x_max << "\t" << "y(min): " << rect_xy.y_min
	//		 << "\t" << "y(max): " << rect_xy.y_max << std::endl;
	if ((rect_xy.x_max > rect_xy.x_min) && (rect_xy.y_max > rect_xy.y_min))
	{
		detect_area = cv::Rect(rect_xy.x_min, rect_xy.y_min, (rect_xy.x_max - rect_xy.x_min), (rect_xy.y_max - rect_xy.y_min));

		if (abs(detect_area.width - detect_area.height) < 10)
		{
			//找到旋转中心（即极坐标下的原点）
			rotation_center = cv::Point2f((detect_area.x + detect_area.width / 2), (detect_area.y + detect_area.height / 2));

			double polar_angle_average = 0;
			int num_point_single_edge = 0;

			double theta_max = 0, theta_min = pi;
			int horizont_num = 0;
			for (auto i : event_map)
			{
				//计算每一个事件的极角theta，极径rhos
				double theta, rhos, temp;
				rhos = sqrt((i.first.x - rotation_center.x) * (i.first.x - rotation_center.x) +
							(i.first.y - rotation_center.y) * (i.first.y - rotation_center.y));
				temp = acos(((i.first.x - rotation_center.x) * 1) / rhos);

				if (i.first.y > rotation_center.y)
				{
					theta = temp;
					if (theta_max < theta)
					{
						theta_max = theta;
					}
					if (theta_min > theta)
					{
						theta_min = theta;
					}

					if (fabs(pi / 2 - temp) < pi / 3)
					{
						// if (i.first.x > rotation_center.x)
						// {
						// 	num_point_single_edge++;
						// 	polar_angle_average += temp;
						// }
						num_point_single_edge++;
						polar_angle_average += temp;
					}
					// else
					// {
					// 	num_point_single_edge++;
					// 	polar_angle_average += temp;
					// }
				}
				else
				{
					theta = 2 * pi - temp;
				}

				i.second.polar_radius = rhos;
				i.second.polar_angle = theta;

				//当旋转向量接近0度和180度时，不检测
				if (((theta > 0) && (theta < pi / 6)) || ((theta > 5.0 * pi / 6.0) && (theta < 7.0 * pi / 6)) || ((theta > 11.0 * pi / 6.0) && (theta < 2 * pi)))
				{
					horizont_num++;
				}
			}

			//判断并设置运动模糊残影角度
			if (fabs(theta_max - theta_min) < pi / 2)
			{
				bulr_angle = fabs(theta_max - theta_min);
				// std::cout << "运动残影角：" << bulr_angle * 180 / pi << std::endl;
			}

			//估计出平均极角
			// std::cout << "horizont_num: " << horizont_num << "\t" << event_map.size() << std::endl;
			if (horizont_num < 1.0 / 4.0 * event_map.size())
			{
				polar_angle_average = polar_angle_average / num_point_single_edge;
				// polar_angle_average = polar_angle_average + bulr_angle * 0.1;
				// std::cout << "平均极角（度）：" << polar_angle_average * 180 / pi << std::endl;
			}
			else
			{
				polar_angle_average = 0;
			}

			//绘制旋转区域
			cv::rectangle(cv_image.image, detect_area, cv::Scalar(0, 255, 0), 1, 1, 0);

			//绘制旋转中心点

			cv::circle(cv_image.image, rotation_center, 2, cv::Scalar(0, 255, 0), -1);

			//绘制旋转中心点与中值点的向量连线，以旋转半径为画出旋转向量
			if (polar_angle_average != 0)
			{
				cv::line(cv_image.image, rotation_center, cv::Point2f(rotation_center.x + (detect_area.height / 2 * cos(polar_angle_average)), rotation_center.y + (detect_area.height / 2 * sin(polar_angle_average))), cv::Scalar(0, 255, 0));
				cv::line(cv_image.image, rotation_center, cv::Point2f(rotation_center.x - (detect_area.height / 2 * cos(polar_angle_average)), rotation_center.y - (detect_area.height / 2 * sin(polar_angle_average))), cv::Scalar(0, 255, 0));
			}

			//每次采样极径所转过的角度之差，这里因为在opencv图像中，y轴向下，逆时针转动的角度差为负值，需要前一次减去后一次的角度值才为正。
			double rotation_angle = polar_angle_last - polar_angle_average;
			// std::cout << "rotation_angle: " << rotation_angle * 180 / pi << std::endl;
			polar_angle_last = polar_angle_average;
			if (rotation_angle > 0 && rotation_angle < pi / 3)
			{
				rotation_angle_average_1 += rotation_angle;
				vec_num++;
			}
			if (vec_num == 10)
			{
				rotation_angle_average_1 = rotation_angle_average_1 / vec_num;
				rotation_angle_average_2 = rotation_angle_average_1;
				// std::cout << "平均转角：" << rotation_angle_average_1 * 180/pi << std::endl;
				std::cout << rotation_angle_average_1 * 180/pi << std::endl;
				rotation_angle_average_1 = 0;
				vec_num = 0;
			}

			// polar_angle_current = polar_angle_average;
			// double delta_theta = polar_angle_current - polar_angle_last;
			// polar_angle_last = polar_angle_current;

			// 检测的极角为90度
			// std::cout << "运动模糊角：" << bulr_angle * 180/pi << "\t平均转角：" << rotation_angle_average_2 * 180/pi << std::endl;
			// if (fabs(polar_angle_average - pi / 2.0) <= bulr_angle * (0.21 * rotation_angle_average_2 / bulr_angle))
			// std::cout << fabs(polar_angle_average - pi / 2.0) * 180 / pi << "\t" << (1.0 * rotation_angle_average_2 * 180 / pi ) << std::endl;
			double rev_parm = 0.93;
			// if()
			// {
			// 	rev_parm = 0.5;
			// }
			// else if()
			// {
			// 	rev_parm = 0.93;
			// }
			// if (fabs(polar_angle_average - pi / 2.0) <= (0.47 * rotation_angle_average_2))
			if (fabs(polar_angle_average - pi / 2.0) <= (0.45 * 0.62577))
			{
				// if (detect_num == 1)
				// {
				// 	double delta_theta = polar_angle_average - polar_angle_last_cal;
				// 	std::cout << "delta_theta: " << delta_theta << std::endl;
				// 	ros::Duration delta_t = msg->events[msg->events.size() / 2].ts - last_time;
				// 	std::cout << "时间间隔delta: " << delta_t << std::endl;
				// 	rotation_velocity = (1.0 * 60 * delta_theta / (delta_t.toSec() * 2 * pi));
				// 	std::cout << "转速：" << rotation_velocity << std::endl;
				// 	detect_num = 0;
				// }

				// if (detect_num == 0)
				// {
				// 	last_time = msg->events[msg->events.size() / 2].ts;
				// 	polar_angle_last = polar_angle_average;
				// }

				// detect_num++;

				if (detect_num == 0)
				{
					last_time = msg->events[msg->events.size() / 2].ts;
				}
				if (last_if == true)
				{
					last_time = msg->events[msg->events.size() / 2].ts;
					last_if = false;
				}
				detect_num++;
			}

			if (detect_num >= 20)
			{
				detect_num = 0;
				ros::Duration delta_t = msg->events[msg->events.size() / 2].ts - last_time;
				// std::cout << "时间间隔delta: " << delta_t << std::endl;
				rotation_velocity = (1.0 * 60 * 10 / (delta_t.toSec()));
				// std::cout << rotation_velocity << std::endl;

				last_time = msg->events[msg->events.size() / 2].ts;

				vel_num++;
				rotation_velocity_aver += rotation_velocity;

				if (vel_num == 5)
				{
					rotation_velocity_aver = rotation_velocity_aver / vel_num;
					// std::cout << rotation_velocity_aver << std::endl;

					// std_msgs::Float32 rev_msg;
					// rev_msg.data = rotation_velocity_aver;
					// Rev_pub_.publish(rev_msg);

					rotation_velocity_aver = 0;
					vel_num = 0;
				}
			}

			//旋转向量
			// rotation_vec = cv::Vec3f(30 * cos(polar_angle_average), 30 * cos(polar_angle_average), 0);
			//std::cout << rotation_vec <<std::endl;

			// double rotation_angle = get_vector_angle(rotation_vec, rotation_vec_last);
			// rotation_vec_last = rotation_vec;
			// if (rotation_angle != 0)
			// {
			// 	rotation_angle_average_1 += rotation_angle;
			// 	vec_num++;
			// }
			// if (vec_num == 10)
			// {
			// 	rotation_angle_average_1 = rotation_angle_average_1 / vec_num;
			// 	rotation_angle_average_2 = rotation_angle_average_1;
			// 	rotation_angle_average_1 = 0;
			// 	vec_num = 0;
			// }

			// cv::Vec3f Vn = rotation_vec.cross(detect_vec); //旋转向量与检测向量(1,1,0)的叉积为相似度向量

			// if (rotation_angle_average_2 != 0)
			// {
			// 	//				std::cout << (rotation_angle_average_2 / 2) << std::endl;
			// 	if (get_vector_norm(Vn) < (get_vector_norm(rotation_vec) * sin(rotation_angle_average_2 / 2)))
			// 	{
			// 		//					std::cout << "相似度Vn: " << Vn << std::endl;
			// 		if (detect_num == 0)
			// 		{
			// 			last_time = msg->events[msg->events.size() / 2].ts;
			// 		}
			// 		detect_num++;
			// 		if (last_if)
			// 		{
			// 			last_time = msg->events[msg->events.size() / 2].ts;
			// 			last_if = false;
			// 		}
			// 	}
			// }

			// if (detect_num >= 10)
			// {
			// 	detect_num = 0;
			// 	ros::Duration delta_t = msg->events[msg->events.size() / 2].ts - last_time;
			// 	rotation_velocity = 1.0 * 60 * 5 / (delta_t.toSec());
			// 	//std::cout << "时间间隔delta: " << delta_t <<std::endl;

			// 	rotation_velocity = rotation_velocity * (1 + sin(rotation_angle_average_2 / 2));

			// 	std::cout << "电机转速: " << rotation_velocity << std::endl;
			// 	last_time = msg->events[msg->events.size() / 2].ts;

			// 	rotation_velocity_aver += rotation_velocity;
			// 	vel_num++;
			// 	if (vel_num == 10)
			// 	{
			// 		rotation_velocity_aver = rotation_velocity_aver / vel_num;
			// 		std::cout << "平均转速：" << rotation_velocity_aver << std::endl;
			// 		rotation_velocity_aver = 0;
			// 		vel_num = 0;
			// 	}
			// }
		}
	}

	Image_pub_.publish(cv_image.toImageMsg());

	generate_num++;
	count_points++;
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
	return sqrt(a(0) * a(0) + a(1) * a(1) + a(2) * a(2));
}
