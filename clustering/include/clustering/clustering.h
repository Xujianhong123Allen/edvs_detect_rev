#include <ros/ros.h>
#include <ros/console.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float32.h>
#include <map>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <cmath>

inline int get_length(int x)
{
	int leng = 0;
	while (x)
	{
		x /= 10;
		leng++;
	}
	return leng;
}

struct event_xy
{
	uint16_t x;
	uint16_t y;
	// int16_t x;
	// int16_t y;
	bool operator<(const struct event_xy &Pair) const
	{
		double X = x * pow(10, get_length(y)) + y;
		double Y = Pair.x * pow(10, get_length(Pair.y)) + Pair.y;
		return X < Y;
	}
};

struct Ts_pol
{
	int64_t ts;
	bool polarity;

	//转化极坐标参数
	double polar_radius;
	double polar_angle;
};

struct Rect_xy
{
	uint16_t x_min;
	uint16_t x_max;
	uint16_t y_min;
	uint16_t y_max;
};

const double pi = 3.141592653;

//获取两个向量之间的夹角
double get_vector_angle(cv::Vec3f a, cv::Vec3f b);

//获取向量的模长
double get_vector_norm(cv::Vec3f a);

class Clustering
{
public:
	Clustering(ros::NodeHandle &nh, ros::NodeHandle nh_private);
	~Clustering();

private:
	ros::NodeHandle nh_;
	ros::Subscriber EventArray_sub_;
	ros::Publisher Image_pub_;
	ros::Publisher Rev_pub_;

	std::map<event_xy, Ts_pol> event_map;
	std::map<event_xy, int> event_accumulation_map;

	// std::ofstream fout;
	// bool area_once = true;
	 
	const int filtering_area_point_num = 4;

	double bulr_angle = pi / 4;

	double polar_angle_current, polar_angle_last;
	double polar_angle_last_cal;

	cv::Rect detect_area;

	Rect_xy rect_xy;

	int generate_num = 0;
	int generate_num_2 = 0;
	int detect_num = 0;

	ros::Time last_time;

	bool last_if = true;

	cv::Vec3f rotation_vec_last; //前一时刻的旋转向量位置
	int vec_num = 0;			 //对旋转向量转动次数的计数

	double rotation_angle_average_1 = 0; //平均的每一次采样的旋转角度
	double rotation_angle_average_2 = 0; //用于检测旋转向量是否在区域内

	cv::Point2f rotation_center; //旋转中心点
	cv::Vec3f rotation_vec;		 //旋转向量
	cv::Vec3f detect_vec;		 //该向量用于检测

	double rotation_velocity = 0; //电机转速
	double rotation_velocity_aver = 0;

	int vel_num = 0;

	int num_total;
	int count_points;
	int rev_class = 100;

	void Clustering_generateCb(const dvs_msgs::EventArray::ConstPtr &msg);
};
