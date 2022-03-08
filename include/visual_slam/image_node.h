// Author: Srikanth Malla
// Purpose: image node header with abstract method and member declarations

#ifndef _IMAGE_NODE_H_
#define _IMAGE_NODE_H_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <string>     // std::string, std::to_string
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

class ImageNode{

public:
	ImageNode();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	~ImageNode();

private:
	ros::NodeHandle nh_;
	ros::Subscriber img_sub_;//subscribing images
	ros::Publisher odom_pub_;//publish path

  std::string OPENCV_WINDOW_="Left Camera";

  std::vector<cv::Point2f> points1;
	std::vector<cv::Point2f> points2;

  cv::Mat I1;//Image at time t1
	cv::Mat I2;//Image at time t2

	cv::Mat output; //Output to display (by adding features)
	cv::Mat R;// rotation matrix
	cv::Mat t;// translation matrix

  visualization_msgs::Marker odom, gt_odom;// predicted and ground truth odometry
	tf::Transform curr_pose;

  double focal_length=984.2439;//diagonal element of K matrix
	float frame_rate=0;//to check the processing speed
	int counter=0; //for checking first few frames
	int threshold=1000; //number of feature points

	cv::Point2d pp=cv::Point2d(690.0,233.1966);//third column of K matrix

};


#endif
