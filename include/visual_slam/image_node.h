#ifndef _IMAGE_NODE_H_
#define _IMAGE_NODE_H_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
class ImageNode{
public:
	ImageNode();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	~ImageNode();
private:
	ros::NodeHandle nh_;
	ros::Subscriber img_sub_;
	std::string OPENCV_WINDOW_="Left Camera";
	std::vector<cv::Point2f> points1;
	std::vector<cv::Point2f> points2;
	cv::Mat I1;
	cv::Mat I2;
	int counter=0; //for checking first few frames
	int threshold=1000; //number of feature points
	cv::Mat output; //Output to display (by adding features)
};


#endif