#ifndef _IMAGE_NODE_H_
#define _IMAGE_NODE_H_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

class ImageNode{
public:
	ImageNode();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	~ImageNode();
private:
	ros::NodeHandle nh_;
	ros::Subscriber img_sub_;
	std::string OPENCV_WINDOW_="Left Camera";
};


#endif