#include "visual_slam/image_node.h"

ImageNode::ImageNode(){
	cv::namedWindow(OPENCV_WINDOW_);
	img_sub_ = nh_.subscribe<sensor_msgs::Image>("/cam00/left/image_raw", 1, &ImageNode::imageCallback, this);
	ros::spin();

}

void ImageNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_, cv_ptr->image);
    cv::waitKey(1);
}
ImageNode::~ImageNode(){
	cv::destroyWindow(OPENCV_WINDOW_);
}