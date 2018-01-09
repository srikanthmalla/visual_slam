#include "visual_slam/image_node.h"
#include "visual_slam/feature_detector.h"
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
    I2 = cv_ptr->image;//get new image
    if (counter==0){
	    corner_detector(I2,points2,output);
    	std::cout<<"Running FAST corner detector at frame "<<counter <<" no of corners ::"<<points2.size()<<"\n";
	}
    else{
    	if (points1.size()<threshold){
    		points1.clear();
			corner_detector(I1,points1,output);
	    	std::cout<<"\nRunning FAST corner detector at frame "<<counter <<" no of corners ::"<<points1.size()<<"\n";
    	}
    	featureTracking(I1,I2,points1,points2,output);
    	std::cout<<"\rKLT FLow:: no of corners::"<<points2.size()<<std::flush;    	
    }
    cv::imshow(OPENCV_WINDOW_, output);// Update GUI Window
    cv::waitKey(1);
	//copy back
    I1 = I2;
    points1 = points2;
    counter++;
}
ImageNode::~ImageNode(){
	cv::destroyWindow(OPENCV_WINDOW_);
}