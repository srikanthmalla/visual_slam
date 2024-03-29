// Author: Srikanth Malla
// Purpose: image node node for tackling image subscriber callback

#include "visual_slam/image_node.h"
#include "visual_slam/feature_detector.h"
#include "visual_slam/find_motion.h"

ImageNode::ImageNode(){
	// Purpose: initialize the image node with subscribers and publishers

	cv::namedWindow(OPENCV_WINDOW_);
	initPathMarker(odom);//predicted
	// initPathMarker(gt_odom);//groundtruth
	curr_pose.setIdentity();
	img_sub_ = nh_.subscribe<sensor_msgs::Image>("/cam00/left/image_raw", 10, &ImageNode::imageCallback, this);
	odom_pub_ =  nh_.advertise<visualization_msgs::Marker> ("odom", 10);
	// gt_odom_pub_= nh_.advertise<visualization_msgs::Marker> ("gt_odom", 10);
	ros::spin();
}

void ImageNode::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	// Purpose: Callback function for image subscribers
	// Input: image ptr message

	clock_t timer;
	timer=clock();
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
			get_RT(points1,points2,R,t,focal_length,pp);//from find_motion header
			addWayPoint(odom, R, t, curr_pose);
			odom_pub_.publish(odom);
				// gt_odom_pub_.publish(gt_odom);
		}
		timer=clock()-timer;
		float time_taken=((float)timer)/CLOCKS_PER_SEC;
		frame_rate=1/(time_taken);
		//add frame rate to the output
	cv::putText(output, "Frame Rate: "+std::to_string(frame_rate), cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 4);
	cv::imshow(OPENCV_WINDOW_, output);// Update GUI Window
	cv::waitKey(1);
	//copy back
	I1 = I2;
	points1 = points2;
	counter++;
}

ImageNode::~ImageNode(){
	// Purpose: Destructor for Imagenode

	cv::destroyWindow(OPENCV_WINDOW_);
}
