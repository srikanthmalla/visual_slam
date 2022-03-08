// Author: Srikanth Malla
// Purpose: Motion related transformation computation and ros visualization functions

#ifndef _FIND_MOTION_H_
#define _FIND_MOTION_H_

#include "tf/tf.h"//this is from ros
#include <geometry_msgs/Point.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>// for visualizing groundtruth path
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//pp is principal point (3rd column of K matrix, which is intrinsics)
//f is focal length (diagonal element of K matrix, which is intrinsics)

void get_RT(std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, cv::Mat& R,cv::Mat& t, double& focal,cv::Point2d& pp){
  // Purpose: compute essential matrix and Rotation, translation
  // input: 2d points reference (points1, points2), Rotation 3x3 (R), translation (3x1), focal length (focal), 2d points (pp)

	cv::Mat E=cv::findEssentialMat(points1, points2, focal, pp, cv::RANSAC, 0.999, 1.0);
	cv::recoverPose(E, points1, points2, R, t, focal, pp);
}

//LINE STRIP marker is used. NOTE: scale of translation is normalized to 1 in monocular scenario
void initPathMarker(visualization_msgs::Marker &m){
    // Purpose: create a path marker
    // input: visualization ros marker m

    m.header.frame_id = "/world"; 
    m.header.stamp = ros::Time();
    m.ns = "path";
    m.id = 1;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.25;
    // m.color.r = 0.0;
    // m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a=1.0;  
}

void cvmat2Transform(cv::Mat& R, cv::Mat& t, tf::Transform& T){
  // Purpose: convert CVmat to transform
  // input: Rotation 3x3 (R), translation (3x1), output ros transform (T)

	double r11=R.at<double>(0,0);
	double r12=R.at<double>(0,1);
	double r13=R.at<double>(0,2);	
	double r21=R.at<double>(1,0);
	double r22=R.at<double>(1,1);
	double r23=R.at<double>(1,2);
	double r31=R.at<double>(2,0);
	double r32=R.at<double>(2,1);
	double r33=R.at<double>(2,2);
	tf::Matrix3x3 rotation(r11,r12,r13,r21,r22,r23,r31,r32,r33);
	double t1=t.at<double>(0,0);
	double t2=t.at<double>(0,1);
	double t3=t.at<double>(0,2);
	tf::Vector3 translation(t1,t2,t3);
	T.setBasis(rotation);
	T.setOrigin(translation);
}

void addWayPoint(visualization_msgs::Marker &m, cv::Mat& R, cv::Mat& t, tf::Transform& curr_pose){
  // Purpose: add visualization waypoint
  // input: visualization marker to update (m),  Rotation 3x3 (R), translation (3x1), curr pose transform (T)

	static tf::TransformBroadcaster br;
	tf::Transform T;
	// tf::TransformListener listener;// ground truth listener
	// tf::StampedTransform gt;
	tf::Matrix3x3 R1(0,0,-1,-1,0,0,0,1,0);
	tf::Vector3 t1(0,0,0);
	tf::Transform V(R1,t1);// V for Visualization
	cvmat2Transform(R,t,T);
	curr_pose=curr_pose*T;
	V=V*curr_pose;
	std::cout<<"\n"<<V.getOrigin().x()<<" "<<V.getOrigin().y()<<" "<<V.getOrigin().z()<<"\n"<<std::flush;
	br.sendTransform(tf::StampedTransform(V, ros::Time::now(), "world", "tested"));
	//getting the position to draw path
	tf::Vector3 l(V.getOrigin());
	geometry_msgs::Point p;
	p.x=l.x();p.y=l.y();p.z=l.z();
	m.points.push_back(p);
  //   try{
		// listener.lookupTransform("/tested", "/velodyne",  
  //                              ros::Time(0), gt);
  //   	p.x=gt.getOrigin().x();p.y=gt.getOrigin().y();p.z=gt.getOrigin().z();
		// gt_m.points.push_back(p);
  //   }
  //   catch (tf::TransformException ex){
  //     ROS_ERROR("%s",ex.what());
  //   }
}

#endif
