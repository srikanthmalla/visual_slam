#ifndef _FIND_MOTION_H_
#define _FIND_MOTION_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//pp is principal point (3rd column of K matrix, which is intrinsics)
//f is focal length (diagonal element of K matrix, which is intrinsics)
void get_RT(std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, cv::Mat& R,cv::Mat& t, double& focal,cv::Point2d& pp){
	cv::Mat E=cv::findEssentialMat(points1, points2, focal, pp, cv::RANSAC, 0.999, 1.0);
	cv::recoverPose(E, points1, points2, R, t, focal, pp);
}

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//LINE STRIP marker is used. NOTE: scale of translation is normalized to 1 in monocular scenario
void initPathMarker(visualization_msgs::Marker &m)
{
    m.header.frame_id = "/world"; 
    m.header.stamp = ros::Time();
    m.ns = "path";
    m.id = 1;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    // m.pose.position.x = 0.0;
    // m.pose.position.y = 0.0;
    // m.pose.position.z = 0.0;
    // m.pose.orientation.x = 0.0;
    // m.pose.orientation.y = 0.0;
    // m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.25;
    // m.scale.y = 0.25;
    // m.scale.z = 0.25;
    // m.color.r = 0.0;
    // m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a=1.0;  
}

#include "tf/tf.h"//this is from ros
void cvmat2Transform(cv::Mat& R,cv::Mat& t,tf::Transform& T){
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
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>

void addWayPoint(visualization_msgs::Marker &m, cv::Mat& R,cv::Mat& t,tf::Transform& curr_pose)
{	
	static tf::TransformBroadcaster br;
	tf::Transform T;
	cvmat2Transform(R,t,T);
	curr_pose=curr_pose*T;
	std::cout<<"\n"<<curr_pose.getOrigin().x()<<" "<<curr_pose.getOrigin().y()<<" "<<curr_pose.getOrigin().z()<<"\n"<<std::flush;
	br.sendTransform(tf::StampedTransform(curr_pose, ros::Time::now(), "world", "tested"));
	//getting the position to draw path
	tf::Vector3 l(curr_pose.getOrigin());
	geometry_msgs::Point p;
	p.x=l.x();p.y=l.y();p.z=l.z();
	m.points.push_back(p);
}
#endif