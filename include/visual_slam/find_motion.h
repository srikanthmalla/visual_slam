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
#include <geometry_msgs/Point.h>
//LINE STRIP marker is used. NOTE: scale of translation is normalized to 1 in monocular scenario
void initPathMarker(visualization_msgs::Marker &m)
{
    m.header.frame_id = "/world"; 
    m.header.stamp = ros::Time();
    m.ns = "path";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    // m.pose.position.x = 0.0;
    // m.pose.position.y = 0.0;
    // m.pose.position.z = 0.0;
    // m.pose.orientation.x = 0.0;
    // m.pose.orientation.y = 0.0;
    // m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.5;
    // m.scale.y = 0.5;
    // m.scale.z = 0.5;
    m.color.b = 1.0;
    m.color.a=1.0;  
}

void addWayPoint(visualization_msgs::Marker &m, cv::Mat& R,cv::Mat& t,geometry_msgs::Pose& curr_pose)
{	
	geometry_msgs::Point p;
	//TODO: Update curr_pose with R and t
	p=curr_pose.position;
	m.points.push_back(p);
}
#endif