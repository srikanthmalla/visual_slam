// Author: Srikanth Malla
// Purpose: feature and flow computation and opencv visualization functions

#ifndef _FEATURE_DETECTOR_H_
#define _FEATURE_DETECTOR_H_

// using namespace std;//because of opencv2/gpu
// #include <opencv2/gpu/gpu.hpp>
#include <opencv2/features2d/features2d.hpp>
//optical flow (ref: https://avisingh599.github.io/vision/monocular-vo/)
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/core.hpp>

void corner_detector(cv::Mat& img_1, std::vector<cv::Point2f>& points1,cv::Mat& output){
	// purpose: corner detection, FAST features using opencv
	// input:  input image (to find corners, img_1), 2d points reference (points1), output image (to visualize keypoints, output)

	std::vector<cv::KeyPoint> keypoints_1;
	int fast_threshold = 20;//this is important
	bool nonmaxSuppression = true;
	// cv::gpu::GpuMat d_src;
	cv::FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	cv::drawKeypoints(img_1,keypoints_1,output);
	cv::KeyPoint::convert(keypoints_1, points1, std::vector<int>());
}


void drawfeatures(cv::Mat& img,std::vector<cv::Point2f>& points){
	// purpose: draw corner features on image
	// input:  input image (to visualize corners, img), 2d corner points (points)

	int radius=5;
	for (int i=0;i<points.size();i++)
			cv::circle(img,cvPoint(points[i].x,points[i].y),radius,CV_RGB(100,0,0),-1,8,0);
}

void featureTracking(cv::Mat& img_1, cv::Mat& img_2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2,cv::Mat& output){
	// purpose: Feature tracking, this function also automatically gets rid of points for which tracking fails
	// input:  input images (img_1, img_2), 2d corner points to output for each image (points1, points2), visualization image (output)

	std::vector<uchar> status;
	std::vector<float> err;
	cv::Size winSize=cv::Size(21,21);
	cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
	cv::calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
	//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
	int indexCorrection = 0;
	for( int i=0; i<status.size(); i++)
	{
		cv::Point2f pt = points2.at(i- indexCorrection);
		if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0)){
			if((pt.x<0)||(pt.y<0)){
					status.at(i) = 0;
				}
				points1.erase (points1.begin() + i - indexCorrection);
				points2.erase (points2.begin() + i - indexCorrection);
				indexCorrection++;
		}
	}
	output=img_2;
	drawfeatures(output,points2);
}

#endif
