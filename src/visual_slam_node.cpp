// Author: Srikanth Malla
// Purpose: main node for visual slam

#include "visual_slam/image_node.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "visual_slam_node");
	ImageNode node;
	return 0;
}
