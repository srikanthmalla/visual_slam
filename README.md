# Visual SLAM

## Requirements

1. ROS (tested with kinetic)

2. `sudo pip install kitti2bag` (for conversion to rosbag)


## Run the process

1. Clone the repo to your workspace `https://github.com/srikanthmalla/visual_slam.git`

2. cd `visual_slam` (go in to the package)

3. Download Kitti Visual Odometry Dataset `sh download_dataset.sh` (automatically into datasets folder)

4. Compile the code using `catkin_make`

5. Run `roslaunch visual_slam visual_slam.launch`

## RunTime

Input data (camera) is 9.690 hz

With cpu_flow (KLT)~ program runs at 3hz

For using gpu_flow from openCV, please check `setup_notes.txt`

## To remember

* In opencv z is considered forward, but x is considered backward in ROS (kitti) and z down.
