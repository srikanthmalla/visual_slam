#install kitti2bag
# sudo pip install kitti2bag

# echo "downloading visual odometry dataset of size 22 GB"
# wget http://kitti.is.tue.mpg.de/kitti/data_odometry_gray.zip -o 2011_09_26_odometry_gray.zip
# wget http://kitti.is.tue.mpg.de/kitti/data_odometry_calib.zip -o 2011_09_26_calib.zip
# unzip 2011_09_26_odometry_gray.zip
# unzip 2011_09_26_calib.zip
# kitti2bag -t 2011_09_26 raw_synced .

#small dataset to test
# echo "downloading small datset and converting to rosbag"
# wget http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
# wget http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_26_calib.zip
# unzip 2011_09_26_drive_0002_sync.zip
# unzip 2011_09_26_calib.zip
# kitti2bag -t 2011_09_26 -r 0002 raw_synced .

# already created rosbags
mkdir datasets && cd datasets
wget https://www.dropbox.com/s/jgxqtwfsqtoavz6/2011_09_26_0001.bag
cd ..