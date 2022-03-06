#install kitti2bag
sudo pip install kitti2bag
mkdir datasets
cd datasets

# echo "downloading visual odometry dataset of size 22 GB"
# wget http://kitti.is.tue.mpg.de/kitti/data_odometry_gray.zip -o 2011_09_26_odometry_gray.zip
# wget http://kitti.is.tue.mpg.de/kitti/data_odometry_calib.zip -o 2011_09_26_calib.zip
# unzip 2011_09_26_odometry_gray.zip
# unzip 2011_09_26_calib.zip
# kitti2bag -t 2011_09_26 raw_synced .

#small dataset to test
echo "downloading small datset and converting to rosbag"
## recently kitti is moved to aws
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
unzip 2011_09_26_drive_0002_sync.zip
unzip 2011_09_26_calib.zip
kitti2bag -t 2011_09_26 -r 0002 raw_synced .
cd ..
