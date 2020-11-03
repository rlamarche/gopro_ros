# gopro_ros

This project was an attempt to run inertial visual slam algorithms on a Gopro5 video + imu data.
Imu data is written as gpmf data in the mp4 video files.
The node ros extract theses informations, then emit them on ros topics along with the video data.

The user can then generate rosbags and replay them after using any slam algorithms.

Visual SLAM was working, with OpenVSLAM, but inertial visual slam was not. There might be a problem in the synchronisation process of my program, or the data is not well synchronized in the gopro, or my datasets were not good enough, of my calibration too bad.
By the way, if this code can be useful for somebody, so be it.
