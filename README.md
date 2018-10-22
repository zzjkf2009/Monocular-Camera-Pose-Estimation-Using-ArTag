# Monocular Camera Pose Estimation Using ArTag
---
## Overview and Motivation
This is a tutorial ROS project that detecting the pose of the monocular camera (a usb-webcam) using ARtags. The camera information is got from the package [**camera_calibration**](http://wiki.ros.org/camera_calibration). With this package, the width, height, camera matrix, distortion coefficients, projection matrix, rectification matrix will be obtained. Those data can be stored in a yaml file. The camera_info publisher can read the data from this yaml file and publish the camera info ([sensor_msgs/CameraInfo](http://docs.ros.org/jade/api/sensor_msgs/html/msg/CameraInfo.html)). The raw image recorder by the usb webcam can be published by the node [**usb_cam_node**](http://wiki.ros.org/usb_cam). Finally, the pose of the camera with ARtag can be determined using package [**ar_tracker_alvar**](http://wiki.ros.org/ar_track_alvar). Therefore, the robot pose (with the camera) relative to ARTags can be founded. This is used for indoor navigation and object recognition (using ARTag's ID). This is only sample practice approach for camera-based localization, in next step, the multi-sensors (camera, IMU) will be used for Direct Visual-Inertial Odometry. Besides, a better choice is to use 3D camera (stereo camera or kinetic) for camera pose estimation, which may lead more accurate result. This also can be done by those packages mentioned above.

## Prerequisite
- [usb_cam](http://wiki.ros.org/usb_cam)
Connect your external webcam to the computer, check the width, height, frame rate of this camera, and run code like below, the device can be modified according you device stetting, eg. _video_device:=/dev/video1.
```
rosrun usb_cam usb_cam_node _image_width:=1280 _image_height:=720 _framerate:=30 _video_device:=/dev/video0 _pixel_format:=mjpeg _camera_frame_id:=camera_frame
```
- [camera_calibration](http://wiki.ros.org/camera_calibration)
There is a [tutorial](http://ros-developer.com/2017/04/23/camera-calibration-with-ros/) for camera calibration in ros using the package mentioned above. The camera parameters can also be obtained from lots of other toolkit eg. MATLAB or OpenCV.
```
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.02517 image:=/usb_cam/image_raw camera:=/usb_cam
```
- [ar_tracker_alvar](http://wiki.ros.org/ar_track_alvar)
Please refer more details about this package from the link. And there are some resources for ARTag detection, recognition, and pose estimation.
**Reference**: [Vision Based Localization using Reliable Fiducial Markers](https://pdfs.semanticscholar.org/11f2/a68d1332851067898088cb33075566396d7c.pdf)

- [Yaml_cpp](https://github.com/jbeder/yaml-cpp)
This is YAML parser and emitter in C++, which can help us "read" the yaml file. More details can be founded in reference link.
**Reference**: [Read yaml file with yaml-cpp in ROS C++ ] (http://ossyaritoori.hatenablog.com/entry/2017/08/16/Read_yaml_file_with_yaml-cpp_in_ROS_C%2B%2B_/)
## Build and run via command-line
 Build:
```
git clone --recursive https://github.com/zzjkf2009/
cd <path to workspace>
catkin_make
```
Run:
```
roslaunch usb_webcam_calibration web_cam_indv.launch
```
## Run python node
To run a python script using rosrun, the script has to be executable. You can make
publisher_node.py executable by:
```
chmod +x <package_path>/<package_name>/src/publisher_node.py
chmod +x /home/yzy/catkin_ws/src/usb_webcam_calibration/src/camera_info_publisher.py
```
Then Run
```
rosrun <package_name> publisher_node.py
rosrun usb_webcam_calibration src/camera_info_publisher.py /home/yzy/catkin_ws/src/usb_webcam_calibration/camera_info.yaml
```
## Notice
The current *camera_info*(sensor_msgs/CameraInfo) is got from the node *hard_cam_info_publisher*, which manually typed the camera info and the source file *camera_info_publisher.cpp* can read the parameters info from yaml file.One can run the node as:
```
rosrun usb_webcam_calibration cam_info_pub _yaml_path:=/home/yzy/catkin_ws/src/usb_webcam_calibration/camera_info.yaml
```
