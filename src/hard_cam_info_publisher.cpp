/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-10-21T14:41:28-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: hard_cam_info_publisher.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-10-22T15:08:13-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

int main(int argc, char** argv){
        ros::init(argc, argv, "camera_info_publisher");
        ros::NodeHandle n;
        sensor_msgs::CameraInfoPtr cam(new sensor_msgs::CameraInfo());
        ros::Publisher cam_info_pub = n.advertise<sensor_msgs::CameraInfo>("usb_cam_info",500);
        cam->width = 1280;
        cam->height = 720;
        cam->distortion_model = "distortion_model";
        cam->D = {0.013750, -0.162804, 0.008105, 0.002423, 0.000000};
        ROS_INFO("D is done");
        cam->K = {1133.600994, 0.0, 643.406473, 0.0, 1136.313402, 427.424258, 0.0, 0.0, 1.0};
        cam->R =  {1, 0, 0, 0, 1, 0, 0, 0, 1};
        cam->P =  {1119.652222, 0, 646.539079, 0, 0, 1133.008911, 430.780251, 0, 0, 0, 1, 0};
        while(n.ok()) {
                cam_info_pub.publish(cam);
                ros::Duration(0.5).sleep();
        }
}
