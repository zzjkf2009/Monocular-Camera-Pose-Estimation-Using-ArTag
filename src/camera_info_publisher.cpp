/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-10-22T15:07:46-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: camera_info_publisher.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-10-22T15:08:01-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */


/**
 * @brif: this node read the camera info from the yaml file and create a cameraInfo publisher to publish
 * the associated parameters.
 */
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/CameraInfo.h>
#include <fstream>

int main(int argc, char** argv){
        ros::init(argc, argv, "camera_info_publisher");
        ros::NodeHandle n("~");
        std::string yaml_path;
        sensor_msgs::CameraInfoPtr cam(new sensor_msgs::CameraInfo());
        ros::Publisher cam_info_pub = n.advertise<sensor_msgs::CameraInfo>("usb_cam_info",500);
        /*  if( argc != 2 ) {
                  ROS_ERROR("No path of the yaml file was defined, Use the default path");
                  //yaml_path = "/home/yzy/catkin_ws/src/usb_webcam_calibration/camera_info.yaml";
                  yaml_path = "$(find usb_webcam_calibration)/camera_info.yaml";
           }
           else */
        //yaml_path = argv[1];
        n.getParam("yaml_path",yaml_path);
        std::ifstream fin(yaml_path.c_str(),std::ifstream::in);
        if(fin.is_open()) {
                std::cout << "Successful read the yaml file" << '\n';
        }
        YAML::Node conf = YAML::LoadFile(yaml_path);
        cam->width = conf["image_width"].as<int>();
        cam->height = conf["image_height"].as<int>();
        cam->distortion_model = conf["distortion_model"].as<std::string>();

        // get value
        std::vector<double> dd,kk,rr,pp,buf;
        dd = conf["distortion_coefficients"]["data"].as<std::vector<double> >();
        kk = conf["camera_matrix"]["data"].as<std::vector<double> >();
        rr = conf["rectification_matrix"]["data"].as<std::vector<double> >();
        pp = conf["projection_matrix"]["data"].as<std::vector<double> >();
        // conversion
        boost::array<double, 9ul> kl;
        std::memcpy(&kl[0], &kk[0], sizeof(double)*9);
        boost::array<double, 9ul> rl;
        std::memcpy(&rl[0], &rr[0], sizeof(double)*9);
        boost::array<double, 12ul> pl;
        std::memcpy(&pl[0], &pp[0], sizeof(double)*12);
        // put value
        cam->D =  dd;
        cam->K =  kl;
        cam->R =  rl;
        cam->P =  pl;
        while(n.ok()) {
                cam_info_pub.publish(cam);
                ros::Duration(0.5).sleep();
        }
}
