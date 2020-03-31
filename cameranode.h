#ifndef CAMERANODE_H
#define CAMERANODE_H

#include <math.h>
#include <ros/ros.h>
#include <string>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>



class CameraNode
{
public:
    CameraNode();

protected:
     std::string homepath;
     std::string prefix;
     std::string topic_conti_cam;
     std::string camera_Device_Id;
     std::string camera_port;
     std::string camera_info_url;

    camera_info_manager::CameraInfoManager *p_camera_info;

    const char *ext = "jpg";
    bool enable_publish = false;
    bool enable_imwrite = true;
    int jpeg_compression_level = 95;

    void FramePublish(cv::Mat frame);

private:
    void ReadRosCameraNodeParameters();
    void CreateCameraPointer();
    void IsCameraCalibrated();



    ros::NodeHandle nh_cam_publisher;
    sensor_msgs::ImagePtr msg;


};


#endif // CAMERANODE_H
