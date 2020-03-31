#ifndef CONTICAM_H
#define CONTICAM_H

#include "icamera.h"
#include "cameranode.h"
#include <math.h>
#include <ros/ros.h>
#include <string>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>



class ContiCam :ICamera , CameraNode
{
public:

    ContiCam();
    void CheckCameraOpen();
    void GetCameraParameters();
    void GetCameraSetup();
    void GetCameraImageGrab();
    void GetCameraImagePublish();

protected:

    std::string homepath;
    std::string prefix;
    std::string topic_conti_cam;
    std::string camera_Device_Id;
    std::string camera_port;
    std::string camera_info_url;


    std::string camDevice;


    camera_info_manager::CameraInfoManager *p_camera_info;

    const char *ext = "jpg";
    bool enable_publish = false;
    bool enable_imwrite = true;
    int jpeg_compression_level = 95;
    std::vector<int> compression_params;
    char filename[100];


    cv::VideoCapture cap;
    cv::Mat frame;

    double dWidth;
    double dHeight;
    double frames;


    uint32_t counter = 0;


    sensor_msgs::ImagePtr msg;




private:


    void InitializeCameraParameters();
    int IsCameraOpen();
    void CameraSetup();
    int CameraImageGrab();
    void CameraImagePublish();
    void CameraImageSave();
    void createDateTimeFilename();



    ros::NodeHandle nh_conti_cam;


};

#endif // CONTICAM_H
