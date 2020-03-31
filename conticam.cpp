#include "conticam.h"
#include "cameranode.h"
#include "icamera.h"
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


ContiCam::ContiCam()
{

}

void ContiCam::CheckCameraOpen()
{
    IsCameraOpen();
}

void ContiCam::GetCameraParameters()
{
    InitializeCameraParameters();
}


void ContiCam::GetCameraSetup()
{
    CameraSetup();
}


void ContiCam::GetCameraImageGrab()
{
    CameraImageGrab();
}


void ContiCam::GetCameraImagePublish()
{
    CameraImagePublish();
}

void ContiCam :: InitializeCameraParameters()
{


    nh_conti_cam.param<std::string>("prefix", prefix, homepath + "LogFiles/Images_Conti_120_10/");
    nh_conti_cam.param<std::string>("topic_conti_cam", topic_conti_cam, "/conti_cam/image_raw");
    nh_conti_cam.param<std::string>("camera_Device_Id", camera_Device_Id, "192.168.120.1");
    nh_conti_cam.param<std::string>("camera_port", camera_port, "25000");
    nh_conti_cam.param<std::string>("camera_info_url", camera_info_url, "file:///home/i011398/workspace/autonomous_tractor/project/ros/src/conti_cam/calib/conti_calib.yaml");
    nh_conti_cam.param<bool>("enable_publish", enable_publish, true);
    nh_conti_cam.param<bool>("enable_imwrite", enable_imwrite, true);
    nh_conti_cam.param<int32_t>("jpeg_compression_level", jpeg_compression_level, 95);

}

int ContiCam :: IsCameraOpen()

{
    camDevice = "udp://" + camera_Device_Id + ":" + camera_port + "/video?x.mjpeg";

    //try to open the cam device
    std::cout << "Trying to open device: " << camDevice <<"\n";

    cap.open(camDevice);

    if (!cap.isOpened())  // if not success, exit program
    {
        std::cout << "Failed to open the camera device: "<< camDevice << "\n";
        return -1;
    }

     std::cout << "Camera device: " << camDevice << " opened\n";

}

void ContiCam ::CameraSetup()
{
    //get the width of frames of the video
    dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);

    //get the height of frames of the video
    dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    //get the value for the frames per second
     frames = cap.get(cv::CAP_PROP_FPS);


     std::cout << "Got frame size : " << dWidth << " x " << dHeight << "\nFrames per second: " << frames << "\nStart grabbing frames\n";
}

int ContiCam::CameraImageGrab()
{
    bool bSuccess = cap.grab();
    cap.retrieve(frame);

    //break loop if no frames has been grabbed
    if (!bSuccess)
    {
        std::cout << "Cannot read frame from video stream" << "\n";;
        return -1;
    }

    return 0;

}

void ContiCam::CameraImageSave()
{
    std::cout << "Camera Image Saved " <<"\n";
}


void ContiCam::CameraImagePublish()
{
    ContiCam cam;
    cam.FramePublish(frame);
    std::cout << "Camera image published " <<"\n";

}

void ContiCam ::createDateTimeFilename()
{
    std::cout << "File created " <<"\n";
}
