#ifndef ICAMERA_H
#define ICAMERA_H

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


//   ICamera class provides a generic interface to Cameras


class ICamera
{
public:
    ICamera();

protected:


private:


    virtual void InitializeCameraParameters()=0;
    virtual int IsCameraOpen()=0;
    virtual void CameraSetup()=0;
    virtual int CameraImageGrab()=0;
    virtual void CameraImagePublish()=0;
    virtual void CameraImageSave()=0;
    virtual void createDateTimeFilename()=0;


};

#endif // ICAMERA_H
