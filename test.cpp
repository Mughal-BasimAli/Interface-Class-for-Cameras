#include "icamera.h"
#include "cameranode.h"
#include "conticam.h"

/**
 * creates the name for one image
 * @param[in] prefix, path where the images are saved
 * @param[in] frameid, id the current image
 * @param[out] filename, full output name
 * @param[in] ext, type of the image to be saved
 * @param[in] size of the filename
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conti_cam");

      CameraNode camnode;
      ContiCam com;

      com.GetCameraParameters();
      com.CheckCameraOpen();
      com.GetCameraSetup();

      while (ros::ok())
      {
      com.GetCameraImageGrab();
      com.GetCameraImagePublish();

      }


      ros::spin();

    return 0;

}





