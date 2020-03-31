#include "cameranode.h"
#include <image_transport/image_transport.h>


CameraNode::CameraNode()
{
    ReadRosCameraNodeParameters();
    CreateCameraPointer();
    IsCameraCalibrated();


   //CreatePublisherTopic();

}

void CameraNode::ReadRosCameraNodeParameters()
{
    nh_cam_publisher.param<std::string>("prefix", prefix, homepath + "LogFiles/Images_Conti_120_10/");
    nh_cam_publisher.param<std::string>("topic_conti_cam", topic_conti_cam, "/conti_cam/image_raw");

    nh_cam_publisher.param<std::string>("camera_Device_Id", camera_Device_Id, "192.168.120.1");
    nh_cam_publisher.param<std::string>("camera_port", camera_port, "25000");
    nh_cam_publisher.param<std::string>("camera_info_url", camera_info_url, "file:///home/i011398/workspace/autonomous_tractor/project/ros/src/conti_cam/calib/conti_calib.yaml");

    nh_cam_publisher.param<bool>("enable_publish", enable_publish, true);
    nh_cam_publisher.param<bool>("enable_imwrite", enable_imwrite, true);

    nh_cam_publisher.param<int32_t>("jpeg_compression_level", jpeg_compression_level, 95);
}



void CameraNode::CreateCameraPointer()
{

        p_camera_info = new camera_info_manager::CameraInfoManager(nh_cam_publisher, "conti_camera", camera_info_url);
}

void CameraNode::IsCameraCalibrated()
{

    // Check if the camera is calibrated
    bool ok = false;
    ok = p_camera_info->isCalibrated();

    if (ok == false)
        printf("  isCalibrated failed on camera\n");

}

/*
void CameraNode::CreatePublisherTopic()
{

    image_transport::ImageTransport imageTransport(nh_cam_publisher);
    image_transport::CameraPublisher pub = imageTransport.advertiseCamera(topic_conti_cam, 1);

} */

void CameraNode::FramePublish(cv::Mat frame)
{
    image_transport::ImageTransport imageTransport(nh_cam_publisher);
    image_transport::CameraPublisher pub = imageTransport.advertiseCamera(topic_conti_cam, 1);

    if (enable_publish == true)
    {
        // Publish ROS image message:
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();

        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(p_camera_info->getCameraInfo()));
        ci->header.frame_id = msg->header.frame_id;
        ci->header.stamp = msg->header.stamp;
        pub.publish(*msg, *ci);
    }
}
