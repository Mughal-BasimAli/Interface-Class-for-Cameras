#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// Libraries for Socket programming

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <bitset>


/**
 * creates the name for one image
 * @param[in] prefix, path where the images are saved
 * @param[in] frameid, id the current image
 * @param[out] filename, full output name
 * @param[in] ext, type of the image to be saved
 * @param[in] size of the filename
 */
 void createDateTimeFilename(const char* prefix, uint32_t frameid, char* filename, const char* ext, const size_t size)
 {
    std::vector<char> name(size);
    struct timespec ts;

    //Modifies the std::timespec object pointed to by ts to hold the current calendar time in the time base TIME_UTC
    //Only TIME_UTC is supported
    timespec_get(&ts, TIME_UTC);
    memset(filename, 0, size);

    //Converts the date and time information from a given calendar time ts.tv_sec to a null-terminated multibyte character name
    size_t pos = strftime(name.data(), sizeof(name), "%Y%m%d-%H%M%S", localtime(&ts.tv_sec));

    //expand filename with nanoseconds
    std::snprintf(name.data() + pos, sizeof name - pos, ".%09ld", ts.tv_nsec);

    //set complete filename
    if(prefix == nullptr){
        std::sprintf(filename, "%s_%6.6d.%s", name.data(), frameid, ext);
    }
    else
    {
        std::sprintf(filename, "%s%s_%6.6d.%s",prefix, name.data(), frameid, ext);
    }
 }


int main(int argc, char* argv[]) {


    std::string homepath = getenv("HOME");
    std::string prefix(homepath + "LogFiles/Images_Conti_120_10/");
    std::string topic_conti_cam("/sense/ContiCam_Image");
    std::string camera_Device_Id("192.168.120.1");
    std::string camera_port("25000");
    std::string camera_info_url("");

    camera_info_manager::CameraInfoManager *p_camera_info;

    const char *ext = "jpg";
    bool enable_publish = false;
    bool enable_imwrite = true;

    int jpeg_compression_level = 95;
    std::vector<int> compression_params;

    char filename[100];

    ros::init(argc, argv, "conti_cam");
    ros::NodeHandle node_conti_cam("~");

    node_conti_cam.param<std::string>("prefix", prefix, homepath + "LogFiles/Images_Conti_120_10/");
    node_conti_cam.param<std::string>("topic_conti_cam", topic_conti_cam, "/sense/ContiCam_Image/1");

    node_conti_cam.param<std::string>("camera_Device_Id", camera_Device_Id, "192.168.120.1");
    node_conti_cam.param<std::string>("camera_port", camera_port, "25000");
    node_conti_cam.param<std::string>("camera_info_url", camera_info_url, "file://~/.ros/camera_info/conti/info.ini");

    node_conti_cam.param<bool>("enable_publish", enable_publish, false);
    node_conti_cam.param<bool>("enable_imwrite", enable_imwrite, true);

    node_conti_cam.param<int32_t>("jpeg_compression_level", jpeg_compression_level, 95);

    //Create camera info pointer
    p_camera_info = new camera_info_manager::CameraInfoManager(node_conti_cam, "conti_camera", camera_info_url);

    //Check if the provided exists, otherwise try to create this directory
    if(enable_imwrite && boost::filesystem::exists(prefix) == false){
        ROS_ERROR("Provided path: %s does not exists. Trying to make directory", prefix.c_str());
        auto res = boost::filesystem::create_directories(prefix);
        if (res == false){
            ROS_ERROR("Could not create directory %s", prefix.c_str());
            ros::shutdown();
        }
    }

    image_transport::ImageTransport imageTransport(node_conti_cam);
    image_transport::CameraPublisher pub = imageTransport.advertiseCamera(topic_conti_cam, 1);

    ros::NodeHandle sh;
    int count = 0 ;

    ros::Publisher sync_message_publisher = sh.advertise<std_msgs::Bool>("topic_sync_message", 1000);


    //set up parameters for the jpeg quality
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(jpeg_compression_level);

    //set up camera device to be opened
    std::string camDevice("udp://");
    camDevice = camDevice + camera_Device_Id + ":" + camera_port + "/video?x.mjpeg";

    cv::VideoCapture cap;

    //try to open the cam device
    std::cout << "Trying to open device: " << camDevice <<"\n";
    cap.open(camDevice);
    if (!cap.isOpened())  // if not success, exit program
    {
        std::cout << "Failed to open the camera device: "<< camDevice << "\n";
        return -1;
    }

    // Check if the camera is calibrated
    bool ok = false;
    ok = p_camera_info->isCalibrated();
    if (ok == false)
        printf("isCalibrated failed on camera\n");

    //get the width of frames of the video
    //double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    //get the height of frames of the video
    //double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    double dHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    //get the value for the frames per second
    //double frames = cap.get(CV_CAP_PROP_FPS);
    double frames = cap.get(cv::CAP_PROP_FPS);

    std::cout << "Camera device: " << camDevice << " opened\n";
    std::cout << "Got frame size : " << dWidth << " x " << dHeight << "\nFrames per second: " << frames << "\nStart grabbing frames\n";

    cv::Mat frame;

    uint32_t counter = 0;
    constexpr size_t filenameSize = sizeof(filename) / sizeof(filename[0]);

    sensor_msgs::ImagePtr msg;

    // Socket Programming

    int sockfd;                         // Socket file descriptor
    unsigned char syncmessage [] = {0x04, 0x00};

    struct sockaddr_in servaddr;


    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);

        memset(&servaddr, 0, sizeof(servaddr));


        // Filling server information
        servaddr.sin_family    = AF_INET; // IPv4
        servaddr.sin_addr.s_addr =inet_addr("192.168.120.255");
        servaddr.sin_port = htons(8000);      // Host Byte Order to Network Byte Order (Little to big Endian)


        int broadcast = '1';

        if(setsockopt(sockfd,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)) < 0)

        {
            std::cout<<"Error in setting Broadcast option";
            close(sockfd);
            return 0;
        }


    }
    while (ros::ok()) {


        std_msgs::Bool bool_message;
        bool_message.data = false;

        // read a new frame from video
        bool bSuccess = cap.grab();
        cap.retrieve(frame);


        //break loop if no frames has been grabbed
        if (!bSuccess)
        {
            std::cout << "Cannot read frame from video stream" << "\n";;
            break;
        }

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


        if (enable_imwrite)
        {
            createDateTimeFilename((prefix.c_str()), counter, filename, ext, filenameSize);
            //save the image given the parameter
            auto success = cv::imwrite(filename, frame, compression_params);
            counter++;
            if(success == false){
                std::cout << "Could not save image: " << filename << "\n";
            }

            //reset counter for filename
            if(counter > 999999){
                counter = 0;
            }
        }

       if (count < 1)
       { count=count+1;
         continue;}

       if (count == 1 )
       {
           bool_message.data = true;
           sync_message_publisher.publish(bool_message);


            sendto(sockfd,syncmessage,sizeof (syncmessage),MSG_CONFIRM, (const struct sockaddr *) &servaddr,sizeof (servaddr));
            count =0 ;

       }



    }

}
