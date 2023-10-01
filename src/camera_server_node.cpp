/**
 * @file camera_server_node.cpp
 * @author Julian Rendon (julianrendon514@gmail.com)
 * @brief ROS Publisher node used to send video feeds.
 *
 * This publisher node using the class 'Camera' to capture frames from a camera and
 * converts the CvImage to a ROS message which is then published.
 *
 * @version 1.1.0
 * @date 2023-09-09
 * @copyright Copyright (c) 2023
 */
#include <camera_driver/camera.h>
#include <camera_driver/camera_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <std_msgs/Float32.h>

#define encoding "bgr8"
#define API      cv::VideoCaptureAPIs::CAP_V4L2

class CameraServerNode : protected Camera
{
  private:
    ros::NodeHandle nh;
    CameraUtils::CameraInfo camera;
    image_transport::ImageTransport imgTr;
    image_transport::Publisher imgPub;
    sensor_msgs::ImagePtr imgMsg;
    ros::Publisher luminosityPub;
    std_msgs::Float32 luminosityMsg;
    std::shared_ptr<cv::VideoCapture> videoCap;
    cv::Mat frame;

  public:
    // Constructor.
    CameraServerNode(std::string name, std::string type, int index, int fps)
        : Camera(name, type, index, fps), camera(GetCameraSpecs()), imgTr(nh), videoCap(nullptr)
    {
        imgPub = imgTr.advertise("/camera/raw_image", 10);
        luminosityPub = nh.advertise<std_msgs::Float32>("/luminosity", 20);
        try
        {
            SendCameraFeeds();
        }
        catch (std::exception &e)
        {
            throw;
        }
    };

    // Destructor
    ~CameraServerNode()
    {
        if (videoCap)
        {
            videoCap->release();
        }
    }

    void SendCameraFeeds()
    {
        ROS_INFO("%s::Starting camera feeds for [%s].", __func__, camera.name.c_str());

        bool isFirstFrame = true;   // Flag to track the first frame
        cv::namedWindow(camera.name);
        try
        {
            while (ros::ok())
            {
                if (!videoCap)
                {
                    videoCap = std::make_shared<cv::VideoCapture>(camera.index, API);
                    ROS_DEBUG("%s::%s", __func__, CameraUtils::DumpCamInfo(camera).c_str());
                }
                try
                {
                    frame = CaptureFrame(*videoCap, camera.index);
                }
                catch (std::exception &e)
                {
                    videoCap->release();
                    ROS_INFO("%s()::%s", __func__, e.what());
                    throw;   // Rethrow exception
                }
                if (isFirstFrame)
                {
                    ROS_INFO("%s::[%s] feed is running.", __func__, camera.name.c_str());
                    isFirstFrame = false;
                }

                imgMsg = cv_bridge::CvImage(std_msgs::Header(), encoding, frame).toImageMsg();
                imgPub.publish(imgMsg);

                luminosityMsg.data = CameraUtils::GetLuminosityValue(frame);
                luminosityPub.publish(luminosityMsg);

                ros::Rate(camera.fps).sleep();
            }
        }
        catch (std::exception &e)
        {
            ROS_INFO("%s", e.what());
            throw;   // Rethrow exception
        }
        ROS_DEBUG("%s::Releasing [%s]", __func__, camera.name.c_str());
        videoCap->release();
    }
};

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "camera_server_node");
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       RosUtils::ConfigureLogLevel(argc, argv));
        CameraServerNode camera(RosUtils::GetParam<std::string>(CameraUtils::cameraParams["NAME"]),
                                RosUtils::GetParam<std::string>(CameraUtils::cameraParams["TYPE"]),
                                RosUtils::GetParam<int>(CameraUtils::cameraParams["INDEX"]),
                                RosUtils::GetParam<int>(CameraUtils::cameraParams["FPS"]));
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Exception caught: %s.", e.what());
    }
    return 0;
}