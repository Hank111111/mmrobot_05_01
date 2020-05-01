#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "cameraZedUVC.h"
namespace mm_zed_camera_rectification{
class CameraRectification{
private:
	cv::Mat rightMap1, rightMap2;
	cv::Mat leftMap1, leftMap2;
    ros::NodeHandle& nh_;
    ros::Publisher rect_image_pub;
    ZedWrapper zed_wrapper;
    cv::Mat raw_image;
public:
    CameraRectification(ros::NodeHandle& nh): nh_(nh){};
    void init();
    void rectifyImage();
    void captureRawImage();
};





class CameraRectificationNodelet : public nodelet::Nodelet
{   
private:
	std::shared_ptr<CameraRectification> camera_rectification_ptr;

public:
    virtual void onInit();
    virtual void main();


};
}
