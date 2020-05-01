#ifndef __STEREO_IMAGE_RECEIVER_H__
#define __STEREO_IMAGE_RECEIVER_H__

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include "mm_visual_postion/utils/StereoCameraArmModel.h"

//#define READ_IMAGE_MODE

class StereoImageReceiver{
private:
    ros::NodeHandle nh_;
    std::shared_ptr<StereoCameraArmModel> model_ptr_;
    bool model_got_;

	//ros::Subscriber left_camera_info_sub_, right_camera_info_sub_; 
    cv::Mat rightMap1, rightMap2;
	cv::Mat leftMap1, leftMap2;
    bool calibrated;
    #ifdef READ_IMAGE_MODE
        cv::Mat latest_left_image, latest_right_image;
    #endif
public:
    StereoImageReceiver(ros::NodeHandle& nh, bool uncalibrated=false):nh_(nh),
	    model_got_(false),
        calibrated(!uncalibrated)

    {
        model_ptr_ = std::make_shared<StereoCameraArmModel>();
        if(calibrated)
            readCameraInfo();
        //left_camera_info_sub_ = nh_.subscribe("/zed/left/camera_info",1,&StereoImageReceiver::leftCameraInfoCallback, this);
		//right_camera_info_sub_ = nh_.subscribe("/zed/right/camera_info",1,&StereoImageReceiver::rightCameraInfoCallback, this);
		// camera_info only published if image_rect_color is subscribed.

        #ifdef READ_IMAGE_MODE
            std::string path = ros::package::getPath("mm_visual_postion");
            latest_right_image = cv::imread(path + std::string("/dataset/trunk/bad_case/left_image_0.png"));
            latest_left_image = cv::imread(path + std::string("/dataset/trunk/bad_case/right_image_0.png"));
            if(latest_right_image.empty()){
                ROS_ERROR("%s",(std::string("cannot load image: ")+path +std::string("/dataset/trunk/bad_case/right_image_0.png")).c_str());
            }
            if(latest_left_image.empty()){
                ROS_ERROR("%s",(std::string("cannot load image: ")+path +std::string("/dataset/trunk/bad_case/left_image_0.png")).c_str());
            }
            ROS_WARN("read image from disk, thus image will not be changed");
        #endif
    }
    
	void imageCallback(const sensor_msgs::Image::ConstPtr &left_msg, const sensor_msgs::Image::ConstPtr &right_msg){
        try
		{

        }
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("Could not convert to image!");
		}
    }
    void readCameraInfo(){
        model_got_ = model_ptr_->loadDefaultParams();
        initUndistortRectifyMap(model_ptr_->left_camera.intrinsic_mat, model_ptr_->left_camera.distortion_mat, cv::Mat(), model_ptr_->left_camera.intrinsic_mat, model_ptr_->left_camera.image_size, CV_32FC1, leftMap1, leftMap2);
        initUndistortRectifyMap(model_ptr_->right_camera.intrinsic_mat, model_ptr_->right_camera.distortion_mat, cv::Mat(), model_ptr_->right_camera.intrinsic_mat, model_ptr_->right_camera.image_size, CV_32FC1, rightMap1, rightMap2);
    }
    void rectifyLeftImage(const cv::Mat& raw_image, cv::Mat& rectified_image){
        cv::remap(raw_image, rectified_image, leftMap1, leftMap2, cv::INTER_LINEAR);
    }
    void rectifyRightImage(const cv::Mat& raw_image, cv::Mat& rectified_image){
        cv::remap(raw_image, rectified_image, rightMap1, rightMap2, cv::INTER_LINEAR);
    }
    bool getLatestImages(cv::Mat& left_image, cv::Mat& right_image, ros::Time& stamp){
        #ifdef READ_IMAGE_MODE
            left_image = latest_left_image;
            right_image = latest_right_image;
            stamp = ros::Time(0);
            return true;

        #else
            try
            {
                ros::topic::waitForMessage<sensor_msgs::Image>("/zed/left/image_raw_color",ros::Duration(3));
                ros::topic::waitForMessage<sensor_msgs::Image>("/zed/right/image_raw_color",ros::Duration(3));
                const sensor_msgs::Image::ConstPtr left_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/zed/left/image_raw_color",ros::Duration(3));
                const sensor_msgs::Image::ConstPtr right_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/zed/right/image_raw_color",ros::Duration(3));
                
                stamp = right_msg->header.stamp + (left_msg->header.stamp - right_msg->header.stamp) * 0.5;
                cv_bridge::CvImageConstPtr left_msg_image = cv_bridge::toCvShare(left_msg, "bgr8");
                cv_bridge::CvImageConstPtr right_msg_image = cv_bridge::toCvShare(right_msg, "bgr8");

                cv::remap(left_msg_image->image, left_image, leftMap1, leftMap2, cv::INTER_LINEAR);
                cv::remap(right_msg_image->image, right_image, rightMap1, rightMap2, cv::INTER_LINEAR);
            
                return true;
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("Could not convert to image!");
                return false;
            }

        #endif
    }

    bool getLatestRawImages(cv::Mat& left_image, cv::Mat& right_image, ros::Time& stamp){
        #ifdef READ_IMAGE_MODE
            left_image = latest_left_image;
            right_image = latest_right_image;
            stamp = ros::Time(0);
            return true;

        #else
            try
            {
                const sensor_msgs::Image::ConstPtr left_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/zed/left/image_raw_color",ros::Duration(3));
                const sensor_msgs::Image::ConstPtr right_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/zed/right/image_raw_color",ros::Duration(3));
                stamp = right_msg->header.stamp + (left_msg->header.stamp - right_msg->header.stamp) * 0.5;
                cv_bridge::CvImageConstPtr left_msg_image = cv_bridge::toCvShare(left_msg, "bgr8");
                cv_bridge::CvImageConstPtr right_msg_image = cv_bridge::toCvShare(right_msg, "bgr8");
                left_image = left_msg_image->image.clone();
                right_image = right_msg_image->image.clone();          
                
                return true;
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("Could not convert to image!");
                return false;
            }

        #endif
    }

    bool cameraInfoGot(){
		return model_got_;
	}

    bool getArmCameraModel(std::shared_ptr<StereoCameraArmModel>& ptr){
        if(!model_got_) return false;
        ptr = model_ptr_;
        return true;
    }

};

#endif //__STEREO_IMAGE_RECEIVER_H__