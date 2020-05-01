#include "streamer/streamer.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <chrono>
#include <image_transport/image_transport.h>
#include "cameraZed.h"
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <mutex>
#include <queue>
#include<thread>

template<class T>
class SafeQueue {

    std::queue<T> q;
    std::mutex m;

public:
    SafeQueue() {}
    void push(T& elem) {
        std::lock_guard<std::mutex> lock(m);
        q.push(elem);
    }
    bool next(T& elem) {
        std::lock_guard<std::mutex> lock(m);
        if (q.empty()) {
            return false;
        }
        elem = q.front();
        q.pop();
        return true;
    }
};
SafeQueue <std::shared_ptr<cv::Mat> > image_ptr_queue_for_streamer;
SafeQueue <std::shared_ptr<cv::Mat> > left_image_ptr_queue_for_ros_topic, right_image_ptr_queue_for_ros_topic;


class StreamerThread{
private:
    streamer::Streamer streamer_;
    ros::Subscriber enable_streamer_subscriber_, disable_streamer_subscriber_;
    std::thread current_thread;
public:
    bool streamer_enabled_;

StreamerThread(ros::NodeHandle& nh){
    enable_streamer_subscriber_ = nh.subscribe("/zed/stream/enable", 10, &StreamerThread::enableCallback, this);
    disable_streamer_subscriber_ = nh.subscribe("/zed/stream/disable", 10, &StreamerThread::disableCallback, this);
}
void enableCallback(const std_msgs::EmptyConstPtr msg){
    streamer_enabled_ = true;
    ROS_INFO_STREAM("streamer nodelet has been enabled.]");
}

void disableCallback(const std_msgs::EmptyConstPtr msg){
    streamer_enabled_ = false;
    ROS_INFO_STREAM("streamer nodelet has been disabled.]");
}
void init(int bitrate, int fps, int frame_width, int frame_height, int stream_width, int stream_height, std::string rtmp_server_adress, bool autostart){
    ROS_INFO("Initializing streamer node...");

    streamer::StreamerConfig streamer_config(frame_width, frame_height,
                                stream_width, stream_height,
                                fps, bitrate, "high", rtmp_server_adress);
    streamer_.enable_av_debug_log();
    streamer_.init(streamer_config);
    ROS_INFO("Initialized streamer node...");

    streamer_enabled_ = autostart;
}
void startThread(){
    current_thread = std::thread(&StreamerThread::streamFrame, this);
}
void streamFrame()
{   
    ros::Rate r(30);
    while(ros::ok()){
        std::shared_ptr<cv::Mat> image_ptr;
        if(image_ptr_queue_for_streamer.next(image_ptr)){
            streamer_.stream_frame(*image_ptr);
        }
        r.sleep();
    }
}
};

class ImagePublishThread{
private:
    
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    image_transport::ImageTransport it;

    std::thread current_thread;
    void leftCameraConnectCallback(){
        left_camera_enabled_ = true;
    }

    void leftCameraDisconnectCallback(){
        left_camera_enabled_ = false;
    }
    void rightCameraConnectCallback(){
        right_camera_enabled_ = true;
    }
    void rightCameraDisconnectCallback(){
        right_camera_enabled_ = false;
    }
public:
    bool left_camera_enabled_;
    bool right_camera_enabled_;
    ImagePublishThread(ros::NodeHandle& nh):it(nh){
        left_camera_enabled_ = false;
        right_camera_enabled_ = false;

        image_transport::SubscriberStatusCallback left_image_connect_cb =  boost::bind(&ImagePublishThread::leftCameraConnectCallback, this);
        image_transport::SubscriberStatusCallback left_image_disconnect_cb =  boost::bind(&ImagePublishThread::leftCameraDisconnectCallback, this);
        image_transport::SubscriberStatusCallback right_image_connect_cb =  boost::bind(&ImagePublishThread::rightCameraConnectCallback, this);
        image_transport::SubscriberStatusCallback right_image_disconnect_cb =  boost::bind(&ImagePublishThread::rightCameraDisconnectCallback, this);
        
        left_image_pub_ = it.advertise("/zed/left/image_raw_color", 1, left_image_connect_cb, left_image_disconnect_cb);
        right_image_pub_ = it.advertise("/zed/right/image_raw_color", 1, right_image_connect_cb, right_image_disconnect_cb);

    }
    void sendImageToTopic(){
        ros::Rate r(30);
        while(ros::ok()){
            
            std::shared_ptr<cv::Mat> image_ptr;
            if(left_image_ptr_queue_for_ros_topic.next(image_ptr)){
                sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image_ptr).toImageMsg();
                left_image_pub_.publish(left_msg);
            }
            
            if(right_image_ptr_queue_for_ros_topic.next(image_ptr)){
                sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *image_ptr).toImageMsg();
                right_image_pub_.publish(right_msg);
            }
            
            r.sleep();
        }
    }
    void startThread(){
        current_thread = std::thread(&ImagePublishThread::sendImageToTopic, this);
    }

};

class StreamerCameraNode{
private:
    StreamerThread streamer_thread;
    ImagePublishThread image_publish_thread;
    ZedWrapper zed_wrapper;
    int fps_;
public:
    StreamerCameraNode(ros::NodeHandle& nh, int fps, int bitrate, int stream_width, int stream_height, std::string rtmp_server_adress, bool autostart):streamer_thread(nh), image_publish_thread(nh){
        zed_wrapper.init();
        fps_ = fps;
        streamer_thread.init(bitrate, fps, zed_wrapper.getResolution().width, zed_wrapper.getResolution().height, stream_width, stream_height, rtmp_server_adress, autostart);
    }
    void start(){
        streamer_thread.startThread();
        image_publish_thread.startThread();

        ros::Rate r(fps_);
        while(ros::ok()){
            std::shared_ptr<cv::Mat> left_image_ptr, right_image_ptr;
            left_image_ptr = std::make_shared<cv::Mat>();
            right_image_ptr = std::make_shared<cv::Mat>();
            if(streamer_thread.streamer_enabled_ || image_publish_thread.left_camera_enabled_ || image_publish_thread.right_camera_enabled_){
                if(zed_wrapper.grab(*left_image_ptr, *right_image_ptr) < 0) continue;
                if(streamer_thread.streamer_enabled_){
                    image_ptr_queue_for_streamer.push(left_image_ptr);
                }
                if(image_publish_thread.left_camera_enabled_){
                    left_image_ptr_queue_for_ros_topic.push(left_image_ptr);
                }
                if(image_publish_thread.right_camera_enabled_){
                    right_image_ptr_queue_for_ros_topic.push(right_image_ptr);
                }
            }


            ros::spinOnce();
            r.sleep();
        }
    }
};



int main(int argc, char** argv){
    ros::init(argc, argv, "mm_video_streamer_node");
	ros::NodeHandle nh;
    std::string autostart_str;
    nh.getParam("autostart", autostart_str);
    bool autostart;
    if(autostart_str.compare("true"))
        autostart = true;
    else if(autostart_str.compare("false"))
        autostart = false;
    else
        ROS_ERROR("please set the correct parameter \"autostart\" to \"true\" or \"false\"");
	StreamerCameraNode streamer_camera_node(nh, 10, 300000, 640, 360, "rtmp://localhost/zed_left_camera/stream", autostart);
    
    streamer_camera_node.start();
}
