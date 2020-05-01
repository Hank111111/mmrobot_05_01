
// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include "streamer_nodelet.h"


// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(streamer_pkg::StreamerNodelet, nodelet::Nodelet)

namespace streamer_pkg
{

    StreamerNodelet::StreamerNodelet()
    {

        enabled_ = false;

    }

    void StreamerNodelet::init(ros::NodeHandle& nh, int stream_fps, int bitrate, int frame_width, int frame_height, int stream_width, int stream_height, std::string rtmp_server_adress){
        enable_streamer_subscriber_ = nh.subscribe("enable", 10, &StreamerNodelet::enableCallback, this);
        disable_streamer_subscriber_ = nh.subscribe("disable", 10, &StreamerNodelet::disableCallback, this);
        image_subscriber_ = nh.subscribe("image", 2, &StreamerNodelet::ImageCallback, this);
        frame_width_ = frame_width;
        frame_height_ = frame_height;
        StreamerConfig streamer_config(frame_width, frame_height,
                                    stream_width, stream_height,
                                    stream_fps, bitrate, "high", rtmp_server_adress);
        streamer_.enable_av_debug_log();
        streamer_.init(streamer_config);
    }

    void StreamerNodelet::onInit()
    {
        NODELET_DEBUG("Initializing streamer nodelet...");
        ros::NodeHandle nh = this->getMTPrivateNodeHandle();
        
        init(nh, 15, 500000, 2208, 1242, 640, 360, "rtmp://localhost/zed_left_camera/stream");
        NODELET_DEBUG("Initializing streamer initialised");
        
    }
    void StreamerNodelet::enableCallback(const std_msgs::EmptyConstPtr msg){
        enabled_ = true;
        ROS_INFO_STREAM("streamer nodelet has been enabled.]");
    }

    void StreamerNodelet::disableCallback(const std_msgs::EmptyConstPtr msg){
        enabled_ = false;
        ROS_INFO_STREAM("streamer nodelet has been disabled.]");
    }
    void StreamerNodelet::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {   
        if(enabled_){
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            assert(frame_width_ == cv_ptr->image.cols && frame_height_ == cv_ptr->image.rows && "you should initialize StreamerNodelet with correct frame image size");
            streamer_.stream_frame(cv_ptr->image);
        }    

    }
}