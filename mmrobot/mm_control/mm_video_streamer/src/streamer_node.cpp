#include "streamer/streamer.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <chrono>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
class StreamerNode
{
    public:
        StreamerNode();
        void init(ros::NodeHandle& nh, int stream_fps, int bitrate, int frame_width, int frame_height, int stream_width, int stream_height, std::string rtmp_server_adress, bool autostart);

    private:
        int stream_fps_set;
        size_t streamed_frames;
        double avg_frame_time;
        streamer::MovingAverage moving_average;
        std::chrono::high_resolution_clock clk;
        std::chrono::high_resolution_clock::time_point time_start;
        std::chrono::high_resolution_clock::time_point time_stop;
        std::chrono::high_resolution_clock::time_point time_prev;
        std::chrono::duration<double> elapsed_time;
        std::chrono::duration<double> frame_time;

        void enableCallback(const std_msgs::EmptyConstPtr msg);
        void disableCallback(const std_msgs::EmptyConstPtr msg);
        void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

        streamer::Streamer streamer_;
        int frame_height_, frame_width_;
        ros::Subscriber enable_streamer_subscriber_, disable_streamer_subscriber_;
        ros::Subscriber image_subscriber_;
        bool enabled_;
};
StreamerNode::StreamerNode():moving_average(10)
{
    streamed_frames = 0;
    enabled_ = false;

}

void StreamerNode::init(ros::NodeHandle& nh, int stream_fps, int bitrate, int frame_width, int frame_height, int stream_width, int stream_height, std::string rtmp_server_adress, bool autostart){
    ROS_INFO("Initializing streamer node...");
    stream_fps_set = stream_fps;
    enable_streamer_subscriber_ = nh.subscribe("enable", 10, &StreamerNode::enableCallback, this);
    disable_streamer_subscriber_ = nh.subscribe("disable", 10, &StreamerNode::disableCallback, this);
    image_subscriber_ = nh.subscribe("image", 2, &StreamerNode::ImageCallback, this);
    frame_width_ = frame_width;
    frame_height_ = frame_height;
    streamer::StreamerConfig streamer_config(frame_width, frame_height,
                                stream_width, stream_height,
                                stream_fps, bitrate, "high", rtmp_server_adress);
    streamer_.enable_av_debug_log();
    streamer_.init(streamer_config);
    ROS_INFO("Initialized streamer node...");
    time_start = clk.now();
    time_stop = time_start;
    time_prev = time_start;
    elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start);
    frame_time = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_prev);

    enabled_ = autostart;
    ros::spin();
}
void StreamerNode::enableCallback(const std_msgs::EmptyConstPtr msg){
    enabled_ = true;
    ROS_INFO_STREAM("streamer nodelet has been enabled.]");
}

void StreamerNode::disableCallback(const std_msgs::EmptyConstPtr msg){
    enabled_ = false;
    ROS_INFO_STREAM("streamer nodelet has been disabled.]");
}
void StreamerNode::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
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
        time_stop = clk.now();
        elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start);
        frame_time = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_prev);
        streamed_frames++;
        moving_average.add_value(frame_time.count());
        avg_frame_time = moving_average.get_average();
        streamer::add_delay(streamed_frames, stream_fps_set, elapsed_time.count(), avg_frame_time);
    }    
}

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
	StreamerNode streamer;
    streamer.init(nh, 13, 500000, 2208, 1242, 640, 360, "rtmp://localhost/zed_left_camera/stream", autostart);
    
}
