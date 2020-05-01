#include "streamer/streamer.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <chrono>

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
using namespace streamer;

namespace streamer_pkg
{

    class StreamerNodelet : public nodelet::Nodelet
    {
        public:
            StreamerNodelet();
            virtual void onInit();
            void init(ros::NodeHandle& nh, int stream_fps, int bitrate, int frame_width, int frame_height, int stream_width, int stream_height, std::string rtmp_server_adress);

        private:

            void enableCallback(const std_msgs::EmptyConstPtr msg);
            void disableCallback(const std_msgs::EmptyConstPtr msg);
            void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

            Streamer streamer_;
            int frame_height_, frame_width_;
            ros::Subscriber enable_streamer_subscriber_, disable_streamer_subscriber_;
            ros::Subscriber image_subscriber_;
            bool enabled_;
    };

}
PLUGINLIB_EXPORT_CLASS(streamer_pkg::StreamerNodelet,
                       nodelet::Nodelet);
/*
int main(int argc, char *argv[])
{
    if(argc != 2) {
        printf("must provide one command argument with the video file or stream to open\n");
        return 1;
    }
    std::string video_fname;
    video_fname = std::string(argv[1]);
    cv::VideoCapture video_capture;
    bool from_camera = false;
    if(video_fname == "0") {
        video_capture = cv::VideoCapture(0);
        from_camera = true;
    } else {
        video_capture=  cv::VideoCapture(video_fname, cv::CAP_FFMPEG);
    }


    if(!video_capture.isOpened()) {
        fprintf(stderr, "could not open video %s\n", video_fname.c_str());
        video_capture.release();
        return 1;
    }

    int cap_frame_width = video_capture.get(cv::CAP_PROP_FRAME_WIDTH);
    int cap_frame_height = video_capture.get(cv::CAP_PROP_FRAME_HEIGHT);

    int cap_fps = video_capture.get(cv::CAP_PROP_FPS);
    printf("video info w = %d, h = %d, fps = %d\n", cap_frame_width, cap_frame_height, cap_fps);

    int stream_fps = cap_fps;

    int bitrate = 500000;
    Streamer streamer;
    StreamerConfig streamer_config(cap_frame_width, cap_frame_height,
                                   640, 360,
                                   stream_fps, bitrate, "high", "rtmp://localhost/zed_left_camera/stream");

    streamer.enable_av_debug_log();
    streamer.init(streamer_config);

    size_t streamed_frames = 0;

    std::chrono::high_resolution_clock clk;
    std::chrono::high_resolution_clock::time_point time_start = clk.now();
    std::chrono::high_resolution_clock::time_point time_stop = time_start;
    std::chrono::high_resolution_clock::time_point time_prev = time_start;

    MovingAverage moving_average(10);
    double avg_frame_time;

    cv::Mat read_frame;
    cv::Mat proc_frame;
    bool ok = video_capture.read(read_frame);

    std::chrono::duration<double> elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start);
    std::chrono::duration<double> frame_time = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_prev);

    while(ok) {
        process_frame(read_frame, proc_frame);
        if(!from_camera) {
            streamer.stream_frame(proc_frame);
        } else {
            streamer.stream_frame(proc_frame, frame_time.count()*streamer.inv_stream_timebase);
        }

        time_stop = clk.now();
        elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_start);
        frame_time = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop - time_prev);

        if(!from_camera) {
            streamed_frames++;
            moving_average.add_value(frame_time.count());
            avg_frame_time = moving_average.get_average();
            add_delay(streamed_frames, stream_fps, elapsed_time.count(), avg_frame_time);
        }

        ok = video_capture.read(read_frame);
        time_prev = time_stop;
    }
    video_capture.release();

    return 0;
}
*/