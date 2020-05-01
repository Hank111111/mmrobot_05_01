 // Standard includes
#include <stdio.h>
#include <string.h>

// OpenCV include (for display)
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

// Using std and sl namespaces
using namespace std;

class ZedWrapper{
private:
    cv::VideoCapture cap;
public:
    ZedWrapper():cap(0){
        sleep(1);
        if(!cap.isOpened()){
            ROS_ERROR("camera is not opened.");
        }
    };
    ~ZedWrapper(){
        cap.release();
    };
    void init(){
        
        // Set the video resolution to HD2K (2208*1242)
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 2208*2);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1242);
        cap.set(CV_CAP_PROP_FPS, 5);
        std::cout<<"use camera resolution: "<< 2208*2 << "x"<<1242<<endl;

        //discard 10 first unstable frames
        for(unsigned int i=0; i < 10; i++)
        {
            cv::Mat frame, left, right;
            // Get a new frame from camera
            cap >> frame;
        }

    }
    int grab(cv::Mat& left_image, cv::Mat& right_image){
        cv::Mat frame;
        cap >> frame;
        if(frame.cols != 2208*2 || frame.rows != 1242) return -1;

        // Extract left and right images from side-by-side
        left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        right_image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
    

        return 0; //image is grabbed;
    }
    int grab(cv::Mat& image, string side){
        cv::Mat frame;
        cap >> frame;
        if(frame.cols != 2208*2 || frame.rows != 1242) return -1;

        
        if(side == "left"){
            image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));

        }
        if(side == "right"){
            image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
        }
        return 0;
    }
};

