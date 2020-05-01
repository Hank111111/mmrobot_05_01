#ifndef __RECT_SWITCH_FINDER_TEST_GOLDEN_DATA__
#define __RECT_SWITCH_FINDER_TEST_GOLDEN_DATA__
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <Eigen/Dense>
class HandTrunkSolverTestGoldenData{
public:
    cv::Mat left_image;
    cv::Mat right_image;
    cv::Rect left_roi;
    cv::Rect right_roi;
    double radius;
    Eigen::Matrix4d T_cam_to_obj;
    HandTrunkSolverTestGoldenData(int id){
        std::string path = ros::package::getPath("mm_visual_postion");
        std::string left_image_path = path + std::string("/test/hand_trunk_solver/data/") + std::to_string(id) + "_left.bmp";
        std::string right_image_path = path + std::string("/test/hand_trunk_solver/data/") + std::to_string(id) + "_right.bmp";
        left_image = cv::imread(left_image_path);
        right_image = cv::imread(right_image_path);
        if(left_image.empty()){
            ROS_ERROR("%s",(std::string("cannot load image: ")+left_image_path).c_str());
        }
        if(right_image.empty()){
            ROS_ERROR("%s",(std::string("cannot load image: ")+right_image_path).c_str());
        }
        if(id == 30){
            T_cam_to_obj << -0.675434,  -0.737335, -0.0112186,    104.931,
                            -0.735929,   0.673023,    0.07382,    1.49112,
                            -0.0468797,  0.0581167,  -0.997208,    418.741,
                                    0,          0,          0 ,         1;
            
            left_roi = cv::Rect(1320, 540, 300, 250);
            right_roi = cv::Rect(910, 582, 250, 250);
            radius = 28;
        }
    }
};

class HandTrunkSolverRealTestGoldenData{
public:
    cv::Mat left_image;
    cv::Mat right_image;
    cv::Rect left_roi;
    cv::Rect right_roi;
    double radius;
    Eigen::Matrix4d T_cam_to_obj;
    HandTrunkSolverRealTestGoldenData(int num){
        std::string path = ros::package::getPath("mm_visual_postion");
        std::string left_image_path = path + std::string("/test/hand_trunk_solver/data/5_1_4/left_") + std::to_string(num)+".png";
        std::string right_image_path = path + std::string("/test/hand_trunk_solver/data/5_1_4/right_") + std::to_string(num)+".png";
        left_image = cv::imread(left_image_path);
        right_image = cv::imread(right_image_path);
        if(left_image.empty()){
            ROS_ERROR("%s",(std::string("cannot load image: ")+left_image_path).c_str());
        }
        if(right_image.empty()){
            ROS_ERROR("%s",(std::string("cannot load image: ")+right_image_path).c_str());
        }

            T_cam_to_obj << -0.675434,  -0.737335, -0.0112186,    104.931,
                            -0.735929,   0.673023,    0.07382,    1.49112,
                            -0.0468797,  0.0581167,  -0.997208,    418.741,
                                    0,          0,          0 ,         1;
            
            right_roi = cv::Rect(1320, 356, 400, 300);
            left_roi = cv::Rect(826, 408, 400, 300);
            radius = 20;
        
    }
};


#endif //__RECT_SWITCH_FINDER_TEST_GOLDEN_DATA__

