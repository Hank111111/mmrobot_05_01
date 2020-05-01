#ifndef __RECT_SWITCH_FINDER_TEST_GOLDEN_DATA__
#define __RECT_SWITCH_FINDER_TEST_GOLDEN_DATA__
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <Eigen/Dense>
class RectSwitchFinderTestGoldenData{
public:
    cv::Mat left_image;
    cv::Mat right_image;
    cv::Rect left_roi;
    cv::Rect right_roi;
    double width, height;
    Eigen::Matrix4d T_cam_to_obj;
    RectSwitchFinderTestGoldenData(int id){
        std::string path = ros::package::getPath("mm_visual_postion");
        std::string left_image_path = path + std::string("/test/rect_switch_finder/data/") + std::to_string(id) + "_left.bmp";
        std::string right_image_path = path + std::string("/test/rect_switch_finder/data/") + std::to_string(id) + "_right.bmp";
        left_image = cv::imread(left_image_path);
        right_image = cv::imread(right_image_path);
        if(left_image.empty()){
            ROS_ERROR("%s",(std::string("cannot load image: ")+left_image_path).c_str());
        }
        if(right_image.empty()){
            ROS_ERROR("%s",(std::string("cannot load image: ")+right_image_path).c_str());
        }
        if(id == 0){
            // xyzabc = 105.985, -24.2222, 345.361,  3.0777, -0.0080077, 0.0340119
            // 105.855, -24.158, 345.177, 3.07342, -0.0203885, 0.0335294
            double xyzabc[6] = {105.985, -24.2222, 345.361,  3.0777, -0.0080077, 0.0340119};
            T_cam_to_obj.setIdentity();
            T_cam_to_obj.block<3,3>(0,0) = (Eigen::AngleAxisd(xyzabc[3], Eigen::Vector3d::UnitX()) 
                                        * Eigen::AngleAxisd(xyzabc[4], Eigen::Vector3d::UnitY())
                                        * Eigen::AngleAxisd(xyzabc[5], Eigen::Vector3d::UnitZ())).toRotationMatrix(); 
            T_cam_to_obj(0,3) = xyzabc[0];
            T_cam_to_obj(1,3) = xyzabc[1];
            T_cam_to_obj(2,3) = xyzabc[2];
            Eigen::Matrix4d T_rot = Eigen::Matrix4d::Identity(); 
            T_rot.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
            T_cam_to_obj = T_cam_to_obj.eval() * T_rot;
            left_roi = cv::Rect(1378, 424, 300, 250);
            right_roi = cv::Rect(900, 476, 250, 250);
            width = 49;
            height = 49;
        }
    }
};



#endif //__RECT_SWITCH_FINDER_TEST_GOLDEN_DATA__

