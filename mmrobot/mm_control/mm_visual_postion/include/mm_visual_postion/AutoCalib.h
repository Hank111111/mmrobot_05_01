#ifndef __AUTO_CALIB_H__
#define __AUTO_CALIB_H__
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_eigen/tf2_eigen.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <visp/vpCalibration.h>

#include "mm_visual_postion/utils/CheckCreatePath.h"
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
#include "mm_visual_postion/utils/StereoImageReceiver.h"
#include "mm_visual_postion/utils/ArmCommander.h"
#include "mm_visual_postion/utils/utils.h"


class AutoCalib{
/** 
 * Calibrate the intrinsic parameters of the camera and the camera-endeffector transformation.
 * To use this program, you should fix the chess board and only move the end effector of arm.
 */ 
private:

    bool getRawImages(cv::Mat& left, cv::Mat& right, ros::Time& stamp);
    bool getEndeffectorPose(cv::Mat& T_endeffector_to_base, const ros::Time& stamp);
    bool findCornersPrecisely(const cv::Mat& image, std::vector<cv::Point2f>& corners);
    void autoMoveToNewPose();
    void manuallyMoveToNewPose(std::string text, cv::Scalar color);
    void generateMoveCommand(std::vector<double>& xyzabc_cmd, bool always_center=false);
    bool canSeeChessboardAndIsSafe(Eigen::Matrix4d& T_endeffector_to_base);
    void generateRealGrid(int n_data, std::vector<std::vector<cv::Point3f> >& obj_points_vec);

    std::string save_path;
    StereoImageReceiver images_receiver;
    StereoCameraArmModel model;
    std::vector<cv::Mat> T_endeffector_to_base_vec;
    std::vector< std::vector<cv::Point2f> > left_corners_vec, right_corners_vec;
    std::vector<cv::Mat> left_r_vec, left_t_vec, right_r_vec, right_t_vec;
    cv::Size board_size;
    cv::Size left_image_size, right_image_size;
    double square_length;
    std::vector<Eigen::Vector4d> chess_board_corners; //position o~f 4 outer corners of the chessboard
    ArmCommander arm_commander;
    Eigen::Matrix4d T_endeffector_to_cam_estimate; 
    Eigen::Matrix4d T_chessboard_to_base_estimate;
    std::random_device rd;
    std::default_random_engine gen; 
    Eigen::Matrix4d origin_T_endeffector_to_base;   

public:
    AutoCalib(int cam_calib_num, int hand_eye_calib_num, double square_length, int board_width_num, int board_height_num, std::string save_path, ros::NodeHandle& n)
    :save_path(save_path), images_receiver(n,true), square_length(square_length), arm_commander(n), gen(rd()), nh(n){
    /**
     * the number of images you want to use.
     * If there is already a calibration file (save_path+"/stereo_cam_arm_model.yaml"), this programme can run completely automatically,
     * the exist calibration file will be used to roughly estimate the transformation from endeffector to camera, and the parameteres will be refined automatically.
     * 
     * If the calibration file doesn't exist, this programme can only run semi-automatically, you should move the arm manually.
     */
        tfListenerPtr = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
        board_size.width = board_width_num;
        board_size.height = board_height_num;
        chess_board_corners.resize(4);
        chess_board_corners[0] << 0, 0, 0, 1.0;
        chess_board_corners[1] << square_length * board_size.width, 0, 0, 1.0;
        chess_board_corners[2] << square_length * board_size.width, square_length * board_size.height, 0, 1.0;
        chess_board_corners[3] << 0, square_length * board_size.height, 0, 1.0;


        // try to load existed calibration file
        bool auto_move;
        if(model.load(save_path+"/stereo_cam_arm_model.yaml")){
            std::cout<<"old calibration file exists! This program will refine the parameters automatically."<<std::endl;
            auto_move = true;
        }
        else{
            auto_move = false;
            std::cout<<"cannot detect the old calibration file, this program can only be run in manual mode. You need to move the arm manually between the captured frames"<<std::endl;
        }

        if(!auto_move){
            collectData("calibrate camera params", CV_RGB(125, 0, 255), cam_calib_num);
            calibCamera();
            collectData("calibrate eye in hand params", CV_RGB(255, 0, 0), hand_eye_calib_num);
            calibHandEye();
        }
        else{
            
            manuallyMoveToNewPose("move to the origin pose", CV_RGB(125, 125, 0));
            cv::Mat T_endeffector_to_base_cv;
            while(!getEndeffectorPose(T_endeffector_to_base_cv, ros::Time(0))){}; // estimate the chessboard's position, which will be used to auto move
            cv2eigen(T_endeffector_to_base_cv, origin_T_endeffector_to_base);
 
            // load initial existed calibration file
            model.load(save_path+"/stereo_cam_arm_model.yaml");
            T_endeffector_to_cam_estimate = model.endeffector_to_cam_transform;
            collectData("calibrate camera params", CV_RGB(125, 0, 255), cam_calib_num,true);
            calibCamera();           
            
            cv::Mat T_cv_mat;
            RVecTVecToHomogenousMatrix(left_r_vec[0], left_t_vec[0], T_cv_mat);
            Eigen::Matrix4d T_chessboard_to_cam_estimate;
            cv2eigen(T_cv_mat, T_chessboard_to_cam_estimate);
            T_chessboard_to_base_estimate = T_chessboard_to_cam_estimate * T_endeffector_to_cam_estimate.inverse();
            collectData("calibrate eye in hand params", CV_RGB(255, 0, 0), hand_eye_calib_num, true);
            calibHandEye();
        }

        model.save(save_path+"/stereo_cam_arm_model.yaml");
    }
    void collectData(std::string show_text, cv::Scalar color, int collect_num, bool auto_move=false);
    void calibCamera();
    void calibHandEye();
private:
    tf2_ros::Buffer tfBuffer;
	std::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
    ros::NodeHandle nh;

};





#endif //__AUTO_CALIB_H__