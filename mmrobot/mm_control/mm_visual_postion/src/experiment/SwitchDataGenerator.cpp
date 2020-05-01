#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp>

#include <random>
#include <algorithm>
#include <ros/package.h>

#include <ctime>
#include <iomanip>
#include <iostream>

#include "mm_visual_postion/utils/ArmCommander.h"
#include "mm_visual_postion/utils/CheckCreatePath.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/utils/StereoImageReceiver.h"
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
#include "mm_visual_postion/experiment/TransformStampedSaver.h"
#include "mm_visual_postion/experiment/IteractFindCornersByHough.h"
#include "mm_visual_postion/experiment/DataGeneratorBase.h"

class PointSolver{
    // solve the point's 3d position by the multi views at different camera's pose.
private:
    Eigen::Matrix<double, 3, 4> left_projection_matrix_in_cam, right_projection_matrix_in_cam;
    Eigen::Matrix<double, 3, 4> left_projection_matrix_in_base, right_projection_matrix_in_base;
    cv::Mat mLeftM, mRightM;
    Eigen::Matrix4d T_endeffector_to_cam, T_cam_to_endeffector;
public:
    void setEndeffectorToCamTransform(Eigen::Matrix4d T_endeffector_to_cam_input){
        T_endeffector_to_cam = T_endeffector_to_cam_input;
        T_cam_to_endeffector = T_endeffector_to_cam.inverse();
    }
    void setProjectionMatrix(cv::Mat cv_left_projection_matrix, cv::Mat cv_right_projection_matrix){
        mLeftM = cv_left_projection_matrix;
        mRightM = cv_right_projection_matrix;
        cv2eigen(cv_left_projection_matrix, left_projection_matrix_in_cam);
	    cv2eigen(cv_right_projection_matrix, right_projection_matrix_in_cam);
    }
    void setProjectionMatrix(Eigen::Matrix<double, 3,4> left_projection_matrix_in_cam_input, Eigen::Matrix<double, 3,4> right_projection_matrix_in_cam_input){
        left_projection_matrix_in_cam = left_projection_matrix_in_cam_input;
	    right_projection_matrix_in_cam = right_projection_matrix_in_cam_input;
    }

    void solve(std::vector<cv::Point2d>& left_corners, std::vector<cv::Point2d>& right_corners, std::vector<geometry_msgs::TransformStamped>& vec_transform_endeffector_to_base, cv::Point3d& point, Eigen::Vector2d& avg_pixel_error){
        Eigen::Matrix<double, Eigen::Dynamic, 4> A; //A is the matrix that we will do SVD later
        A.resize(left_corners.size()*4, 4);
        Eigen::Matrix4d T_endeffector_to_base;
        for(unsigned int i=0; i<left_corners.size(); i++){ //fill A

            transformToMatrix(vec_transform_endeffector_to_base[i], T_endeffector_to_base);
            left_projection_matrix_in_base = left_projection_matrix_in_cam * T_cam_to_endeffector * T_endeffector_to_base;
            right_projection_matrix_in_base = right_projection_matrix_in_cam * T_cam_to_endeffector * T_endeffector_to_base;

            A.block<1,4>(i*4,0) = left_corners[i].y * left_projection_matrix_in_base.block<1,4>(2,0) - 1.0 * left_projection_matrix_in_base.block<1,4>(1,0);
            A.block<1,4>(i*4+1,0) = -left_corners[i].x * left_projection_matrix_in_base.block<1,4>(2,0) + 1.0 * left_projection_matrix_in_base.block<1,4>(0,0);
            A.block<1,4>(i*4+2,0) = right_corners[i].y * right_projection_matrix_in_base.block<1,4>(2,0) - 1.0 * right_projection_matrix_in_base.block<1,4>(1,0);
            A.block<1,4>(i*4+3,0) = -right_corners[i].x * right_projection_matrix_in_base.block<1,4>(2,0) + 1.0 * right_projection_matrix_in_base.block<1,4>(0,0);
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
        std::cout<<"A "<<A<<std::endl;
        std::cout<<svd.nonzeroSingularValues()<<std::endl;
        Eigen::Vector4d solution =  svd.matrixV().block<4,1>(0, svd.nonzeroSingularValues()-1); //solution is the last colom of V

        // normalize
        solution(0) = solution(0) / solution(3);
        solution(1) = solution(1) / solution(3);
        solution(2) = solution(2) / solution(3);
        solution(3) = 1.0;

        point.x = solution(0);
        point.y = solution(1);
        point.z = solution(2);
	    
        avg_pixel_error(0) = 0;
        avg_pixel_error(1) = 0;
        //estimate average pixel error
        for(unsigned int i=0; i<left_corners.size(); i++){ 
            transformToMatrix(vec_transform_endeffector_to_base[i], T_endeffector_to_base);
            left_projection_matrix_in_base = left_projection_matrix_in_cam * T_cam_to_endeffector * T_endeffector_to_base;
            right_projection_matrix_in_base = right_projection_matrix_in_cam * T_cam_to_endeffector * T_endeffector_to_base;
            Eigen::Vector3d left_pixel_pos = left_projection_matrix_in_base * solution;
            Eigen::Vector3d right_pixel_pos = right_projection_matrix_in_base * solution;
            normalizeVector3d(left_pixel_pos);
            normalizeVector3d(right_pixel_pos);
            avg_pixel_error(0) += fabs(left_pixel_pos(0) - left_corners[i].x) + fabs(right_pixel_pos(0) - right_corners[i].x);
            avg_pixel_error(1) += fabs(left_pixel_pos(1) - left_corners[i].y) + fabs(right_pixel_pos(1) - right_corners[i].y);
        
        
        
        }
        avg_pixel_error(0) = avg_pixel_error(0) / left_corners.size() /2;
        avg_pixel_error(1) = avg_pixel_error(1) / left_corners.size() /2;
        
    }
};

class SwitchDatasetGenerator:public DataGeneratorBase{
private:

    std::ofstream test_image_csv_file;
    std::ofstream gt_image_csv_file, gt_result_csv_file;


    std::vector<std::vector<cv::Point2f>> gt_left_corners, gt_right_corners; //ground truth corners in several images
    std::vector<cv::Point3d> gt_3d_points; //ground truth corners 3d position in base coordinate

public:
    SwitchDatasetGenerator(ros::NodeHandle& n_input, std::string save_dir_input): 
        DataGeneratorBase(n_input, save_dir_input)
    {
        test_image_csv_file.open(save_dir + std::string("/image.csv"));
        gt_image_csv_file.open(save_dir + std::string("/ground_truth/image.csv"));
    }



    void manuallyGetGroundTruthData(int capture_num){
        gt_left_corners.clear();
        gt_right_corners.clear();
        // capture data
        for(int capture_i = 0; capture_i < capture_num; capture_i ++){


            // manually move robot to position  then press any key
            waitForManuallyMove("Move to proper position");

            geometry_msgs::TransformStamped transform_endeffector_to_base;
            grabImagesAndTransform(latest_left_image, latest_right_image, transform_endeffector_to_base ,latest_update_time);
            
            // save images
            std::string left_image_save_path = save_dir + std::string("/ground_truth/") + std::to_string(capture_i) + std::string("_left.bmp");
            std::string right_image_save_path = save_dir + std::string("/ground_truth/") + std::to_string(capture_i) + std::string("_right.bmp");
            imwrite( left_image_save_path, latest_left_image );
            imwrite( right_image_save_path, latest_right_image );

            writeLine(gt_image_csv_file,capture_i, left_image_save_path, right_image_save_path);

            // save transform
            gt_transform_data.push_back(transform_endeffector_to_base);

            // manually point out corners
            IteractFindCornersByHough interact_find_corners_left(latest_left_image);
            interact_find_corners_left.start();
            
            gt_left_corners.push_back(std::vector<cv::Point2f>());
            interact_find_corners_left.getConers(gt_left_corners[gt_left_corners.size()-1]);

            // manually point out corners
            IteractFindCornersByHough interact_find_corners_right(latest_right_image);
            interact_find_corners_right.start();
            
            gt_right_corners.push_back(std::vector<cv::Point2f>());
            interact_find_corners_right.getConers(gt_right_corners[gt_right_corners.size()-1]);        
        }
        gt_image_csv_file.close();
        writeTransformData(gt_transform_data, "ground_truth_transform");

        // save ground left/right image corners
        std::ofstream gt_image_corners_csv_file;
        gt_image_corners_csv_file.open(save_dir + std::string("/ground_truth/ground_truth_image_corners.csv"));
        for(unsigned int image_i=0; image_i<gt_left_corners.size(); image_i ++){
            gt_image_corners_csv_file << gt_left_corners[image_i][0].x << ","<< gt_left_corners[image_i][0].y <<","
                            << gt_left_corners[image_i][1].x << ","<< gt_left_corners[image_i][1].y <<","
                            << gt_left_corners[image_i][2].x << ","<< gt_left_corners[image_i][2].y <<","
                            << gt_left_corners[image_i][3].x << ","<< gt_left_corners[image_i][3].y <<","
                            << gt_right_corners[image_i][0].x << ","<< gt_right_corners[image_i][0].y <<","
                            << gt_right_corners[image_i][1].x << ","<< gt_right_corners[image_i][1].y <<","
                            << gt_right_corners[image_i][2].x << ","<< gt_right_corners[image_i][2].y <<","
                            << gt_right_corners[image_i][3].x << ","<< gt_right_corners[image_i][3].y <<"\n";
        }
        gt_image_corners_csv_file.close();
    }

    void debugInput(){

        gt_left_corners.clear();
        
        gt_right_corners.clear();
        for(unsigned int image_i=0; image_i<2; image_i ++){
            geometry_msgs::TransformStamped transform_endeffector_to_base;
            do{
                ros::spinOnce();
                image_receiver.getLatestImages(latest_left_image, latest_right_image, latest_update_time);
            }
            while(!getTransform(transform_endeffector_to_base, latest_update_time));
            gt_transform_data.push_back(transform_endeffector_to_base);


            gt_left_corners.push_back(std::vector<cv::Point2f>(4));
            gt_right_corners.push_back(std::vector<cv::Point2f>(4));

            gt_left_corners[image_i][0].x = 1013;
            gt_left_corners[image_i][0].y = 586;
            gt_left_corners[image_i][1].x = 1108;
            gt_left_corners[image_i][1].y = 586;
            gt_left_corners[image_i][2].x = 1108;
            gt_left_corners[image_i][2].y = 680;
            gt_left_corners[image_i][3].x = 1013;
            gt_left_corners[image_i][3].y = 680;
            gt_right_corners[image_i][0].x = 761;
            gt_right_corners[image_i][0].y = 623;
            gt_right_corners[image_i][1].x = 856;
            gt_right_corners[image_i][1].y = 623;
            gt_right_corners[image_i][2].x = 856;
            gt_right_corners[image_i][2].y = 717;
            gt_right_corners[image_i][3].x = 762;
            gt_right_corners[image_i][3].y = 717;        
        }
    }

    void computeGroundTruth(){ //left_up, right_up, right_down, left_down
        // you need to call manuallyGetGroundTruthData firstly
        PointSolver point_solver;
        gt_3d_points.resize(4);
        std::vector<Eigen::Vector2d> avg_pixel_error_vec;
        avg_pixel_error_vec.resize(4);

        point_solver.setEndeffectorToCamTransform(T_endeffector_to_cam);
        point_solver.setProjectionMatrix(left_projection_matrix_in_cam, right_projection_matrix_in_cam);

        for(unsigned int corner_i=0; corner_i < 4; corner_i ++) //left_up, right_up, right_down, left_down
        {
            std::vector<cv::Point2d> left_pixel_points, right_pixel_points;
            left_pixel_points.resize(gt_left_corners.size());
            right_pixel_points.resize(gt_right_corners.size());

            for(unsigned int pair_i =0; pair_i < gt_left_corners.size(); pair_i ++){
                left_pixel_points[pair_i] = gt_left_corners[pair_i][corner_i];
                right_pixel_points[pair_i] = gt_right_corners[pair_i][corner_i];
            }
            
            point_solver.solve(left_pixel_points, right_pixel_points, gt_transform_data, gt_3d_points[corner_i], avg_pixel_error_vec[corner_i]);
        }

        std::cout<<"ground truth corners in base coordinate: "<<std::endl;
        std::cout<<"    left_up:      "<<gt_3d_points[0]<<std::endl;
        std::cout<<"    right_up:     "<<gt_3d_points[1]<<std::endl;
        std::cout<<"    right_down:   "<<gt_3d_points[2]<<std::endl;
        std::cout<<"    left_down:    "<<gt_3d_points[3]<<std::endl;

        // save ground truth corners
        gt_result_csv_file.open(save_dir + std::string("/ground_truth/result.csv"));
        for(unsigned int corner_i=0; corner_i<4; corner_i ++){
            gt_result_csv_file << gt_3d_points[corner_i].x<<","<<gt_3d_points[corner_i].y<<","<<gt_3d_points[corner_i].z<<"\n";
        }
        gt_result_csv_file.close();
        
        std::ofstream result_error_txt;
        result_error_txt.open(save_dir + std::string("/ground_truth/result_error.txt"));
        result_error_txt<<"estimate pixel error: "<<"\n";
        result_error_txt<<"    left up:    x: "<<avg_pixel_error_vec[0](0)<<"   y: "<<avg_pixel_error_vec[0](1)<<"\n";
        result_error_txt<<"    right up:   x: "<<avg_pixel_error_vec[1](0)<<"   y: "<<avg_pixel_error_vec[1](1)<<"\n";
        result_error_txt<<"    right down: x: "<<avg_pixel_error_vec[2](0)<<"   y: "<<avg_pixel_error_vec[2](1)<<"\n";
        result_error_txt<<"    left down:  x: "<<avg_pixel_error_vec[3](0)<<"   y: "<<avg_pixel_error_vec[3](1)<<"\n";  
        result_error_txt.close();
        
        //print result errors:
        std::cout<<"estimate pixel error: "<<std::endl;
        std::cout<<"    left up:    x: "<<avg_pixel_error_vec[0](0)<<"   y: "<<avg_pixel_error_vec[0](1)<<std::endl;
        std::cout<<"    right up:   x: "<<avg_pixel_error_vec[1](0)<<"   y: "<<avg_pixel_error_vec[1](1)<<std::endl;
        std::cout<<"    right down: x: "<<avg_pixel_error_vec[2](0)<<"   y: "<<avg_pixel_error_vec[2](1)<<std::endl;
        std::cout<<"    left down:  x: "<<avg_pixel_error_vec[3](0)<<"   y: "<<avg_pixel_error_vec[3](1)<<std::endl;
    }


    virtual bool cmdIsValid(Eigen::Matrix4d& sample_T_endeffector_to_base){
        // check whether all of the four corners can be seen from two images by this command
        std::vector<Eigen::Vector4d> check_points_in_base(4);
        for(unsigned int corner_i = 0; corner_i < 4; corner_i ++){
            check_points_in_base[corner_i] << gt_3d_points[corner_i].x , gt_3d_points[corner_i].y , gt_3d_points[corner_i].z , 1.0;
        }

        if(canSeePointsAndIsSafe(sample_T_endeffector_to_base, 50, 200, check_points_in_base))
            return true;
        else return false;
    }
    void generateDataset(int sample_num){
        std::vector<double> xyzabc_cmd;
        int sample_i = 0;
        generateMoveCommand(gt_transform_data[0], 200, 200, 200, 0.01, xyzabc_cmd);

        arm_commander.moveByBaseSpacePlanning(xyzabc_cmd[0] / 1000., xyzabc_cmd[1]/ 1000., xyzabc_cmd[2]/ 1000., 
                                xyzabc_cmd[3], xyzabc_cmd[4], xyzabc_cmd[5]);

        while(ros::ok()){
            bool time_out = arm_commander.isTimeOut();
            if(time_out){
            generateMoveCommand(gt_transform_data[0], 200, 200, 200, 0.01, xyzabc_cmd);
                arm_commander.moveByBaseSpacePlanning(xyzabc_cmd[0] / 1000., xyzabc_cmd[1]/ 1000., xyzabc_cmd[2]/ 1000., 
                                xyzabc_cmd[3], xyzabc_cmd[4], xyzabc_cmd[5]);
            } 
            if(arm_commander.isArrived()){
                // arm arrives the cmd position
                // capture the image
                sleep(1); // wait for camera to stablize

                while(grabTestData(sample_i) != 0);
                sample_i += 1;
                if(sample_i >= sample_num) break;
                generateMoveCommand(gt_transform_data[0], 200, 200, 200, 0.01, xyzabc_cmd);
                arm_commander.moveByBaseSpacePlanning(xyzabc_cmd[0]/ 1000., xyzabc_cmd[1]/ 1000., xyzabc_cmd[2]/ 1000., 
                            xyzabc_cmd[3], xyzabc_cmd[4], xyzabc_cmd[5]);
                sleep(1); //wait for arm to move
            }
            sleep(1);
            ros::spinOnce();
        }
        test_image_csv_file.close();
        writeTransformData(test_transform_data, "test_transform");
    }

};


int main(int argc, char** argv){
    ros::init(argc, argv, "dataset_generator_node");
	ros::NodeHandle nh;
    std::string path = ros::package::getPath("mm_visual_postion");

    // get current time string as the folder's name
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d_%m_%Y_%H_%M_%S");
    auto time_str = oss.str();

    path = path +std::string("/")+time_str +std::string("/");


    if(!isDirExist(path+"/dataset/ground_truth")){
        makePath(path+"/dataset/ground_truth");
    }
    SwitchDatasetGenerator dataset_generator(nh, path+"/dataset");

    // compute ground truth pose of object from base
    dataset_generator.manuallyGetGroundTruthData(2);
    //dataset_generator.debugInput();
    dataset_generator.computeGroundTruth();

    // grab test dataset
    dataset_generator.generateDataset(200);

    return 0;
}