#ifndef __DATA_PROCESSOR_BASE_H__
#define __DATA_PROCESSOR_BASE_H__
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <iostream>
#include <sstream>

#include "mm_visual_postion/experiment/TransformStampedSaver.h"
#include "mm_visual_postion/utils/StereoImageReceiver.h"
#include "mm_visual_postion/utils/CheckCreatePath.h"
class DatasetProcessorBase{
protected:
    ros::NodeHandle nh;

    std::string read_dir;
    std::vector<geometry_msgs::TransformStamped> transform_vec;
    std::shared_ptr<StereoCameraArmModel> model_ptr;

    void readTransformDataFile(const std::string& file_name, std::vector<geometry_msgs::TransformStamped>& transform_vec){
        std::string file_path = read_dir + std::string("/")+file_name;
        std::ifstream finput(file_path, std::ios::in | std::ios::binary);
        if( !finput ){
            ROS_ERROR("cannot open the file %s", file_path.c_str());
            exit(-1);
        }
        // attention: the data in vector cannot include pointer
        geometry_msgs::TransformStamped temp;
        TransformStampedSaver temp_saver;
        while( finput.read(reinterpret_cast<char*>(&temp_saver),sizeof(TransformStampedSaver)) ) {
            temp_saver.toTransform(temp);
            transform_vec.push_back(temp); 
        }
        finput.close();
    }

    DatasetProcessorBase(ros::NodeHandle& n, std::string read_dir_input): nh(n)
    {
        read_dir = read_dir_input;
        StereoImageReceiver receiver(n); // to get the model ptr
        receiver.getArmCameraModel(model_ptr);
        readTransformDataFile("test_transform.dat",transform_vec);
        test_size = transform_vec.size();
    }


    void loadImage(int id, cv::Mat& left_image, cv::Mat& right_image){
        std::stringstream ss;
        ss << read_dir<<"/"<<id;
        std::string left_image_path = ss.str() + std::string("_left.bmp");
        std::string right_image_path = ss.str() + std::string("_right.bmp");
        left_image = cv::imread(left_image_path);
        right_image = cv::imread(right_image_path);

        if (left_image.cols == 0 || left_image.rows == 0)
            ROS_ERROR("cannot load left image with id: %d", id);
        if (right_image.cols == 0 || right_image.rows == 0)
            ROS_ERROR("cannot load right image with id: %d", id);
    }

    void saveImage(int id, std::string method_name, cv::Mat& left_image, cv::Mat& right_image){
        std::stringstream ss;
        ss << read_dir<<"/processed/" << method_name ;
        if(!isDirExist(ss.str())){
            makePath(ss.str());
        }
        ss << "/"<<id;

        std::string left_image_path = ss.str() + std::string("_left.bmp");
        std::string right_image_path = ss.str() + std::string("_right.bmp");
        //cv::imwrite(left_image_path, left_image);
        //cv::imwrite(right_image_path, right_image);
    }
public:
    int test_size;

};

#endif