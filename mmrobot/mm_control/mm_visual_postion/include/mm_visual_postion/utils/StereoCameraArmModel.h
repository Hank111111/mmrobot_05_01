#ifndef __STEREO_CAMERA_ARM_MODEL__
#define __STEREO_CAMERA_ARM_MODEL__

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/core/eigen.hpp>

#include <ros/package.h>
#include "ros/ros.h"
class CameraModel{
public:
    cv::Mat intrinsic_mat; //3x3
    cv::Mat distortion_mat; //1x5
    cv::Size image_size;
    Eigen::Matrix<double, 3, 4> projection_mat; // from 3d space to pixel space in calibrated image.

};

template <int M, int N>
cv::FileStorage& operator << (cv::FileStorage& fs, Eigen::Matrix<double, M,N>& mat){
    cv::Mat cv_mat;
    eigen2cv(mat, cv_mat);
    return fs<<cv_mat;
}
template <int M, int N> static inline
void operator >> (const cv::FileNode& n,  Eigen::Matrix<double, M,N>& mat)
{
    cv::Mat cv_mat;
    n >> cv_mat;
    cv2eigen(cv_mat, mat);
}


#define LEFT_CAMERA 0
#define RIGHT_CAMERA 1


class StereoCameraArmModel{
/**
 * High abstract model of tha stereocamera - arm system, including the saving/loading methods of the parameters of model.
 */
public:
    CameraModel left_camera;
    CameraModel right_camera;
    Eigen::Matrix4d right_cam_transform_mat; // right camera's transform matrix comparing to the origin( usually the left camera )
    Eigen::Matrix4d left_cam_transform_mat; // usually identity

    Eigen::Matrix4d endeffector_to_cam_transform; //endeffector's origin to camera's transform

    Eigen::Matrix3d essential_mat;
    Eigen::Matrix3d fundamental_mat;

    template <int M, int N>
    static std::vector<double> serializeEigenMatrix(const Eigen::Matrix<double, M, N>& mat){
        /**
         * serialize the eigen matrix to an array.
         * the input Eigen matrix is in column major, the ouput array is in row major.
         */ 
        std::vector<double> data;
        data.resize(M*N);
        Eigen::Matrix<double, M, N> mat_copy = mat;
        Eigen::Map<Eigen::Matrix<double, M*N, 1>> vec(mat_copy.transpose().data(), M*N);
        Eigen::Map<Eigen::Matrix<double, M*N, 1>>(&data[0], M*N) = vec;
        return data;
    }
    template <int M=4, int N=4>
    static Eigen::Matrix<double, M, N> deserializeEigenMatrix(const std::vector<double>& data){
        /**
         * deserialize the array to an eigen matrix.
         * the input array is in row major, the ouput Eigen matrix is in column major.
         */ 
        Eigen::Matrix<double, M, N> mat;
        std::vector<double> data_copy = data;
        Eigen::Map<Eigen::Matrix<double, N, M>> mat_map(&data_copy[0]);
        mat = mat_map.transpose();
        return mat;
    }
    
    void save(std::string file_name){
        cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
        fs << "left_camera_intrinsic" << left_camera.intrinsic_mat << "left_camera_distortion" << left_camera.distortion_mat
                  << "left_camera_image_size" << left_camera.image_size << "left_camera_projection" << left_camera.projection_mat
                  << "right_camera_intrinsic" << right_camera.intrinsic_mat << "right_camera_distortion" << right_camera.distortion_mat
                  << "right_camera_image_size" << right_camera.image_size << "right_camera_projection" << right_camera.projection_mat
                  << "right_cam_transform" << right_cam_transform_mat
                  << "left_cam_transform" << left_cam_transform_mat
                  << "endeffector_to_cam_transform" << endeffector_to_cam_transform
                  << "essential" << essential_mat
                  << "fundamental" << fundamental_mat;
    }   
    bool loadDefaultParams(){
        std::string path = ros::package::getPath("mm_visual_postion");
		std::string file_path = path + "/stereo_cam_arm_model.yaml";
        
        if (!load(file_path))
        {
            ROS_ERROR("arm-camera calibration yaml file's url is not valid [ %s ]", file_path.c_str());
            return false;
        }
        return true;
    }
    bool load(std::string file_name){
        cv::FileStorage fs(file_name, cv::FileStorage::READ);
        std::vector<double> read_data;

        if (!fs.isOpened())
        {
            std::cerr << "failed to open " << file_name << std::endl;
            return false;
        }
        fs["left_camera_intrinsic"] >> left_camera.intrinsic_mat;
        fs["left_camera_distortion"] >> left_camera.distortion_mat;
        fs["left_camera_image_size"] >> left_camera.image_size;

        fs["left_camera_projection"] >> left_camera.projection_mat;
        
        fs["right_camera_intrinsic"] >> right_camera.intrinsic_mat;
        fs["right_camera_distortion"] >> right_camera.distortion_mat;
        fs["right_camera_image_size"] >> right_camera.image_size;

        fs["right_camera_projection"] >> right_camera.projection_mat;

        fs["right_cam_transform"] >> right_cam_transform_mat;

        fs["left_cam_transform"] >> left_cam_transform_mat;

        fs["endeffector_to_cam_transform"] >> endeffector_to_cam_transform;

        fs["essential"] >> essential_mat;

        fs["fundamental"] >> fundamental_mat;

        //endeffector_to_cam_transform = endeffector_to_cam_transform.inverse().eval();
        
        return true;
    }
};
#endif //__STEREO_CAMERA_ARM_MODEL__