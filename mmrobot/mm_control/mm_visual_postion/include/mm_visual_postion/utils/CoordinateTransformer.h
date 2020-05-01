#ifndef __COORDINATE_TRANSFORMER_H__
#define __COORDINATE_TRANSFORMER_H__
#include <Eigen/Dense>
#include "tf2_eigen/tf2_eigen.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
void createTransformationMatrix(double trans_x, double trans_y, double trans_z, double rot_x, double rot_y, double rot_z, Eigen::Matrix4d& trans);
void createTransformationMatrix(double trans_x, double trans_y, double trans_z, 
                                double q_x, double q_y, double q_z, double q_w,
                                Eigen::Matrix4d& trans);
void createTransformationMatrix(const std::vector<double>& xyzabc, Eigen::Matrix4d& trans);
class CoordinateTransformer{
private:

public:
    //unit: milli-meter and radian

    Eigen::Matrix4d T_cam_to_endeffector; // invariant
    Eigen::Matrix4d T_endeffector_to_cam; // invariant
    
    Eigen::Matrix4d T_base_to_endeffector; // change this
    Eigen::Matrix4d T_endeffector_to_base; // automatically update
    
    Eigen::Matrix4d T_cam_to_object; //object in cam coord
    cv::Point2d pixel_pos;

    Eigen::Matrix4d T_cam_to_base; // automatically update
    //Eigen::Matrix4d T_object_to_base;

    Eigen::Matrix4d T_base_to_object;

    // transform link: obj--?-->cam--(fix)-->endeffector--(know)-->base
    Eigen::Matrix<double, 3, 4> cam_projection_mat;
    bool initialized;
    CoordinateTransformer():initialized(false){};
    void init(Eigen::Matrix4d T_endeffector_to_cam, Eigen::Matrix<double, 3, 4> cam_projection_mat_);
    CoordinateTransformer(Eigen::Matrix4d T_endeffector_to_cam, Eigen::Matrix<double, 3, 4> cam_projection_mat);

    void setTransEndEffectorToBase(const geometry_msgs::TransformStamped& transform_endeffector_to_base);
    void setTransCamToObject(const Eigen::Matrix4d& trans_cam_to_object);
    void setTransCamToObject(const std::vector<double>& xyzabc_in_cam);
    void getPixelPosFromPosInBase(const double x_base, const double y_base, const double z_base, cv::Point2d& pixel_pos);
    void getPixelPosFromPosInBase(const Eigen::Vector4d& pos_in_base_homogene, cv::Point2d& pixel_pos);

    void getPixelPosFromPosInObject(const Eigen::Vector4d& pos_in_object_homogene, cv::Point2d& pixel_pos);
    void getPosInBaseFromPoseInObject(const Eigen::Vector4d& pos_in_object_homogene, Eigen::Vector4d& pos_in_base_homogene);
    void getPosInBaseFromPosInCam(const Eigen::Vector4d& pos_in_cam_homogene, Eigen::Vector4d& pos_in_base_homogene);
    void getObjPoseInBaseFromObjPoseInCam(const std::vector<double>& xyzabc_in_cam, std::vector<double>& xyzabc_in_base);
    void getObjPoseInCamFromObjPoseInBase(const std::vector<double>& xyzabc_in_base, std::vector<double>& xyzabc_in_cam);
    
    void setTransBaseToObject(const std::vector<double>& xyzabc_in_base);
    void setTransBaseToObject(double trans_x, double trans_y, double trans_z, 
                                double rot_x, double rot_y, double rot_z);
    void setTransBaseToObject(double trans_x, double trans_y, double trans_z,
                                    double q_x, double q_y, double q_z, double q_w);

    void setCamEndeffectorStaticBroadCaster();
    void pubCamToObjTransform();
    void pubCamToObjStaticTransform();
};

#endif //__COORDINATE_TRANSFORMER_H__