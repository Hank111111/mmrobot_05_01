#ifndef __VISUAL_POSITION_UTILS_H__
#define __VISUAL_POSITION_UTILS_H__
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "tf2_eigen/tf2_eigen.h"
#include <opencv2/core/eigen.hpp>
#include <ostream>
#include "mm_visual_postion/ROI.h"
#include "mm_robot_decision/pose4.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/utils/ObjectDefinition.h"
std::ostream &operator << (std::ostream &f, const Eigen::Vector4d &vec);
void normalizeVector3d(Eigen::Vector3d& vec);

void transformToMatrix(const geometry_msgs::TransformStamped& transform, Eigen::Matrix4d& T);
void transformToMatrix(const geometry_msgs::Transform& transform, Eigen::Matrix4d& T);

void transformToCvMat(geometry_msgs::TransformStamped& transform, cv::Mat& T);
void transformToCvMat(geometry_msgs::Transform& transform, cv::Mat& T);

void translateTbase2endeffectorToAngleAxisCmd(Eigen::Matrix4d& T_base_to_endeffector, std::vector<double>& XYZRxRyRz);
bool isQuaternionSimilar(Eigen::Quaterniond& q1, Eigen::Quaterniond& q2, const double epsilon);

void inline translateToCvPoint(Eigen::Vector3d& point_in_image, cv::Point2d& point_cv){
    normalizeVector3d(point_in_image);
    point_cv.x = point_in_image(0);
    point_cv.y = point_in_image(1);
}

void inline getTransformMatrix(const Eigen::Vector3d& translate, const Eigen::Quaterniond& q, Eigen::Matrix4d& T){
    T.setIdentity();
    T.block<3,3>(0,0) = Eigen::Matrix3d(q);
    T.block<3,1>(0,3) = translate;
}
void inline getTranslateAndQuat(const Eigen::Matrix4d& T, Eigen::Vector3d& translate, Eigen::Quaterniond& q){
    translate = T.block<3,1>(0,3);
    q = T.block<3,3>(0,0);
}

void inline getPlane(const Eigen::Matrix4d& T_presented_frame_to_obj, Eigen::Vector4d& center, Eigen::Vector4d& plane_presented_frame_to_obj){
    Eigen::Vector4d normal;
    normal << 0,0,1,0; //z-axis
    Eigen::Vector4d plane_normal = T_presented_frame_to_obj * normal;
    plane_presented_frame_to_obj = plane_normal;
 
    center = T_presented_frame_to_obj.block<4,1>(0,3);
    
    plane_presented_frame_to_obj(3) = - plane_normal.transpose() * center;
    //normalize
    plane_presented_frame_to_obj /= plane_presented_frame_to_obj(3);
    plane_presented_frame_to_obj(3) = 1.0;
}

void averageQuaternion(const std::vector<Eigen::Quaterniond>& quat_vec, Eigen::Quaterniond& averaged_quat);
void averageTransform(const std::vector<Eigen::Matrix4d>& T_vec, Eigen::Matrix4d& avg_T);
bool isRectCompletelyInImage(const cv::Rect& rect, const cv::Size img_size);
void ROIMsgToCv(const mm_visual_postion::ROI& msg, cv::Rect2f& cv_roi);

void ROICvToMsg(const cv::Rect2f& cv_roi,  mm_visual_postion::ROI& msg);

bool isTransfromSimilaireToIdentity(Eigen::Matrix4d & T, const double pos_tolerance, const double q_epsilon);
inline bool isTransfromSimilaire(Eigen::Matrix4d & T_0, Eigen::Matrix4d & T_1, const double pos_tolerance, const double q_epsilon){
    Eigen::Vector3d translate = T_0.block<3,1>(0,3) - T_1.block<3,1>(0,3);
    if(translate.norm() > pos_tolerance) return false;
    Eigen::Quaterniond q(T_0.block<3,3>(0,0) * T_1.block<3,3>(0,0).inverse());
    if(fabs(q.x()) > q_epsilon || fabs(q.y()) > q_epsilon || fabs(q.z()) > q_epsilon || 1.0 - fabs(q.w()) > q_epsilon ) return false;
    return true;
}
inline bool isCenterPlaneSimilaire(Eigen::Matrix4d & T_0, Eigen::Matrix4d & T_1, const double pos_tolerance, const double abs_dist_deg_max){
    Eigen::Vector3d translate = T_0.block<3,1>(0,3) - T_1.block<3,1>(0,3);
    if(translate.norm() > pos_tolerance) return false;

    Eigen::Vector4d plane_0, center_0, plane_1, center_1;
    getPlane(T_0, plane_0, center_0);
    getPlane(T_0, plane_1, center_1);
    double cos_angle = plane_0.head<3>().dot(plane_1.head<3>()) / (plane_0.head<3>().norm() * plane_1.head<3>().norm());

    if(cos_angle < std::cos(M_PI - abs_dist_deg_max /180.0 * M_PI)) return false;
    return true;
}

void transformToPose4WithQuaternion(Eigen::Matrix4d& T, mm_robot_decision::pose4& pose);
inline void matrixToTransform(const Eigen::Matrix4d& T_mm, geometry_msgs::Transform& transform_m){
    Eigen::Quaterniond q(T_mm.block<3,3>(0,0));    
    transform_m.translation.x = T_mm(0,3) / 1000.0;
    transform_m.translation.y = T_mm(1,3) / 1000.0;
    transform_m.translation.z = T_mm(2,3) / 1000.0;
    transform_m.rotation.x = q.x();
    transform_m.rotation.y = q.y();
    transform_m.rotation.z = q.z();
    transform_m.rotation.w = q.w();
}
void msgToT_frame_to_obj(const mm_robot_decision::VisualAppResponse& res, Eigen::Matrix4d& T_frame_to_obj, std::string expected_presented_frame);

void RVecTVecToHomogenousMatrix(const cv::Mat& r_vec, const cv::Mat& t_vec, cv::Mat& trans);
void RVecTVecToHomogenousMatrix(const std::vector<cv::Mat>& r_vec_vec, const std::vector<cv::Mat>& t_vec_vec, std::vector<cv::Mat>& trans_vec);
void getPointsFromRect(cv::Rect& rect, std::vector<cv::Point2d>& pts);

void expandROI(cv::Rect& rect, double pattern, cv::Size image_size);
void convertMsgInBaseToMsgInEndeffector(const mm_robot_decision::VisualAppResponse& res_in_base, const Eigen::Matrix4d& T_endeffector_to_base, mm_robot_decision::VisualAppResponse& res_in_endeffector);
void seperateZ_rotation(const Eigen::Matrix4d& T_endeffector_to_obj, Eigen::Matrix4d& T_endeffector_to_plane, Eigen::Matrix4d& T_z_rot, double& z_rot){
    Eigen::Vector3d x_unit_vector = T_endeffector_to_obj.block<3,3>(0,0) * Eigen::Vector3d::UnitX();
    
    z_rot = std::atan2(x_unit_vector(1), x_unit_vector(0));
    T_z_rot = Eigen::Matrix4d::Identity();
    T_z_rot.block<3,3>(0,0) = Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_endeffector_to_plane =   T_endeffector_to_obj * T_z_rot.inverse();
    
}
#endif //__VISUAL_POSITION_UTILS_H__
