#ifndef __CIRCLE_MODEL_H__
#define __CIRCLE_MODEL_H__

#include <Eigen/Dense>
#include <type_traits>
#include <iostream>
#include "mm_visual_postion/utils/utils.h"
typedef Eigen::Matrix<double,6,1> EigenVector6d;
typedef Eigen::Matrix<double,7,1> EigenVector7d;
typedef Eigen::Matrix<double,8,1> EigenVector8d;
typedef Eigen::Matrix<double,9,1> EigenVector9d;

class CirclePlane{
public:
    Eigen::Vector4d center; //circle's center 
    Eigen::Vector4d plane; //  circle's plane (plane * x = 0)
    CirclePlane(){}
    CirclePlane(Eigen::Vector4d& circle_center, Eigen::Vector4d& circle_plane){
        center = circle_center;
        plane = circle_plane;
    }
    void normalizePlane(){
        double norm = plane.head<3>().norm();
        if(plane(3) > 0) norm = -norm; // convention: last element should be negative
        plane /= norm;
    }
    void getTransformOriginToCircle(Eigen::Matrix4d& transform) const{
        /** get transform matrix (4x4) from origin xy plane to circle
         *  X_in_origin_coord = transform * X_in_circles_coord
         *  Attention: the rotation about z-axis is not controlled, thus the transform matrix is not unique.
         *  @ param transform : the 4x4 transform matrix from circle plane to x-o-y plane. 
         *  This function implements a method proposed by https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector 
        */ 
        Eigen::Vector3d normal; 
        normal = plane.head<3>().normalized();
        Eigen::Matrix3d R; 
        double nx_ny_norm = std::sqrt(normal(0) * normal(0) + normal(1) * normal(1));
        R(0,0) = normal(1) / nx_ny_norm;
        R(0,1) = -normal(0) / nx_ny_norm;
        R(0,2) = 0.0;
        R(1,0) = normal(0) * normal(2) / nx_ny_norm;
        R(1,1) = normal(1) * normal(2) / nx_ny_norm;
        R(1,2) = - nx_ny_norm;
        R(2,0) = normal(0);
        R(2,1) = normal(1);
        R(2,2) = normal(2);

        transform.setIdentity();
        transform.block<3,3>(0,0) = R.inverse();
        transform.block<3,1>(0,3) = center.head(3);

        transform.block<3,3>(0,0) = transform.block<3,3>(0,0).eval() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix(); // plan' T has two orientation
    }
    void getTransformCircleToOrigin(Eigen::Matrix4d& transform) const{
        Eigen::Matrix4d origin_to_circle;
        getTransformOriginToCircle(origin_to_circle);
        transform = origin_to_circle.inverse();
    }
    void setTransform(const Eigen::Matrix4d& transform_origin_to_circle){
        getPlane(transform_origin_to_circle, center, plane);
    }
    EigenVector7d getParams() const{
        // params: x, y, z, q1, q2, q3, q4, radius
        EigenVector7d param;
        Eigen::Matrix4d transform;
        getTransformOriginToCircle(transform);
        Eigen::Quaterniond quaternion(transform.block<3,3>(0,0));
        param.head<3>() = transform.block<3,1>(0,3);
        param(3) = quaternion.x();
        param(4) = quaternion.y();
        param(5) = quaternion.z();
        param(6) = quaternion.w();
        
        return param;
    }
    void setParams(const EigenVector7d & params){
        Eigen::Matrix4d transform;
        Eigen::Quaterniond quaternion(params(6), params(3), params(4), params(5)); // qw,qx,qy,qz
        Eigen::Matrix3d rot = quaternion.toRotationMatrix();
        transform.setIdentity();
        transform.block<3,3>(0,0) = rot;
        transform.block<3,1>(0,3) = params.segment(0,3);
        setTransform(transform);
    }
    static int numParams(){
        return 6;
    }
    typedef std::integral_constant<int, 6> num_params;
    Eigen::Vector3d getNormalVector(){
        Eigen::Vector3d normal_vector = plane.head<3>();
        return normal_vector.normalized();
    }
    static CirclePlane average(std::vector<CirclePlane> circle_plane_vec){
        CirclePlane avg_circle_plane;
        avg_circle_plane.center.setZero();
        Eigen::Vector3d avg_normal_vector;
        avg_normal_vector.setZero();
        for(unsigned int i=0; i<circle_plane_vec.size(); i++){
            avg_circle_plane.center += circle_plane_vec[i].center;
            avg_normal_vector += circle_plane_vec[i].getNormalVector();
        }
        avg_circle_plane.center /= circle_plane_vec.size();
        avg_normal_vector /= circle_plane_vec.size();
        avg_circle_plane.plane(3) = - avg_normal_vector.transpose() * avg_circle_plane.center.head<3>();
        avg_circle_plane.plane.head<3>() = avg_normal_vector;
        return avg_circle_plane;
    }
};


class Circle3D:public CirclePlane{
public:
    double radius; //radius of the circle
    double score; // the quality of the detected circle
    //Eigen::Matrix4d transform;
    Circle3D():CirclePlane(){
        radius = 0;
        score = 0;
    }
    Circle3D(CirclePlane circle_plane){
        center = circle_plane.center;
        plane = circle_plane.plane;
        radius = 0;
        score = 0;
    }
    Circle3D(Eigen::Vector4d& circle_center, Eigen::Vector4d& circle_plane, double circle_radius):CirclePlane(circle_center, circle_plane){
        radius = circle_radius;
    }
    EigenVector8d getParams() const{
        // param: x, y, z, q_x, q_y, q_z, q_w, radius
        EigenVector8d param;
        param.head<7>() = ((CirclePlane*)this)->getParams();
        param(7) = radius;
        return param;
    }
    void setParams(const EigenVector8d & param){
        EigenVector7d param_plane = param.head<7>();
        ((CirclePlane*)this)->setParams(param_plane);
        radius = param(7);
    }
    static int numParams(){
        return 8;
    }
    typedef std::integral_constant<int, 7> num_params;
    static Circle3D transformCircle3D(Circle3D circle_in_cam, Eigen::Matrix4d transform_base_to_cam){
        Circle3D circle_in_base;
        
        circle_in_base.radius = circle_in_cam.radius;
        Eigen::Matrix4d transform_cam_to_circle;
        circle_in_cam.getTransformOriginToCircle(transform_cam_to_circle);
        circle_in_base.setTransform(transform_base_to_cam * transform_cam_to_circle);
        circle_in_base.score = circle_in_cam.score;
        return circle_in_base;
    }
    static Circle3D average(std::vector<Circle3D>& circle_3d_vec){
        Circle3D avg_circle;
        
        std::vector<CirclePlane> circle_plane_vec;
        circle_plane_vec.insert(circle_plane_vec.begin(), circle_3d_vec.begin(), circle_3d_vec.end());

        CirclePlane avg_circle_plane = CirclePlane::average(circle_plane_vec);
        avg_circle = avg_circle_plane;
        double r = 0;
        for(unsigned int i=0; i<circle_3d_vec.size(); i++){
            r += circle_3d_vec[i].radius;
        }
        avg_circle.radius = r / circle_3d_vec.size();
        return avg_circle;
    }

};

class ConcentricCircles3D:public CirclePlane{
public:
    double radius_inner;
    double radius_outer;
    double score;
    ConcentricCircles3D():CirclePlane(){
        radius_inner = 0.0;
        radius_outer = 0.0;
    }
    ConcentricCircles3D(Eigen::Vector4d& circle_center, Eigen::Vector4d& circle_plane, double circle_radius_inner, double circle_radius_outer)
    :CirclePlane(circle_center, circle_plane)
    {
        radius_inner = circle_radius_inner;
        radius_outer = circle_radius_outer;
    }
    ConcentricCircles3D(CirclePlane& circle_plane){
        center = circle_plane.center;
        plane = circle_plane.plane;
        radius_inner = 0.0;
        radius_outer = 0.0;
    }
    ConcentricCircles3D(Circle3D& a, Circle3D& b){
        center = (a.center + b.center)/2.0;
        plane = (a.plane + b.plane)/2.0;
        radius_inner = a.radius;
        radius_outer = b.radius;
    }
    ConcentricCircles3D(Circle3D& a, Circle3D& b, double circles_score){
        center = (a.center + b.center)/2.0;
        plane = (a.plane + b.plane)/2.0;
        radius_inner = a.radius;
        radius_outer = b.radius;
        score = circles_score;
    }
    void splitToCircles(Circle3D circles[2]) const{
        circles[0].center = center;
        circles[0].radius = radius_inner;
        circles[0].plane = plane;

        circles[1].center = center;
        circles[1].radius = radius_outer;
        circles[1].plane = plane;
    }
    EigenVector9d getParams() const{ 
        // params: x, y, z, q_a, q_b, q_c, q_w, radius_inner, radius_outer
        EigenVector9d params;
        params.head<7>() = ((CirclePlane*)this)->getParams();
        params(7) = radius_inner;
        params(8) = radius_outer;
        return params;
    }
    void setParams(const EigenVector9d & params){
        EigenVector7d params_plane = params.head<7>();
        ((CirclePlane*)this)->setParams(params_plane);
        radius_inner = params(7);
        radius_outer = params(8);
    }
    static int numParams(){
        return 9;
    }
    typedef std::integral_constant<int, 8> num_params;
    

    static ConcentricCircles3D transformConcentricCircle(ConcentricCircles3D circle_in_cam, Eigen::Matrix4d transform_base_to_cam){
        ConcentricCircles3D circle_in_base;
        
        circle_in_base.radius_inner = circle_in_cam.radius_inner;
        circle_in_base.radius_outer = circle_in_cam.radius_outer;
        Eigen::Matrix4d transform_cam_to_circle;
        circle_in_cam.getTransformOriginToCircle(transform_cam_to_circle);
        circle_in_base.setTransform(transform_base_to_cam * transform_cam_to_circle);
        /*
        circle_in_base.center = tranform_base_to_cam * circle_in_cam.center;
        Eigen::Vector4d plane_normal_in_cam, plane_normal_in_base;
        plane_normal_in_cam.head<3>() = circle_in_cam.plane.head<3>().normalized();
        plane_normal_in_cam(3) = 0.0;
        plane_normal_in_base = tranform_base_to_cam * plane_normal_in_cam;
        circle_in_base.plane(3) = - plane_normal_in_base.transpose() * circle_in_base.center;
        circle_in_base.plane.head<3>() = plane_normal_in_base.head<3>();
        circle_in_base.radius_inner = circle_in_cam.radius_inner;
        circle_in_base.radius_outer = circle_in_cam.radius_outer;
        */
        return circle_in_base;
    }
    static ConcentricCircles3D average(std::vector<ConcentricCircles3D>& concentric_circle_plane_vec){
        ConcentricCircles3D avg_concentric_circle;
        
        std::vector<CirclePlane> circle_plane_vec;
        circle_plane_vec.insert(circle_plane_vec.begin(), concentric_circle_plane_vec.begin(), concentric_circle_plane_vec.end());

        CirclePlane avg_circle_plane = CirclePlane::average(circle_plane_vec);
        avg_concentric_circle = avg_circle_plane;
        double r_inner = 0;
        double r_outer = 0;
        for(unsigned int i=0; i<concentric_circle_plane_vec.size(); i++){
            r_inner += concentric_circle_plane_vec[i].radius_inner;
            r_outer += concentric_circle_plane_vec[i].radius_outer;   
        }
        avg_concentric_circle.radius_inner = r_inner / concentric_circle_plane_vec.size();
        avg_concentric_circle.radius_outer = r_outer / concentric_circle_plane_vec.size();
        return avg_concentric_circle;
    }


};
#endif