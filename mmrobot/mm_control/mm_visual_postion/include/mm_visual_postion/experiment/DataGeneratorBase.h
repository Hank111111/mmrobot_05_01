#ifndef __DATA_GENERATOR_BASE_H__
#define __DATA_GENERATOR_BASE_H__
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include "tf2_eigen/tf2_eigen.h"

#include "mm_visual_postion/utils/StereoImageReceiver.h"
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
#include "mm_visual_postion/utils/ArmCameraControlBase.h"
#include "mm_visual_postion/experiment/TransformStampedSaver.h"
class DataGeneratorBase: public ArmCameraControlBase{
protected:
	cv::Mat latest_left_image, latest_right_image;
    ros::Time latest_update_time;
    std::vector<geometry_msgs::TransformStamped> test_transform_data;
    std::vector<geometry_msgs::TransformStamped> gt_transform_data; //ground truth
    std::string save_dir;

    std::random_device rd;
    std::default_random_engine gen; 
public:
    DataGeneratorBase(ros::NodeHandle& n_input, std::string save_dir_input): 
            ArmCameraControlBase(n_input),
            save_dir(save_dir_input),
            gen(rd())
    {
    }

    void writeTransformData( std::vector<geometry_msgs::TransformStamped>& transform_data, std::string transform_data_name){
        std::ofstream fout(save_dir + std::string("/")+transform_data_name +std::string(".dat"), std::ios::out | std::ios::binary);
        for(unsigned int i=0; i<transform_data.size(); i++){
            TransformStampedSaver saved_transform_stamped(transform_data[i]);
            fout.write((char*)&saved_transform_stamped, sizeof(TransformStampedSaver));
        }
        fout.close();
    }

    bool canSeePointsAndIsSafe(const Eigen::Matrix4d& T_endeffector_to_base, const double pixel_padding,
                               const double safe_min_z, const std::vector<Eigen::Vector4d>& points_in_base) const
    {
        /**
         * determinate where a 3D point can be seen in both left and right images, 
         * and whether the camera is safe (the z-distance from 3D point to camera is lagger than save_min_z).
         * @ param T_endeffector_to_base: transformation from endeffector to base
         * @ param pixel_padding: the square padding of the point should be also completely seen in both images
         * @ param safe_min_z: minimum distance (in mm) in z-axis from camera to 3D point that we can say the camera is safe, and there won't be any collision
         * @ param points_in_base: the vector of 3D points that need to be checked. the point is presented with the homogeneous form in base corrdinate.
         */ 

        Eigen::Matrix4d T_cam_to_base = T_cam_to_endeffector * T_endeffector_to_base;
        Eigen::Matrix<double,3,4> left_projection_matrix_in_base = left_projection_matrix_in_cam * T_cam_to_base;
        Eigen::Matrix<double,3,4> right_projection_matrix_in_base = right_projection_matrix_in_cam * T_cam_to_base;

        for(unsigned int point_i = 0; point_i < points_in_base.size(); point_i ++){
            Eigen::Vector4d point_in_cam;
            point_in_cam = T_cam_to_base  * points_in_base[point_i];
            
            if(point_in_cam(2) < safe_min_z) return false;

            Eigen::Vector3d point_in_pixel_left, point_in_pixel_right;
            point_in_pixel_left = left_projection_matrix_in_base * points_in_base[point_i];
            point_in_pixel_right = right_projection_matrix_in_base * points_in_base[point_i];
            normalizeVector3d(point_in_pixel_left);
            normalizeVector3d(point_in_pixel_right);
            if( point_in_pixel_left(0) < pixel_padding || point_in_pixel_left(0) > image_size.width - pixel_padding
                || point_in_pixel_left(1) < pixel_padding || point_in_pixel_left(1) > image_size.height - pixel_padding
                || point_in_pixel_right(0) < pixel_padding || point_in_pixel_right(0) > image_size.width - pixel_padding
                || point_in_pixel_right(1) < pixel_padding || point_in_pixel_right(1) > image_size.height- pixel_padding){
                return false;
            }
        }
        return true;
    }
    void generateMoveCommand(const geometry_msgs::TransformStamped origin_transform,
                             const double x_var, const double y_var, const double z_var, const double q_var, 
                             std::vector<double>& xyzabc_cmd){
        /**
         * uniformally generate the move command that satisfied the conditions of function "cmdIsValid",
         *  use the origin_transform as the center of sample space.  
         * @ param origin_transform: center of the sample space
         * @ param x_var: standard deviate of x, in millimeter
         * @ param y_var: standard deviate of y, in millimeter
         * @ param z_var: standard deviate of z, in millimeter
         * @ param q_var: standard deviate of quaternion, including q_x, q_y, q_z, q_w
         * 
         */ 
        Eigen::Matrix4d origin_T_endeffector_to_base;
        Eigen::Vector3d origin_T_translate;

        transformToMatrix(origin_transform, origin_T_endeffector_to_base);
        Eigen::Matrix4d origin_T_base_to_endeffector = origin_T_endeffector_to_base.inverse(); //origin_T_base_to_endeffector is the same data from the control panel.
        origin_T_translate = origin_T_base_to_endeffector.block<3,1>(0,3);
        Eigen::Quaternion<double> origin_q(origin_T_base_to_endeffector.block<3,3>(0,0));
        
        std::cout<<"origin_T_translate_x_y_z: "<<origin_T_base_to_endeffector.block<3,1>(0,3)<<std::endl;

        std::cout<<"origin_angles euler(zyx): "<<origin_T_base_to_endeffector.block<3,3>(0,0).eulerAngles(2, 1, 0)<<std::endl;
        Eigen::AngleAxisd angle_axis(origin_T_base_to_endeffector.block<3,3>(0,0));
        Eigen::Vector3d rx_ry_rz = angle_axis.axis() * angle_axis.angle();
        std::cout<<"origin_angle RX RY RZ:" << rx_ry_rz <<std::endl;

        // unifrom sample //http://mathworld.wolfram.com/SpherePointPicking.html
        std::uniform_real_distribution<double> dist_x(origin_T_translate(0) - x_var , origin_T_translate(0) + x_var);
        std::uniform_real_distribution<double> dist_y(origin_T_translate(1) - y_var , origin_T_translate(1) + y_var);
        std::uniform_real_distribution<double> dist_z(origin_T_translate(2) - z_var , origin_T_translate(2) + z_var);
        std::uniform_real_distribution<double> dist_q_x(origin_q.x() - q_var, origin_q.x() + q_var);
        std::uniform_real_distribution<double> dist_q_y(origin_q.y() - q_var, origin_q.y() + q_var);
        std::uniform_real_distribution<double> dist_q_z(origin_q.z() - q_var, origin_q.z() + q_var);
        std::uniform_real_distribution<double> dist_q_w(origin_q.w() - q_var, origin_q.w() + q_var);
        
        
        // for the quaternion, uniform sample of a sphere space, reject if (q_x^2 + q_y^2 + q_z^2 + q_w^2 >1)
        while(true){
            double xyz_q[7];
            xyz_q[0] = dist_x(gen);
            xyz_q[1] = dist_y(gen);
            xyz_q[2] = dist_z(gen);
            xyz_q[3] = dist_q_x(gen);
            xyz_q[4] = dist_q_y(gen);
            xyz_q[5] = dist_q_z(gen);
            xyz_q[6] = dist_q_w(gen);

            if(xyz_q[3]*xyz_q[3] + xyz_q[4]*xyz_q[4] + xyz_q[5]*xyz_q[5] + xyz_q[6]*xyz_q[6] > 1){
                continue;
            }
            // check whether the sample is valid
            Eigen::Matrix4d sample_T_endeffector_to_base, sample_T_base_to_endeffector;
            Eigen::Quaternion<double> sample_q( xyz_q[6], xyz_q[3], xyz_q[4], xyz_q[5]); // w,x,y,z
            sample_q.normalize();
            sample_T_base_to_endeffector.setIdentity();
            sample_T_base_to_endeffector.block<3,3>(0,0) = sample_q.toRotationMatrix();
            sample_T_base_to_endeffector(0,3) = xyz_q[0];
            sample_T_base_to_endeffector(1,3) = xyz_q[1];
            sample_T_base_to_endeffector(2,3) = xyz_q[2];
            sample_T_endeffector_to_base = sample_T_base_to_endeffector.inverse();
            

            if(cmdIsValid(sample_T_endeffector_to_base)){
                translateTbase2endeffectorToAngleAxisCmd(sample_T_base_to_endeffector, xyzabc_cmd);
                break;
            }
        }
    }

    virtual bool cmdIsValid(Eigen::Matrix4d& sample_T_endeffector_to_base)=0;
    
    void writeLine(std::ofstream& fs, int id, std::string left_image_path, std::string right_image_path){
        fs << id<<","<<left_image_path<<","<<right_image_path<<"\n";
    }

    int grabTestData(unsigned int id){
        geometry_msgs::TransformStamped transform_endeffector_to_base;
        grabImagesAndTransform(latest_left_image, latest_right_image, transform_endeffector_to_base, latest_update_time);
        test_transform_data.push_back(transform_endeffector_to_base);
        
        // save images
        std::string left_image_path, right_image_path;
        left_image_path = save_dir + std::string("/") + std::to_string(id) + std::string("_left.bmp");
        right_image_path = save_dir + std::string("/") + std::to_string(id) + std::string("_right.bmp");
        imwrite( left_image_path, latest_left_image );
        imwrite( right_image_path, latest_right_image );

        assert(test_transform_data.size() == id+1);
        return 0;
    }



};


#endif