#ifndef __ARM_CAMERA_CONTROL_BASE_H__
#define __ARM_CAMERA_CONTROL_BASE_H__
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include "tf2_eigen/tf2_eigen.h"

#include "mm_visual_postion/utils/StereoImageReceiver.h"
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
#include "mm_visual_postion/utils/ArmCommander.h"
#include "mm_visual_postion/utils/utils.h"
class ArmCameraBase{
    /**
     * Base class that implements high-level interaction of arm and camera, including Synchronously grab the images and transformation msg, etc
     * You have no permission to control the arm by using this class
     */ 

protected:
    ros::NodeHandle n;

	tf2_ros::Buffer tfBuffer;
	std::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;

	StereoImageReceiver image_receiver;

    Eigen::Matrix<double,4,4> T_cam_to_endeffector, T_endeffector_to_cam;
    Eigen::Matrix<double, 3, 4> left_projection_matrix_in_cam, right_projection_matrix_in_cam;

    cv::Size image_size;
    std::shared_ptr<StereoCameraArmModel> model_ptr;
public:
    ArmCameraBase(ros::NodeHandle& n_input): n(n_input),
            image_receiver(n_input)
    {
        while(ros::ok()){
            if(!image_receiver.cameraInfoGot()){
                ROS_INFO("Wait for camera info msg...");
            }
            else{
				image_receiver.getArmCameraModel(model_ptr);
                break;
			}	
            sleep(1);
        }
        image_size = model_ptr->left_camera.image_size;
        T_endeffector_to_cam = model_ptr->endeffector_to_cam_transform;
        T_cam_to_endeffector = T_endeffector_to_cam.inverse();
        left_projection_matrix_in_cam = model_ptr->left_camera.projection_mat;
        right_projection_matrix_in_cam = model_ptr->right_camera.projection_mat;
        tfListenerPtr = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
    }

    void grabImagesAndTransform(cv::Mat& left_image, cv::Mat& right_image, 
                                geometry_msgs::TransformStamped& transform_endeffector_to_base, ros::Time& stamp)
    {
        bool success_grab_image;
        do{
            ros::spinOnce();
            success_grab_image = image_receiver.getLatestImages(left_image, right_image, stamp);
        }while(ros::ok() && (!success_grab_image || !getTransform(transform_endeffector_to_base, stamp)));
    }

    void grabImages(cv::Mat& left_image, cv::Mat& right_image, ros::Time stamp){
        while(ros::ok() && !image_receiver.getLatestImages(left_image, right_image, stamp)){
            ros::spinOnce();
        }
    }
    
    bool getTransform(geometry_msgs::TransformStamped& transform_endeffector_to_base, ros::Time stamp){
        bool flag = false;
        for(unsigned int i=0; i<5; i++){
            try
            {
                transform_endeffector_to_base = tfBuffer.lookupTransform("tool0_controller", "base", stamp);
                flag = true;
                break;
            }
            catch (tf2::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                flag = false;
            }
        }
        return flag;
    }
    bool getTransform(Eigen::Matrix4d& T_endeffector_to_base, ros::Time stamp){
        geometry_msgs::TransformStamped transform_endeffector_to_base;
        if(!getTransform(transform_endeffector_to_base, stamp)) return false;
        transformToMatrix(transform_endeffector_to_base, T_endeffector_to_base);
        return true;
    }
};
class ArmCameraControlBase: public ArmCameraBase{
    /**
     * Base class that implements high-level interaction of arm and camera, including Synchronously grab the images and transformation msg, moving the arm, etc
     */ 

protected:
    ArmCommander arm_commander;
    
public:
    ArmCameraControlBase(ros::NodeHandle& n_input): ArmCameraBase(n_input),
            arm_commander(n_input)
    {
    }

    void waitForManuallyMove(std::string show_name="Move to proper position"){
        // manually move robot to position  then press any key
        int key = -1; 
        cv::Mat latest_left_image, latest_right_image;
        ros::Time latest_update_time;
        while(key == -1){ //no key is pressed
            cv::namedWindow("Move to proper position", 0);
            cv::Mat image;
            while(ros::ok() && !image_receiver.getLatestImages(latest_left_image, latest_right_image, latest_update_time)){
                ros::spinOnce();
            }
            cv::hconcat(latest_left_image, latest_right_image, image);
            cv::imshow("Move to proper position", image);
            key = cv::waitKey(20);
        }
        cv::destroyWindow("Move to proper position");
    }
    void moveArmByToolSpacePlanningQuaternion(const double& x, const double& y, const double& z,const double& q_x, const double& q_y, const double& q_z, const double& q_w){
        arm_commander.moveByToolSpacePlanning(x/1000.0, y/1000.0, z/1000.0, q_x, q_y, q_z, q_w);
    }
    void moveArmByBaseSpacePlanningAngleAxis(Eigen::Matrix4d& T_base_to_endeffector){
        std::vector<double> XYZRxRyRz;
        translateTbase2endeffectorToAngleAxisCmd(T_base_to_endeffector, XYZRxRyRz); // in mm and rad
        arm_commander.moveByBaseSpacePlanning(XYZRxRyRz[0]/1000.0, XYZRxRyRz[1]/1000.0, XYZRxRyRz[2]/1000.0, XYZRxRyRz[3], XYZRxRyRz[4], XYZRxRyRz[5]);
    }
    void moveArmByBaseSpacePlanningAngleAxis(const double& x, const double& y, const double& z, const double& a, const double& b, const double& c){
        arm_commander.moveByBaseSpacePlanning(x/1000.0, y/1000.0, z/1000.0, a, b, c);
    }
    void moveArmByJointSpacePlanningAngleAxis(Eigen::Matrix4d& T_base_to_endeffector){
        std::vector<double> XYZRxRyRz;
        translateTbase2endeffectorToAngleAxisCmd(T_base_to_endeffector, XYZRxRyRz);// in mm and rad
        arm_commander.moveByJointSpacePlanning(XYZRxRyRz[0]/1000.0, XYZRxRyRz[1]/1000.0, XYZRxRyRz[2]/1000.0, XYZRxRyRz[3], XYZRxRyRz[4], XYZRxRyRz[5]);
    }
    void moveArmByJointSpacePlanningAngleAxis(const double& x, const double& y, const double& z, const double& a, const double& b, const double& c){
        arm_commander.moveByJointSpacePlanning(x/1000.0, y/1000.0, z/1000.0, a, b, c);
    }
};

#endif