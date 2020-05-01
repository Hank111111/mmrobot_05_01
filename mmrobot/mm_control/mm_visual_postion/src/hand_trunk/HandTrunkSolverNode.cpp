#include "ros/ros.h"
#include "mm_visual_postion/hand_trunck/HandTrunkSolver.h"
#include "mm_visual_postion/utils/ArmCameraControlBase.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/AppInnerRequest.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/utils/ObjectDefinition.h"

#define USE_ORIGINAL_PLANE
//#define USE_ORIGINAL_POS_AND_PLANE

class HandTrunkSolverNode: public ArmCameraBase{
private:
    HandTrunkSolver impl;
    bool finished;
    ros::Subscriber cmd_sub;
    ros::Publisher result_pub;
    Eigen::Matrix4d T_cam_to_endeffector;

public:
    HandTrunkSolverNode(ros::NodeHandle& nh): ArmCameraBase(nh), impl(model_ptr){
        result_pub = nh.advertise<mm_robot_decision::VisualAppResponse>("/mm_visual/apps/hand_cart/result", 2);
        cmd_sub = nh.subscribe("/mm_visual/apps/hand_cart/goal", 2, &HandTrunkSolverNode::requestCallback, this);
        T_cam_to_endeffector = model_ptr->endeffector_to_cam_transform.inverse();
    }

    void requestCallback(const mm_visual_postion::AppInnerRequest::ConstPtr &msg){
        mm_robot_decision::VisualAppResponse response;
        response.object_name = msg->object_name;
        response.frame_id = ENDEFFECTOR_FRAME_NAME;
        response.object_unique_id_on_equipment = msg->object_unique_id_on_equipment;

        if(msg->object_name != HANDCART_SWITCH_NAME){
            ROS_WARN("mm_visual_hand_cart_solver_node can only process the order [%s], but the received order is [%s]", HANDCART_SWITCH_NAME, msg->object_name.c_str());
            response.success = false;
            result_pub.publish(response);
            return;
        }
        if(msg->frame_id != ENDEFFECTOR_FRAME_NAME){
            ROS_WARN("mm_visual_hand_cart_solver_node now can only support the request in endeffector frame, but the frame_id of the received cmd is [%s]", msg->frame_id.c_str());
            response.success = false;
            result_pub.publish(response);
        }
        if(msg->additional_numerical_info.size() <2){
            ROS_ERROR("You should put 'estimate radius' in the first place of additional_numerical_info, and 'estimate_square_trunk_length' in the second place...");
            response.success = false;
            result_pub.publish(response);
            return;
        }
        cv::Mat left_image, right_image;
        geometry_msgs::TransformStamped transform_endeffector_to_base;
        ros::Time stamp;
        grabImagesAndTransform(left_image, right_image, 
                            transform_endeffector_to_base, stamp);
        Eigen::Matrix4d T_endeffector_to_base;
        transformToMatrix(transform_endeffector_to_base, T_endeffector_to_base);
        Eigen::Matrix4d rough_T_base_to_obj, rough_T_endeffector_to_obj;

        transformToMatrix(msg->transform_base_to_obj, rough_T_base_to_obj);
        rough_T_endeffector_to_obj = T_endeffector_to_base * rough_T_base_to_obj;

        cv::Rect roi_left_rect, roi_right_rect;
        roi_left_rect.x = (int)msg->left_roi.x -100;
        roi_left_rect.y = (int)msg->left_roi.y -100;
        roi_left_rect.width = (int)msg->left_roi.width +200;
        roi_left_rect.height = (int)msg->left_roi.height + 200;
        
        roi_right_rect.x = (int)msg->right_roi.x -100;
        roi_right_rect.y = (int)msg->right_roi.y -100;
        roi_right_rect.width = (int)msg->right_roi.width +200;
        roi_right_rect.height = (int)msg->right_roi.height + 200;

        boundRectROI(roi_left_rect, left_image.size());
        boundRectROI(roi_right_rect, right_image.size());

        std::vector<cv::Mat> left_rectified_image_roi_vec, right_rectified_image_roi_vec;
        std::vector<cv::Rect> left_rect_roi_vec, right_rect_roi_vec;
        std::vector<Eigen::Matrix4d> transform_cam_to_base_vec;
        cv::Mat left_roi_img = left_image(roi_left_rect);
        cv::Mat right_roi_img = right_image(roi_right_rect);
        left_rectified_image_roi_vec.push_back(left_roi_img);
        right_rectified_image_roi_vec.push_back(right_roi_img);
        left_rect_roi_vec.push_back(roi_left_rect);
        right_rect_roi_vec.push_back(roi_right_rect);
        transform_cam_to_base_vec.push_back(T_cam_to_endeffector * T_endeffector_to_base);
        

        double estimate_radius = msg->additional_numerical_info[0];
        double estimate_square_trunk_length = msg->additional_numerical_info[1];
        double radius_threshold = 5; //mm
        double max_acceptable_cost_per_point = 200;

        #ifdef USE_ORIGINAL_POS_AND_PLANE
            response.success = true;
            response.width = msg->object_width;
            response.height = msg->object_height;
            response.additional_numerical_info.resize(2);
            response.additional_numerical_info[0] = estimate_radius;
            response.additional_numerical_info[1] = estimate_square_trunk_length;
            transformToPose4WithQuaternion(rough_T_endeffector_to_obj, response.pose);
            response.pose.state = "WorkSpacePlanning";
            result_pub.publish(response);
            return;
        #endif
        
        Circle3D result_circle; Eigen::Matrix4d T_base_to_trunk;
        if(! impl.findTrunk(left_rectified_image_roi_vec, right_rectified_image_roi_vec, 
                left_rect_roi_vec, right_rect_roi_vec,
                transform_cam_to_base_vec, 
                estimate_radius, radius_threshold, max_acceptable_cost_per_point,
                estimate_square_trunk_length,
                result_circle, T_base_to_trunk))
        {
            response.success = false;
            ROS_WARN("failed to find the handcart switch");
            result_pub.publish(response);
            return;
        }
        else{
            response.success = true;
            response.width = result_circle.radius * 2;
            response.height = result_circle.radius * 2;
            response.additional_numerical_info.resize(2);
            response.additional_numerical_info[0] = result_circle.radius;
            response.additional_numerical_info[1] = msg->additional_numerical_info[1];
            Eigen::Matrix4d T_endeffector_to_trunk;
            T_endeffector_to_trunk = model_ptr->endeffector_to_cam_transform * transform_cam_to_base_vec[0] * T_base_to_trunk;
            transformToPose4WithQuaternion(T_endeffector_to_trunk, response.pose);
            response.pose.state = "WorkSpacePlanning";

            #ifdef USE_ORIGINAL_PLANE
            mm_robot_decision::pose4 original_pose;
            Eigen::Matrix4d T_endeffector_to_plane; Eigen::Matrix4d T_z_rot; double z_rot;
            seperateZ_rotation(T_endeffector_to_trunk, rough_T_endeffector_to_obj, T_z_rot, z_rot);
            Eigen::Matrix4d rough_T_endeffector_to_obj_with_z_rot = rough_T_endeffector_to_obj * T_z_rot;
            
            transformToPose4WithQuaternion(rough_T_endeffector_to_obj_with_z_rot, original_pose);
            response.pose.a = original_pose.a;
            response.pose.b = original_pose.b;
            response.pose.c = original_pose.c;
            response.pose.w = original_pose.w;
            response.pose.x = original_pose.x;
            response.pose.y = original_pose.y;
            response.pose.z = original_pose.z;   
            //ROS_INFO_STREAM(response);
            #endif
            result_pub.publish(response);
            return;
        }
    }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "mm_visual_hand_cart_solver_node");
	ros::NodeHandle nh;
    HandTrunkSolverNode solver(nh);
    while(ros::ok()){
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
}