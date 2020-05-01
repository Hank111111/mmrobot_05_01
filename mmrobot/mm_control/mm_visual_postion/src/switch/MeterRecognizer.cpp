#include "mm_visual_postion/switch/RectSwitchFinder.h"
#include "ros/ros.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/utils/ArmCameraControlBase.h"
#include "mm_visual_postion/AppInnerRequest.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/utils/ObjectDefinition.h"
class MeterRecoginizerNode: public ArmCameraBase{
private:
    RectSwitchFinder impl;
    bool finished;
    ros::Subscriber cmd_sub;
    ros::Publisher result_pub;
    Eigen::Matrix4d T_cam_to_endeffector;

public:
    MeterRecoginizerNode(ros::NodeHandle& nh): ArmCameraBase(nh){
        impl.setCameraParams(model_ptr);
        result_pub = nh.advertise<mm_robot_decision::VisualAppResponse>("/mm_visual/apps/point_meter/result", 2);
        cmd_sub = nh.subscribe("/mm_visual/apps/point_meter/goal", 2, &MeterRecoginizerNode::requestCallback, this);
        finished = true;
        T_cam_to_endeffector = model_ptr->endeffector_to_cam_transform.inverse();
    }

    void requestCallback(const mm_visual_postion::AppInnerRequest::ConstPtr &msg){
        mm_robot_decision::VisualAppResponse response;
        response.object_name = msg->object_name;
        response.frame_id = ENDEFFECTOR_FRAME_NAME;
        response.object_unique_id_on_equipment = msg->object_unique_id_on_equipment;

        if(msg->object_name != POINT_METERS_NAME){
            ROS_ERROR("This node can only accept the command [%s]", POINT_METERS_NAME);
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

        Eigen::Matrix4d rough_T_base_to_obj, rough_T_cam_to_obj;
        
        transformToMatrix(msg->transform_base_to_obj, rough_T_base_to_obj);
        
        // return directly the request' pose
        Eigen::Matrix4d rough_T_endeffector_to_obj = T_endeffector_to_base * rough_T_base_to_obj;
        transformToPose4WithQuaternion(rough_T_endeffector_to_obj, response.pose);

        rough_T_cam_to_obj = T_cam_to_endeffector * T_endeffector_to_base * rough_T_base_to_obj;
        impl.setSwitchParamsToRecognize(msg->object_name ,msg->object_width, msg->object_height, rough_T_cam_to_obj);
    
        cv::Rect roi_left_rect, roi_right_rect;
        roi_left_rect.x = (int)msg->left_roi.x -20;
        roi_left_rect.y = (int)msg->left_roi.y -20;
        roi_left_rect.width = (int)msg->left_roi.width +40;
        roi_left_rect.height = (int)msg->left_roi.height + 40;
        
        roi_right_rect.x = (int)msg->right_roi.x -20;
        roi_right_rect.y = (int)msg->right_roi.y -20;
        roi_right_rect.width = (int)msg->right_roi.width +40;
        roi_right_rect.height = (int)msg->right_roi.height + 40;
        boundRectROI(roi_left_rect, left_image.size());
        boundRectROI(roi_right_rect, right_image.size());
    
        std::vector<SwitchPtr> all_switches;
        cv::Mat left_roi_img = left_image(roi_left_rect);
        cv::Mat right_roi_img = right_image(roi_right_rect);
        impl.findAllSwitches(left_roi_img, right_roi_img, roi_left_rect, roi_right_rect, all_switches, false);

        
        if(all_switches.size() == 0){
            // failed
            ROS_WARN("failed to detect the object [%s]", msg->object_name.c_str());
            response.success = false;
            result_pub.publish(response);
            return;
        }
        if(all_switches.size() > 1){
            // failed
            ROS_WARN("detected too much object that satisfied the constraints of [%s]...", msg->object_name.c_str());
            response.success = false;
            result_pub.publish(response);
            return;
        }
        cv::Mat corrected_left_switch, corrected_right_switch;
        
        impl.getCorrectedSwitch(left_roi_img,  *(all_switches[0]), cv::Size(200,200), corrected_left_switch, LEFT_CAMERA);
        impl.getCorrectedSwitch(right_roi_img,  *(all_switches[0]), cv::Size(200,200), corrected_right_switch, RIGHT_CAMERA);
        
        cv::Rect inner_roi(10,10,180,180);
        cv::Mat left_roi_edge, right_roi_edge;
        cv::Mat left_roi_gray, right_roi_gray;
        cv::cvtColor(corrected_left_switch(inner_roi), left_roi_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(corrected_right_switch(inner_roi), right_roi_gray, cv::COLOR_BGR2GRAY);

        impl.getEdge(left_roi_gray, left_roi_edge);
        impl.getEdge(right_roi_gray, right_roi_edge);
        
        std::vector<cv::Vec3f> left_lines, right_lines;
        cv_v3_4_5::HoughLines(left_roi_edge, left_lines, 1, M_PI / 180. * 2, 30, 0, 0, 0, M_PI); 
        cv_v3_4_5::HoughLines(right_roi_edge, right_lines, 1, M_PI / 180. * 2, 30, 0, 0, 0, M_PI); 

        if(left_lines.size() == 0 || right_lines.size() == 0){
            ROS_WARN("Failed to detect the indicater");
            response.success = false;
            result_pub.publish(response);
            return;
        }

        // if angle is bigger then the indicate number is bigger
        double angle_left = left_lines[0][0] > 0 ? left_lines[0][1]- M_PI/2.0 : M_PI/2.0 + left_lines[0][1];
        double angle_right = right_lines[0][0] > 0 ? right_lines[0][1]- M_PI/2.0 : M_PI/2.0 + right_lines[0][1];
        if(fabs(angle_left - angle_right) < M_PI/180.0 * 5){ // accept 5 deg error
            double angle = (angle_left + angle_right)/ 2.0 * 180 / M_PI;
            ROS_INFO("point meter indicator's angle: %f",angle);
            
            response.object_status.push_back(int(angle));
            response.width = all_switches[0]->width;
            response.height = all_switches[0]->height;
            response.success = true;
            result_pub.publish(response);
            return;
        }
        else{
            ROS_WARN("indicater's degree is not the same on left [%f] and right image [%f]", angle_left, angle_right);
            response.success = false;
            result_pub.publish(response);
            return;
        }

        
    }
};
int main(int argc, char** argv){
	ros::init(argc, argv, "mm_visual_rect_switch_find_node");
	ros::NodeHandle nh;
    MeterRecoginizerNode meter_recognition_node(nh);

    while(ros::ok()){
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
}