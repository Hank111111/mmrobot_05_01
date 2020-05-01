#include <stdio.h>
#include "mm_visual_postion/digitalmeter/digitalDeal.h"
#include "mm_visual_postion/switch/RectSwitchFinder.h"
#include "ros/ros.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/utils/ArmCameraControlBase.h"
#include "mm_visual_postion/AppInnerRequest.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/utils/ObjectDefinition.h"


using namespace std;
using namespace cv;



class MeterRecoginizerNode: public ArmCameraBase{
private:
    RectSwitchFinder impl;
    bool finished;
    ros::Subscriber cmd_sub;
    ros::Publisher result_pub;
    digitalProcess digitfinder;
    string digit_result;
    Eigen::Matrix4d T_cam_to_endeffector;

public:
    MeterRecoginizerNode(ros::NodeHandle& nh): ArmCameraBase(nh){
        impl.setCameraParams(model_ptr);
        result_pub = nh.advertise<mm_robot_decision::VisualAppResponse>("/mm_visual/apps/digital_meter/result", 2);
        cmd_sub = nh.subscribe("/mm_visual/apps/digital_meter/goal", 2, &MeterRecoginizerNode::requestCallback, this);
        finished = true;
        T_cam_to_endeffector = model_ptr->endeffector_to_cam_transform.inverse();
    }


    void requestCallback(const mm_visual_postion::AppInnerRequest::ConstPtr &msg){
        mm_robot_decision::VisualAppResponse response;
        response.object_name = msg->object_name;
        response.frame_id = ENDEFFECTOR_FRAME_NAME;
        response.pose.state = "WorkSpacePlanning";
        response.object_unique_id_on_equipment = msg->object_unique_id_on_equipment;

        if(msg->object_name != DIGITAL_METERS_NAME){
            ROS_ERROR("This node can only accept the command [%s]", DIGITAL_METERS_NAME);
            response.success = false;
            result_pub.publish(response);
            return;
        }
        cv::Mat left_image, right_image;
        //geometry_msgs::TransformStamped transform_endeffector_to_base;
        ros::Time stamp;
        geometry_msgs::TransformStamped transform_endeffector_to_base;
        grabImagesAndTransform(left_image, right_image, 
                            transform_endeffector_to_base, stamp);
        Eigen::Matrix4d T_endeffector_to_base;
        transformToMatrix(transform_endeffector_to_base, T_endeffector_to_base);

        Eigen::Matrix4d rough_T_base_to_obj, rough_T_endeffector_to_obj;
        
        transformToMatrix(msg->transform_base_to_obj, rough_T_base_to_obj);
        rough_T_endeffector_to_obj = T_endeffector_to_base * rough_T_base_to_obj;
        transformToPose4WithQuaternion(rough_T_endeffector_to_obj, response.pose);


        cv::Rect roi_left_rect;
        roi_left_rect.x = (int)msg->left_roi.x;
        roi_left_rect.y = (int)msg->left_roi.y;
        roi_left_rect.width = (int)msg->left_roi.width;
        roi_left_rect.height = (int)msg->left_roi.height;
        
        boundRectROI(roi_left_rect, left_image.size());
    
        cv::Mat left_roi_img = left_image(roi_left_rect);
        //imshow("roi",left_roi_img);
        //waitKey();
        digit_result = digitfinder.finddigits(left_roi_img);

        if (true){
            response.additional_text_info = digit_result;
            response.width = msg->object_width;
            response.height = msg->object_height;
            response.success = true;
            result_pub.publish(response);
            return;
        }
    }
};

int main(int argc, char** argv){
	  ros::init(argc, argv, "mm_visual_digital_meter_node");
	  ros::NodeHandle nh;
    MeterRecoginizerNode meter_recognition_node(nh);

    while(ros::ok()){
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
}




