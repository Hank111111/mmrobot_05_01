 #include "ros/ros.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/switch/RectSwitchFinder.h"
#include "mm_visual_postion/utils/ArmCameraControlBase.h"
#include "mm_visual_postion/AppInnerRequest.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/utils/CheckCreatePath.h"
#include "mm_pattern_recognition/RecognizeStatus.h"
class RectSwitchFindNode: public ArmCameraBase{
private:
    RectSwitchFinder impl;
    mm_visual_postion::AppInnerRequest last_request;
    bool finished;
    ros::Subscriber cmd_sub;
    ros::Publisher result_pub;
    Eigen::Matrix4d T_cam_to_endeffector;
    void requestCallback(const mm_visual_postion::AppInnerRequest::ConstPtr &msg){
        last_request = *msg;
        finished = false;   
    }
    ros::ServiceClient  remote_switch_recognize_client;
public:
    RectSwitchFindNode(ros::NodeHandle& nh): ArmCameraBase(nh){
        impl.setCameraParams(model_ptr);
        result_pub = nh.advertise<mm_robot_decision::VisualAppResponse>("/mm_visual/apps/rect_switch/result", 2);
        cmd_sub = nh.subscribe("/mm_visual/apps/rect_switch/goal", 2, &RectSwitchFindNode::requestCallback, this);
        finished = true;
        T_cam_to_endeffector = model_ptr->endeffector_to_cam_transform.inverse();
        remote_switch_recognize_client = n.serviceClient<mm_pattern_recognition::RecognizeStatus>("recognize_remote_switch_status");
    }
    bool recognizeRemoteSwitch(cv::Mat& left_image, cv::Mat& right_image, int& status){
        mm_pattern_recognition::RecognizeStatus srv;
        cv::Rect2f left_roi, right_roi;
        ROIMsgToCv(last_request.left_roi, left_roi);
        ROIMsgToCv(last_request.right_roi, right_roi);
        cv::Mat left_image_roi = left_image(left_roi);
        cv::Mat right_image_roi = right_image(right_roi);
    
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_image_roi).toImageMsg();
        srv.request.image_message = *msg;
        if (remote_switch_recognize_client.call(srv))
        {
            if(srv.response.status.size() > 0)
                status = srv.response.status[0];
            else{
                ROS_ERROR("Failed to call service recognize_remote_switch_status, nothing is return");
                return false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
            return false;
        }

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_image_roi).toImageMsg();
        srv.request.image_message = *msg;
        if (remote_switch_recognize_client.call(srv))
        {
            if(srv.response.status.size() > 0){
                if(status != srv.response.status[0]){
                    ROS_ERROR("remote switch's status is not same in left/right image");
                    return false;
                }
            }
            else{
                ROS_ERROR("Failed to call service recognize_remote_switch_status, nothing is return");
                return false;
            }

        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
            return false;
        }
        return true;

    }
    void step(){
        if(!finished){
            mm_robot_decision::VisualAppResponse response;
            response.object_name = last_request.object_name;
            response.frame_id = ENDEFFECTOR_FRAME_NAME;
            response.object_unique_id_on_equipment = last_request.object_unique_id_on_equipment;
            response.pose.state = "WorkSpacePlanning";
            cv::Mat left_image, right_image;
            geometry_msgs::TransformStamped transform_endeffector_to_base;
            ros::Time stamp;
            grabImagesAndTransform(left_image, right_image, 
                                transform_endeffector_to_base, stamp);
            Eigen::Matrix4d T_endeffector_to_base;
            transformToMatrix(transform_endeffector_to_base, T_endeffector_to_base);
            
            Eigen::Matrix4d rough_T_base_to_obj, rough_T_cam_to_obj, rough_T_endeffector_to_obj;
            
            transformToMatrix(last_request.transform_base_to_obj, rough_T_base_to_obj);
            // return directly the request
            rough_T_endeffector_to_obj = T_endeffector_to_base * rough_T_base_to_obj;
            transformToPose4WithQuaternion(rough_T_endeffector_to_obj, response.pose);
            response.success = true;
            if(last_request.object_name == REMOTE_SWITCH_NAME){
                int status;
                if(recognizeRemoteSwitch(left_image, right_image, status)){
                    response.object_status.push_back(status);
                }
                else{
                    response.success = false;
                }
                
            }
            // ROS_WARN("last_request.object_height=%f, last_request.object_width=%f", last_request.object_height, last_request.object_width);
            response.height = last_request.object_height;
            response.width = last_request.object_width;

            std::cout<<response<<std::endl;
            result_pub.publish(response);
            finished = true;

            return;
            /*
            rough_T_cam_to_obj = T_cam_to_endeffector * T_endeffector_to_base * rough_T_base_to_obj;
            impl.setSwitchParamsToRecognize(last_request.object_name ,last_request.object_width, last_request.object_height, rough_T_cam_to_obj);
        
            cv::Rect roi_left_rect, roi_right_rect;
            roi_left_rect.x = (int)last_request.left_roi.x -30;
            roi_left_rect.y = (int)last_request.left_roi.y -30;
            roi_left_rect.width = (int)last_request.left_roi.width +60;
            roi_left_rect.height = (int)last_request.left_roi.height + 60;
            
            roi_right_rect.x = (int)last_request.right_roi.x -30;
            roi_right_rect.y = (int)last_request.right_roi.y -30;
            roi_right_rect.width = (int)last_request.right_roi.width +60;
            roi_right_rect.height = (int)last_request.right_roi.height + 60;
            boundRectROI(roi_left_rect, left_image.size());
            boundRectROI(roi_right_rect, right_image.size());
            

            std::vector<SwitchPtr> all_switches;
            cv::Mat left_roi_img = left_image(roi_left_rect);
            cv::Mat right_roi_img = right_image(roi_right_rect);
            impl.findAllSwitches(left_roi_img, right_roi_img, roi_left_rect, roi_right_rect, all_switches, false);

            if(all_switches.size() == 0){
                // failed
                ROS_WARN("failed to detect the object [%s]", last_request.object_name.c_str());
                response.success = false;
            }
            else{
                // success
            
                if(all_switches.size() > 1){
                    // failed
                    ROS_WARN("detected too much object that satisfied the constraints of [%s]...", last_request.object_name.c_str());
                }
                response.success = true;
                response.width = all_switches[0]->width;
                response.height = all_switches[0]->height;
                Eigen::Matrix4d T_endeffector_to_switch = model_ptr->endeffector_to_cam_transform * all_switches[0]->T_cam_to_switch;
                transformToPose4WithQuaternion(T_endeffector_to_switch, response.pose);
                response.pose.state = "WorkSpacePlanning";

                // recognize the status
                if(last_request.object_name == REMOTE_SWITCH_NAME){
                    if(impl.templates_remote_switch_status_0.empty() || impl.templates_remote_switch_status_1.empty()){
                        if(last_request.additional_text_info == "status_0" || last_request.additional_text_info == "status_1"){
                            ROS_INFO("Start to grab the template for [%s] of [%s]", last_request.additional_text_info.c_str(), REMOTE_SWITCH_NAME);
                            cv::Mat corrected_left_switch, corrected_right_switch;
                            impl.getCorrectedSwitch(left_roi_img,  *(all_switches[0]), cv::Size(200,200), corrected_left_switch, LEFT_CAMERA);
                            impl.getCorrectedSwitch(right_roi_img,  *(all_switches[0]), cv::Size(200,200), corrected_right_switch, RIGHT_CAMERA);

                            std::string path = ros::package::getPath("mm_visual_postion");

                            path += "/template_images/remote_switch/" + last_request.additional_text_info;
                            if(!isDirExist(path)){
                                makePath(path);
                            }
                            if(cv::imwrite(path +"/0.png", corrected_left_switch) && cv::imwrite(path +"/1.png", corrected_right_switch)){
                                ROS_INFO("sucessfully write the template to [%s]", path.c_str());
                            }
                            else{
                                ROS_WARN("Failed to write the tamplate to [%s], please check whether that path existed", path.c_str());
                                response.success = false;
                            }
                        }
                        ROS_ERROR("Unknow additional_text_info [%s]", last_request.additional_text_info.c_str());
                    }
                    else{
                        int left_status, right_status;
                        all_switches[0]->name = last_request.object_name;
                        impl.getStatus(left_roi_img, *(all_switches[0]), left_status, LEFT_CAMERA);
                        impl.getStatus(right_roi_img, *(all_switches[0]), right_status, RIGHT_CAMERA);
                        if(left_status == right_status){
                            response.object_status.push_back(left_status);
                        }
                        else{
                            ROS_WARN("The status result from left/right camera are not the same.");
                            response.success = false;

                        }
                    
                    }
                }
            }
            std::cout<<response<<std::endl;
            result_pub.publish(response);
            finished = true;
            */
        }
        
    }

    void run(){
        while(ros::ok()){
            step();
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        }
    }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "mm_visual_rect_switch_find_node");
	ros::NodeHandle nh;
    RectSwitchFindNode rect_switch_find_node(nh);
    rect_switch_find_node.run();
}