#include "mm_visual_postion/switch/RectSwitchFinder.h"
#include "ros/ros.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/utils/ArmCameraControlBase.h"
#include "mm_visual_postion/AppInnerRequest.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/utils/ObjectDefinition.h"

class MeterRecoginizerTestNode: public ArmCameraBase{
private:
    RectSwitchFinder impl;
    Eigen::Matrix4d T_cam_to_endeffector;
    std::string images_path;
public:
    MeterRecoginizerTestNode(ros::NodeHandle& nh, std::string images_path_input): ArmCameraBase(nh){
        impl.setCameraParams(model_ptr);
        T_cam_to_endeffector = model_ptr->endeffector_to_cam_transform.inverse();
        images_path = images_path_input;
        
    }

    void run(int image_id){
        // load image
        cv::Mat left_image, right_image;
        left_image = cv::imread(images_path + "/left_" + std::to_string(image_id) + ".png");
        right_image = cv::imread(images_path + "/right_" + std::to_string(image_id) + ".png");
        
        Eigen::Matrix4d rough_T_cam_to_obj;
        rough_T_cam_to_obj << 1, 0, 0, 51,
                            0, 1, 0, -22,
                            0, 0, 1, 400,
                            0, 0, 0, 1;
        
        impl.setSwitchParamsToRecognize("point_meter", 70, 70, rough_T_cam_to_obj); 
        cv::Rect roi_left_rect, roi_right_rect;
        roi_left_rect.x = 1192 - 20;
        roi_left_rect.y = 336 - 20;
        roi_left_rect.width = 284 + 40;
        roi_left_rect.height = 284 + 40;
        
        roi_right_rect.x = 758 - 20;
        roi_right_rect.y = 432 - 20;
        roi_right_rect.width = 284 + 40;
        roi_right_rect.height = 284 + 40;
        boundRectROI(roi_left_rect, left_image.size());
        boundRectROI(roi_right_rect, right_image.size());
        
        std::vector<SwitchPtr> all_switches;
        cv::Mat left_roi_img = left_image(roi_left_rect);
        cv::Mat right_roi_img = right_image(roi_right_rect);
        impl.findAllSwitches(left_roi_img, right_roi_img, roi_left_rect, roi_right_rect, all_switches, false);

        
        if(all_switches.size() == 0){
            // failed
            ROS_WARN("failed to detect the object");
            return;
        }
        if(all_switches.size() > 1){
            // failed
            ROS_WARN("detected too much object that satisfied the constraints");
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
            return;
        }
        // if angle is bigger then the indicate number is bigger
        double angle_left = left_lines[0][0] > 0 ? left_lines[0][1]- M_PI/2.0 : M_PI/2.0 + left_lines[0][1];
        double angle_right = right_lines[0][0] > 0 ? right_lines[0][1]- M_PI/2.0 : M_PI/2.0 + right_lines[0][1];
        if(fabs(angle_left - angle_right) < M_PI/180.0 * 5){ // accept 5 deg error
            double angle = (angle_left + angle_right)/ 2.0;
            std::cout<<"indicator angle: "<<angle<<std::endl;
            return;
        }
        else{
            ROS_WARN("indicater's degree is not the same on left and right image");
            return;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mm_visual_rect_switch_find_test_node");
    if(argc < 3){
        ROS_ERROR("You should input the image's folder path and the image id");
    }   
    std::string path = argv[1];
    int image_id = std::atoi(argv[2]);
    
    ros::NodeHandle nh;
    MeterRecoginizerTestNode node(nh, path);
    node.run(image_id);
}