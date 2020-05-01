#ifndef __VISUAL_SAFE_WRAPPER_H__
#define __VISUAL_SAFE_WRAPPER_H__
#include "mm_visual_postion/visual_wrapper/VisualCabinet.h"
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "mm_robot_decision/pose4.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_robot_decision/VisualAppRequest.h"
#include "mm_visual_postion/AppInnerRequest.h"
#include "mm_visual_postion/EquipmentPose.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/utils/ArmCameraControlBase.h"
#include <ros/callback_queue.h>
#include <random>
#include "mm_visual_postion/utils/ObjectDefinition.h"
#include "mm_visual_postion/utils/IteractFindCorners.h"
#include "mm_visual_postion/utils/CheckCreatePath.h"
#include <image_transport/image_transport.h>
#include <map>
#include "mm_endmotor_control/motorCommand.h"
#include "std_msgs/Int8.h"

#define STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD 0
#define STATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE 1 // retreive params from database
#define STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_FIRST_QRCODE 2 // detect the first qr code
#define STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_NEXT_QRCODE 3 // detect the second qr code
#define STATE_INITIALIZATION_OF_VISUAL_CABINET_COMPUTE 4 // compute the T_base_to_cabinet
#define STATE_WAIT_FOR_OTHER_CMD 5
#define STATE_INNER_FAILURE 6
#define STATE_PROCESS_OTHER_CMD 7
#define STATE_ABORT_TASK 8
#define STATE_WAIT_FOR_SPECIFIC_CMD 9

#define DEBUG_DONT_CHECK_WHETHER_BASE_MOVED 
class VisualAppNodeCaller{
protected:
    ros::Subscriber ret_sub;
    ros::Publisher cmd_pub;
    bool finished; // attention: this is not thread safe, thus do not used in multi-thread node. 
    ros::NodeHandle member_nh;
    ros::CallbackQueue member_callback_queue;
    std::string sub_topic;
public:
    const std::string object_name;
    mm_robot_decision::VisualAppResponse last_recv_msg;

    VisualAppNodeCaller(ros::NodeHandle& nh, std::string object_name_input, std::string cmd_pub_topic, std::string ret_recv_topic)
        :member_nh(nh,object_name_input), object_name(object_name_input)
    {
        member_nh.setCallbackQueue(&member_callback_queue);
        cmd_pub = member_nh.advertise<mm_visual_postion::AppInnerRequest>(cmd_pub_topic, 2);
		ret_sub = member_nh.subscribe(ret_recv_topic, 2, &VisualAppNodeCaller::retCallback, this);
        finished = true;
        sub_topic = ret_recv_topic;
    }
    void sendDefaultCmd(cv::Rect2f left_roi, cv::Rect2f right_roi){
        mm_visual_postion::AppInnerRequest req;
        req.frame_id = ENDEFFECTOR_FRAME_NAME;
        req.object_name = object_name;
        ROICvToMsg(left_roi, req.left_roi);
        ROICvToMsg(right_roi, req.right_roi);
        cmd_pub.publish(req);
        finished = false;
    }
    void sendCmd(mm_visual_postion::AppInnerRequest& pub_msg){
        cmd_pub.publish(pub_msg);
        finished = false;
    }
    void retCallback(const mm_robot_decision::VisualAppResponse::ConstPtr &msg){
        if(object_name == msg->object_name){
            last_recv_msg = *msg;
            finished = true;
        }
    }

    bool checkFinished(){
        return finished;
    }
    bool waitToFinish(double timeout_seconds, volatile bool abort_condition){
        // attention: the thread will be blocked
        const double loop_sleep_time = 0.05;
        int wait_loops = timeout_seconds / loop_sleep_time;
        for(int i=0; i<wait_loops; i++){
            if(finished){
                return true;
            }
            if(abort_condition){
                return false;
            }
            ros::Duration(loop_sleep_time).sleep();
            spinOnce();    
        }
        ROS_ERROR("Timeout for receiving result from topic [%s]", sub_topic.c_str());
        return false; //time out;
    }
    void spinOnce(){
        member_callback_queue.callAvailable(ros::WallDuration()); // automatically spin (only for this member node's callback)
    }
};

typedef std::shared_ptr<VisualAppNodeCaller> VisualAppNodeCallerPtr;

class EndMoterCommander{
protected:
    ros::Subscriber ret_sub;
    ros::Publisher cmd_pub;
    bool finished; // attention: this is not thread safe, thus do not used in multi-thread node. 
    ros::NodeHandle member_nh;
    ros::CallbackQueue member_callback_queue;
    int ret_status;
public:
    EndMoterCommander(ros::NodeHandle& nh)
        :member_nh(nh)
    {
        member_nh.setCallbackQueue(&member_callback_queue);
        cmd_pub = member_nh.advertise<mm_endmotor_control::motorCommand>("/mm_motor/motor_command", 1);
		ret_sub = member_nh.subscribe("/mm_motor/isArrived", 1, &EndMoterCommander::retCallback, this);
        finished = true;
    }
    
    void sendCmd(double& degree){
        mm_endmotor_control::motorCommand cmd;
        cmd.workstate = "move";
        cmd.degree = degree;

        finished = false;
        cmd_pub.publish(cmd);
    }
    void retCallback(const std_msgs::Int8::ConstPtr &msg){
        ret_status = msg->data;
        if(ret_status == 1)
            finished = true;
    }
    bool waitToFinish(double timeout_seconds, volatile bool abort_condition){
        // attention: the thread will be blocked
        const double loop_sleep_time = 0.05;
        int wait_loops = timeout_seconds / loop_sleep_time;
        for(int i=0; i<wait_loops; i++){
            if(finished){
                return true;
            }
            if(abort_condition){
                return false;
            }
            ros::Duration(loop_sleep_time).sleep();
            spinOnce();    
        }
        ROS_ERROR("Timeout for receiving result from topic /mm_motor/isArrived");
        return false; //time out;
    }
    void spinOnce(){
        member_callback_queue.callAvailable(ros::WallDuration()); // automatically spin (only for this member node's callback)
    }
};

class VisualSafeWrapper: public ArmCameraControlBase{
protected:
    std::map<std::string, VisualAppNodeCallerPtr> node_caller_ptr_map;
    EndMoterCommander endmotor_commander;
    std::shared_ptr<VisualCabinet> visual_cabinet_ptr;
    
    ros::Subscriber wrapper_cmd_sub, wrapper_refine_cmd_sub, wrapper_grab_data_cmd_sub,wrapper_grab_template_cmd_sub, wrapper_re_read_database_sub;
    ros::Subscriber wrapper_get_cabinet_pose_cmd_sub, wrapper_get_correct_offset, wrapper_get_pose_without_offset;
    ros::Publisher wrapper_cabinet_pose_pub;
    image_transport::Publisher wrapper_visualization_pub;
    ros::Publisher wrapper_result_pub;
    ros::Publisher wrapper_status_pub;

    mm_robot_decision::VisualAppRequest registered_cmd, specific_cmd_wait_for;
    mm_robot_decision::VisualAppResponse last_qr_response_in_endeffector;
    std::vector<AppResponsesWithTransform> qr_codes_responses_msg_in_endeffector_with_transform_vec;
    volatile bool abort_execution;

    int try_num;
    std::random_device rd;
    std::mt19937 gen;
    //bool solved_T_base_to_cabinet; // whether the transform from base to cabinet has been solved.
    Eigen::Vector3d inital_base_pose_in_map; // this usually stores the inital pose when capture the switches
    Eigen::Vector3d base_pose_in_map;

    int current_state;
    ros::NodeHandle member_nh;
    image_transport::ImageTransport it;

    ros::CallbackQueue member_queue;
    bool enable_teaching;
    bool enable_visualization_pub;
    double tool_to_endeffector_mm; // in mm, change this value in "endeffector_to_tool_length.yaml"
    double z_offset_of_teaching_mode;
    mm_robot_decision::VisualAppResponse responses_msg_without_offset_in_base_for_teaching;
    Eigen::Matrix4d T_endeffector_to_base_for_teaching;
    
public:
    VisualSafeWrapper(ros::NodeHandle& nh, bool enable_teaching);
    void loadToolToEndeffector();
    int getCurrentState();
    void spinOnce();
    bool getBasePosition();
    bool checkBaseMovedFromInitalPose();
    void requestCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg);
    bool checkRelatedPoseWithDataBase(std::vector<AppResponsesWithTransform>& responses_in_endeffector_with_transform_vec);
    bool checkGlobalPoseWithDataBase(std::vector<AppResponsesWithTransform>& responses_in_endeffector_with_transform_vec);
    void refineVisualCabinetCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg);
    void reReadDatabaseCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg);

    bool refineVisualCabinet(const int& object_unique_id_on_equipment, int cabinet_id, int cabinet_type_id);
    bool initVisualCabinet(int cabinet_id, int cabinet_type_id, bool teach_mode);

    bool adjustArmToCaptureObject(const int& object_unique_id_on_equipment, bool add_random_move);
    bool moveArmWorkSpacePlanningWithTimeoutConstraintInToolSpace(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double seconds);
    bool moveArmWithTimeoutConstraintInToolSpace(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double seconds);
    bool getROI(const int& object_unique_id_on_equipment, Eigen::Matrix4d& T_endeffector_to_base, cv::Rect& left_roi, cv::Rect& right_roi, bool larger_roi);
    

    bool oneTryForApps(std::vector<mm_visual_postion::AppInnerRequest>& request_cmd_vec, 
                        std::vector<AppResponsesWithTransform>& responses_msg_with_transform_vec, bool use_roi);
    bool getResponseForApps(std::vector<mm_visual_postion::AppInnerRequest>& request_cmd_vec,
                        std::vector<AppResponsesWithTransform>& responses_msg_with_transform_vec,
                        bool move_to_best_capture_pose, bool use_roi,
                        bool check_pose_coherency, bool check_status_coherency, bool check_database_coherency );
    
    void publishResult(mm_robot_decision::VisualAppResponse& response);
    void addOffsetToResultInEndeffector(mm_robot_decision::VisualAppResponse& res_in_endeffector);
    void processForSTATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD();
    void processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE();
    void processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_QRCODE();
    void processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_COMPUTE();
    void processForSTATE_WAIT_FOR_OTHER_CMD();

    void processForSTATE_WAIT_FOR_SPECIFIC_CMD();

    void processForSTATE_PROCESS_OTHER_CMD();
    void generateRandomMoveCmd(double&x, double& y, double &z, bool normalized=true);
    void processForSTATE_INNER_FAILURE();
    void processForSTATE_ABORT_TASK();
    void run();
    void step();
    void manuelPointOutRect(const cv::Mat& left_image_roi, const cv::Rect& left_rect_roi, const Eigen::Matrix4d& T_cam_to_base, VisualObject& visual_obj_in_base);
    bool manuelRefine(const int& object_unique_id_on_equipment, VisualObject& visual_obj_in_base);
    void grabDataCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg);
    void grabTemplateCallback(const mm_visual_postion::AppInnerRequest::ConstPtr& msg);
    void visualizationConnectCallback();
    void visualizationDisconnectCallback();
    void visualizationPublishImage(const cv::Mat& image);
    void requestCabinetPoseCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg);
    void getPoseWithoutOffsetCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg);
    void saveCurrentPoseOffsetCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg);
    bool checkTypeNameExist(const std::string name);
};




#endif