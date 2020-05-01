#include "gtest/gtest.h"
#include "ros/ros.h"
#include "mm_visual_postion/visual_wrapper/VisualSafeWrapper.h"
#include "mm_visual_postion/test/GoldenDataFoVisualCabinet.h"
#include "mm_visual_postion/test/NodeSimulator.h"
#include "mm_robot_decision/pose4.h"

#include "mm_visual_postion/AppInnerRequest.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_robot_decision/VisualAppRequest.h"
#include <ros/callback_queue.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class VisualSafeWrapperTestSuite : public ::testing::Test{
  private:
    VisualNodeSimulatorVector visual_app_nodes_;
    NodeSimulator<mm_robot_decision::pose4,  std_msgs::Int8> arm_node_;
    NodeSimulator<mm_robot_decision::VisualAppRequest, mm_robot_decision::VisualAppResponse> decision_node_;
    
    ros::CallbackQueue call_back_queue_;
    ros::NodeHandle& nh_;
    
    std::unique_ptr<VisualSafeWrapper> wrapper_ptr__;
    
    std::shared_ptr<StereoCameraArmModel> model_ptr_;


  public:
    
    VisualSafeWrapperTestSuite()
    :nh_(ros::NodeHandle()),
    arm_node_(nh_, "/mm_arm/goal", "/mm_arm/isArrived"),
    decision_node_(nh_, "mm_visual/wrapper/request", "/mm_visual/wrapper/response");
    {
        wrapper_ptr_ = new(VisualSafeWrapper(nh_));
        nh_.setCallbackQueue(&call_back_queue_);
        visual_app_nodes_.push_back_new(nh_, REMOTE_SWITCH_NAME, "/mm_visual/calcuPose","/mm_visual/switchPose");
        visual_app_nodes_.push_back_new(nh_, KNIFE_SWITCH_NAME, "/mm_visual/calcuPose","/mm_visual/switchPose");
        visual_app_nodes_.push_back_new(nh_, HANDCART_SWITCH_NAME, "/mm_visual/calcuPose","/mm_visual/switchPose");
        visual_app_nodes_.push_back_new(nh_, LIGHTS_NAME, "/mm_recognization/lightStateOrder","/mm_recognization/lightState");
        visual_app_nodes_.push_back_new(nh_, POINT_METERS_NAME, "/mm_point_meter/goal","/mm_point_meter/result");  
        visual_app_nodes_.push_back_new(nh_, DIGITAL_METERS_NAME, "/mm_digital_meter/goal","/mm_digital_meter/result");              
        StereoImageReceiver image_receiver(nh_);
        image_receiver.getArmCameraModel(model_ptr_);
    }
    void publishCmdToWrapper(std::string name, int equipment_id, int equipment_type){
        mm_robot_decision::VisualAppRequest msg;
        msg.object_name = name;
        msg.equipment_id = equipment_id;
        msg.equipment_type = equipment_type;
        decision_node_.pubMsg(msg);
    }
    void boardcastBasePose(double x, double y, double z){
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_imu";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
    }
    void boardcastEndeffectorToBase(Eigen::Matrix4d T_base_to_endeffector){
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base";
        transformStamped.child_frame_id = "tool0_controller";
        transformStamped.transform.translation.x = T_base_to_endeffector(0,3);
        transformStamped.transform.translation.y = T_base_to_endeffector(1,3);
        transformStamped.transform.translation.z = T_base_to_endeffector(2,3);
        Eigen::Quaterniond q(T_base_to_endeffector.block<3,3>(0,0));
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
    }
    void spinOnce(){
        call_back_queue_.callAvailable(ros::WallDuration());
    }
    

    void sendInfoToSTATE_INITIALIZATION_OF_VISUAL_CABINET(){
        // send initialization required info to simulation node
        boardcastBasePose(12,15,0.5);// base pose in map

        Eigen::Matrix4d golden_T_base_to_endeffector;
        VisualCabinet golden_cabinet = createGoldenDataForStandardVisualCabinet();
        
        // assuming T from cam to remote switch
        Eigen::Matrix4d golden_T_cam_to_remote_switch;
        golden_T_cam_to_remote_switch << 1,0,0,-0.2,
                                0,1,0,0,
                                0,0,1,0.5
                                0,0,0;
        Eigen::Matrix4d golden_T_base_to_cabinet;
        assumeT_base_to_cabinet(golden_T_base_to_cabinet);
        Eigen::Matrix4d T_cabinet_to_remote_switch;
        golden_cabinet[REMOTE_SWITCH_NAME].getTransformFromPresentedFrameToObj(T_cabinet_to_remote_switch);
        golden_T_base_to_endeffector = golden_T_base_to_cabinet * T_cabinet_to_remote_switch 
                                        * golden_T_cam_to_remote_switch.inverse()
                                        * model_ptr_->endeffector_to_cam_transform.inverse();
    
    }

    ~VisualSafeWrapperTestSuite() {}
};

TEST_F(VisualSafeWrapperTestSuite, WAIT_FOR_INIT_VISUAL_CABINET_CMD_to_INITIALIZATION_OF_VISUAL_CABINET){
    publishCmdToWrapper(CMD_INIT_CABINET, 0,0);
    spinOnce();
    ros::spinOnce();
    wrapper_ptr_->step();

    // check for state
    ASSERT_EQ(wrapper_ptr_->getCurrentState == STATE_INITIALIZATION_OF_VISUAL_CABINET);
    



    

    
    

}









TEST_F(VisualSafeWrapperTestSuite)








int main(int argc, char** argv){
  ros::init(argc, argv, "VisualSafeWrapperTestNode");
  testing::InitGoogleTest(&argc, argv);

  thread t([]{while(ros::ok()) ros::spin();});

  auto res = RUN_ALL_TESTS();

  ros::shutdown();
  return res;
}