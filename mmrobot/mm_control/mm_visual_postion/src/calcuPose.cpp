//#include"ZedCalib.h"
#include "ros/ros.h"

#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/AppInnerRequest.h"

//#include"camera.h"
#include <opencv2/opencv.hpp>
#include <string.h>
#include "cameraZedUVC.h"
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include "mm_visual_postion/utils/StereoImageReceiver.h"
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
#include "mm_visual_postion/switch/CoordinateCalculation.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/hand_trunck/HandTrunkSolver.h"
#include "mm_visual_postion/visual_wrapper/VisualObject.h"
#include <ros/package.h>

using namespace std;



class PoseOrderSubscribeAndPublish
{
  private:
	std::string lastOrder;
	bool isLastOrderValid;
	tf2_ros::Buffer tfBuffer;
	std::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;


	cv::Mat latest_left_image, latest_right_image;
	StereoImageReceiver image_receiver;
	ros::Time latest_update_time;
	bool camera_params_updated;
	sensor_msgs::CameraInfo left_camera_info, right_camera_info;

	ros::NodeHandle cameraNode;
	ros::Publisher posePub;
	ros::Subscriber calcuSub;
	ros::Subscriber zedImageSub;
	CoordinateCalculation coordCal;
	std::shared_ptr<StereoCameraArmModel> model_ptr;
	std::vector<cv::Mat> templateRemoteSwitchRemoteStatusVec, templateRemoteSwitchGroundStatusVec, templateKnifeSwitchVec;
	cv::Vec4f templateMargin;
  public:
	PoseOrderSubscribeAndPublish(ros::NodeHandle &n) : cameraNode(n), 
														image_receiver(n),
														camera_params_updated(false){}
	void SubscribeAndPublish()
	{

		posePub = cameraNode.advertise<mm_robot_decision::VisualAppResponse>("/mm_visual/apps/switch/result", 100);
		calcuSub = cameraNode.subscribe("/mm_visual/apps/switch/goal", 20, &PoseOrderSubscribeAndPublish::calcuCallback, this);
		std::string path = ros::package::getPath("mm_visual_postion");

		readImagesInFolder(path + "/template_images/remote_switch/remote_state", templateRemoteSwitchRemoteStatusVec);
        if (templateRemoteSwitchRemoteStatusVec.empty())
            ROS_ERROR("cannot load template image for remote switch in remote state");
		else{
			ROS_INFO("loaded %d template images for remote switch in remote state", (int)(templateRemoteSwitchRemoteStatusVec.size()));
		}

		
		readImagesInFolder(path + "/template_images/remote_switch/ground_state", templateRemoteSwitchGroundStatusVec);
        if (templateRemoteSwitchGroundStatusVec.empty())
            ROS_ERROR("cannot load template image for remote switch in ground state");
		else{
			ROS_INFO("loaded %d template images for remote switch in ground state", (int)(templateRemoteSwitchGroundStatusVec.size()));
		}

		
		readImagesInFolder(path + "/template_images/knife_switch", templateKnifeSwitchVec);
        if (templateKnifeSwitchVec.empty())
            ROS_ERROR("cannot load template image for knife_switch");
		else{
			ROS_INFO("loaded %d template images for knife switch", (int)(templateKnifeSwitchVec.size()));
		}


		templateMargin = Vec4f(0.0, 0.0, 0.0, 0.0); //up, down, left, right, //margin to the inner edge (deprecated)

		isLastOrderValid = false;

		tfListenerPtr = std::make_shared<tf2_ros::TransformListener>(tfBuffer);

	}
	void readImagesInFolder(std::string folder_name, std::vector<cv::Mat>& images){
		// the filename of images should be 0.bmp, 1.bmp, 2.bmp and so on.
		int id = 0;
		while(true){
			cv::Mat one_image = cv::imread(folder_name +"/" + std::to_string(id) + std::string(".bmp"));
			id ++;
			if(!one_image.empty()){
				images.push_back(one_image);
			}
			else{
				return;
			}
		}
	}
	void calcuCallback(const mm_visual_postion::AppInnerRequest::ConstPtr &msg)
	{
		assert(msg->frame_id == ENDEFFECTOR_FRAME_NAME);
		ROS_INFO("Copy that");
		cout << msg->object_name.c_str() << endl;
		lastOrder = msg->object_name;
		isLastOrderValid = true;
	}
	void publishFailedMsg(){
		mm_robot_decision::VisualAppResponse response;
		response.frame_id = ENDEFFECTOR_FRAME_NAME;
		response.object_name = lastOrder;
		response.success = false;
		posePub.publish(response);
	}
	void executeOrder()
	{

		///////////////////////////////////
		////////      OFFSET      /////////
		///////////////////////////////////
		///////////////////////////////////
		double a_angle_offset = 180.0; //deg
		double x_offset= 0.0; //mm
		double y_offset = 0.0;
		double z_offset = 0.0; //mm

		if(!camera_params_updated){
			if(!image_receiver.cameraInfoGot()){
				ROS_INFO("Wait for camera info msg...");
				return;
			}
			else{
				
				image_receiver.getArmCameraModel(model_ptr);
				coordCal.updateCameraArmParams(model_ptr);
				camera_params_updated = true;
			}	
		}
		
		if (isLastOrderValid && image_receiver.getLatestImages(latest_left_image, latest_right_image, latest_update_time))
		{
			isLastOrderValid = false; //last order is excuted

			geometry_msgs::TransformStamped transform_endeffector_to_base;
			#ifndef BYPASS_TF
				
				try
				{
					transform_endeffector_to_base = tfBuffer.lookupTransform("tool0_controller", "base", latest_update_time);
				}
				catch (tf::TransformException ex)
				{
					ROS_ERROR("%s", ex.what());
					publishFailedMsg();
					return;
				}
			#endif

			vector<double> xyzabc;
			int return_val;
			ResultInfo* result_info_ptr;
			ResultInfo result_info; 
			result_info_ptr = & result_info;


			mm_robot_decision::VisualAppResponse response;
			response.frame_id = ENDEFFECTOR_FRAME_NAME;
			response.object_name = lastOrder;

			if (lastOrder == KNIFE_SWITCH_NAME)
			{
				double match_score;
				return_val = coordCal.calculateSwitchPosSimple(latest_left_image, latest_right_image, templateKnifeSwitchVec,templateMargin,
																transform_endeffector_to_base, xyzabc,
																METHOD_HOUGH, match_score, result_info_ptr);
			}
			else if (lastOrder == REMOTE_SWITCH_NAME)
			{
				double ground_match_score, remote_match_score;
				int ground_state_return_val = coordCal.calculateSwitchPosSimple(latest_left_image, latest_right_image, templateRemoteSwitchGroundStatusVec, 
																templateMargin, transform_endeffector_to_base, xyzabc, 
																METHOD_HOUGH, ground_match_score, result_info_ptr);
			
				int remote_state_return_val = coordCal.calculateSwitchPosSimple(latest_left_image, latest_right_image, templateRemoteSwitchRemoteStatusVec, 
																templateMargin, transform_endeffector_to_base, xyzabc, 
																METHOD_HOUGH, remote_match_score, result_info_ptr);
				if(ground_state_return_val == 0 && remote_state_return_val == 0 ){
					if(ground_match_score - remote_match_score > 0.05){
						remote_state_return_val = -1;
						return_val = 0;
					}
					else if(remote_match_score - ground_match_score > 0.05){
						ground_state_return_val = -1;
						return_val = 0;
					}
					else{
					ROS_WARN("found the remote switch, but not sure it is in which state...");
					return_val = -1;
				}
				}
				
				if(ground_state_return_val < 0 && remote_state_return_val == 0){
					ROS_INFO("found the remote switch in remote state");
					return_val = 0;
					response.object_status.push_back(0);
				}
				else if(ground_state_return_val == 0 && remote_state_return_val != 0){
					ROS_INFO("found the remote switch in ground state");
					return_val = 0;
					response.object_status.push_back(1);
				}
				else if(ground_state_return_val < 0 && remote_state_return_val < 0){
					ROS_WARN("cannot found the remote switch in remote state or in ground state");
					return_val = -1;
				}
			}
			else if (lastOrder == HANDCART_SWITCH_NAME)
			{
				HandTrunkSolver handtrunk_solver(model_ptr);

				std::vector<cv::Mat> latest_left_image_vec;
				latest_left_image_vec.push_back(latest_left_image);
				std::vector<cv::Mat> latest_right_image_vec;
				latest_right_image_vec.push_back(latest_right_image);
				std::vector<Eigen::Matrix4d> transform_cam_to_base_vec(1);
				transformToMatrix(transform_endeffector_to_base, transform_cam_to_base_vec[0]);
				Circle3D result_circle;
				cv::Mat show_img;
				Eigen::Matrix4d T_base_to_trunk;
				if(!handtrunk_solver.findTrunk(latest_left_image_vec, latest_right_image_vec, 
                    transform_cam_to_base_vec,56,3,1000,
                    result_circle, T_base_to_trunk,
                    &show_img)){
						ROS_WARN("didn't find the hand trunk");
					return_val = -1;
				}
				else {
					return_val = 0;
				
					// convert T_base_to_trunk to T_endeffector_to_trunk
					Eigen::Matrix4d T_endeffector_to_trunk = model_ptr->endeffector_to_cam_transform * transform_cam_to_base_vec[0] * T_base_to_trunk;
					
					xyzabc.resize(6);
					xyzabc[0] = T_endeffector_to_trunk(0,3);
					xyzabc[1] = T_endeffector_to_trunk(1,3);
					xyzabc[2] = T_endeffector_to_trunk(2,3);
					Eigen::Vector3d abc = T_endeffector_to_trunk.block<3,3>(0,0).eulerAngles(0,1,2);
					xyzabc[3] = abc(0);
					xyzabc[4] = abc(1);
					xyzabc[5] = abc(2);
				}
				x_offset= 0;//1; //mm
				y_offset =0;//3.5;
			    z_offset = 0; //mm
				a_angle_offset = 0; //deg
				
			}
			else
			{
				ROS_WARN("The switch is not exist");
				return_val = -1;
			}

			if (return_val != 0)
			{
				ROS_WARN("detection failed, please try again");
				publishFailedMsg();
				return;
			}



			response.pose.state = "WorkSpacePlanning";

			// rot from camera to switch
			Eigen::Matrix4d T_endeffector_to_switch;
			T_endeffector_to_switch.setIdentity();
			T_endeffector_to_switch(0,3) = xyzabc[0];
			T_endeffector_to_switch(1,3) = xyzabc[1];
			T_endeffector_to_switch(2,3) = xyzabc[2];

			T_endeffector_to_switch.block<3,3>(0,0) = Eigen::AngleAxisd(xyzabc[3], Eigen::Vector3d::UnitX()).toRotationMatrix()
				* Eigen::AngleAxisd(xyzabc[4], Eigen::Vector3d::UnitY()).toRotationMatrix()
				* Eigen::AngleAxisd(xyzabc[5], Eigen::Vector3d::UnitZ()).toRotationMatrix(); 
			Eigen::Matrix4d T_switch_to_switch_operation;
			T_switch_to_switch_operation.setIdentity();
			T_switch_to_switch_operation(0,3) = x_offset;
			T_switch_to_switch_operation(1,3) = y_offset;
			T_switch_to_switch_operation(2,3) = z_offset;

			T_switch_to_switch_operation.block<3,3>(0,0) = Eigen::AngleAxisd(a_angle_offset * M_PI/180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
			Eigen::Matrix4d T_endeffector_to_switch_operation = T_endeffector_to_switch * T_switch_to_switch_operation;
			
			Eigen::Quaterniond q(T_endeffector_to_switch_operation.block<3,3>(0,0));
			
			response.pose.a = q.x();
			response.pose.b = q.y(); 
			response.pose.c = q.z(); // in endeffector coordinate
			response.pose.w = q.w(); // in endeffector coordinate
			
			response.pose.x = T_endeffector_to_switch_operation(0,3) / 1000.0;
			response.pose.y = T_endeffector_to_switch_operation(1,3)/ 1000.0;
			response.pose.z = T_endeffector_to_switch_operation(2,3)/ 1000.0;

			response.success = true;
			ROS_INFO("The pose of switch is:");
			cout << response.pose << endl;
			cout <<"state: 'WorkSpacePlanning', x: "<<response.pose.x<<", y: "<<response.pose.y<<", z: "<<response.pose.z<<", a: "<<response.pose.a<<
											", b: " << response.pose.b<<", c: "<<response.pose.c<<", w: "<<response.pose.w<<std::endl; 	

			posePub.publish(response);
		}
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mm_visiual_position");
	ros::NodeHandle nh;

	PoseOrderSubscribeAndPublish poseOrderSubAndPub(nh);
	poseOrderSubAndPub.SubscribeAndPublish();
	ros::Rate r(10);
	while (ros::ok())
	{
		poseOrderSubAndPub.executeOrder();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
