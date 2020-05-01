#include "mm_visual_postion/visual_wrapper/VisualSafeWrapper.h"
//#define DEBUG

VisualSafeWrapper::VisualSafeWrapper(ros::NodeHandle& nh, bool enable_teaching)
:ArmCameraControlBase(nh), endmotor_commander(nh),  gen(rd()), member_nh(nh), it(member_nh),  enable_teaching(enable_teaching)
{
    loadToolToEndeffector();
    z_offset_of_teaching_mode = -350; //mm including the tool to endeffector
    member_nh.setCallbackQueue(&member_queue);
    wrapper_result_pub = member_nh.advertise<mm_robot_decision::VisualAppResponse>("/mm_visual/wrapper/response", 1);
    wrapper_cmd_sub = member_nh.subscribe("mm_visual/wrapper/request", 1, &VisualSafeWrapper::requestCallback, this);
    if(enable_teaching){
        wrapper_refine_cmd_sub = member_nh.subscribe("mm_visual/wrapper/refine_cabinet",1,&VisualSafeWrapper::refineVisualCabinetCallback, this);
        wrapper_grab_data_cmd_sub = member_nh.subscribe("mm_visual/wrapper/grab_data",1,&VisualSafeWrapper::grabDataCallback, this);
        wrapper_grab_template_cmd_sub = member_nh.subscribe("mm_visual/wrapper/grab_template", 1, &VisualSafeWrapper::grabTemplateCallback, this);
        wrapper_get_cabinet_pose_cmd_sub = member_nh.subscribe("mm_visual/wrapper/request_cabinet_pose", 1, &VisualSafeWrapper::requestCabinetPoseCallback, this);
        wrapper_cabinet_pose_pub = member_nh.advertise<mm_visual_postion::EquipmentPose>("mm_visual/wrapper/result_cabinet_pose", 1);
        wrapper_get_pose_without_offset = member_nh.subscribe("mm_visual/wrapper/get_pose_without_offset", 1, &VisualSafeWrapper::getPoseWithoutOffsetCallback, this);
        wrapper_get_correct_offset = member_nh.subscribe("mm_visual/wrapper/save_correct_offset", 1, &VisualSafeWrapper::saveCurrentPoseOffsetCallback, this);
    }
    wrapper_re_read_database_sub = member_nh.subscribe("mm_visual/wrapper/re_read_database",1,&VisualSafeWrapper::reReadDatabaseCallback, this);
    node_caller_ptr_map[KNIFE_SWITCH_NAME] = std::make_shared<VisualAppNodeCaller>(nh,KNIFE_SWITCH_NAME, "/mm_visual/apps/rect_switch/goal", "/mm_visual/apps/rect_switch/result");
    node_caller_ptr_map[REMOTE_SWITCH_NAME] = std::make_shared<VisualAppNodeCaller>(nh, REMOTE_SWITCH_NAME,"/mm_visual/apps/rect_switch/goal", "/mm_visual/apps/rect_switch/result");
    node_caller_ptr_map[HANDCART_SWITCH_NAME] = std::make_shared<VisualAppNodeCaller>(nh, HANDCART_SWITCH_NAME,"/mm_visual/apps/hand_cart/goal", "/mm_visual/apps/hand_cart/result");
    node_caller_ptr_map[POINT_METERS_NAME] = std::make_shared<VisualAppNodeCaller>(nh,POINT_METERS_NAME,"/mm_visual/apps/point_meter/goal" ,"/mm_visual/apps/point_meter/result");
    node_caller_ptr_map[DIGITAL_METERS_NAME] = std::make_shared<VisualAppNodeCaller>(nh,DIGITAL_METERS_NAME,"/mm_visual/apps/digital_meter/goal" ,"/mm_visual/apps/digital_meter/result");
    node_caller_ptr_map[LIGHTS_NAME] = std::make_shared<VisualAppNodeCaller>(nh,LIGHTS_NAME,"/mm_visual/apps/light/goal" ,"/mm_visual/apps/light/result");
    node_caller_ptr_map[QRCODE_NAME] = std::make_shared<VisualAppNodeCaller>(nh,QRCODE_NAME,"/mm_visual/apps/qr_code/goal" ,"/mm_visual/apps/qr_code/result");

    image_transport::SubscriberStatusCallback visualization_image_connect_cb =  boost::bind(&VisualSafeWrapper::visualizationConnectCallback, this);
    image_transport::SubscriberStatusCallback visualization_image_disconnect_cb =  boost::bind(&VisualSafeWrapper::visualizationDisconnectCallback, this);
    // see http://answers.ros.org/question/11327/argument-for-subscriberstatuscallback-in-advertise/
    enable_visualization_pub = false;
    wrapper_visualization_pub = it.advertise("/mm_visual/wrapper/visualization", 1, visualization_image_connect_cb, visualization_image_disconnect_cb);
    abort_execution = false;
    //cv::namedWindow("object_roi",0);  
}
void VisualSafeWrapper::loadToolToEndeffector(){
    std::string path = ros::package::getPath("mm_visual_postion");
    std::string file_path = path + "/endeffector_to_tool_length.yaml";
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    std::vector<double> read_data;

    if (!fs.isOpened())
    {
        std::cerr << "failed to open " << file_path << std::endl;
        return;
    }
    fs["endeffector_to_tool"] >> tool_to_endeffector_mm;
    tool_to_endeffector_mm = tool_to_endeffector_mm* 1000.0;
}

void VisualSafeWrapper::visualizationConnectCallback()
{
    enable_visualization_pub = true;
}

void VisualSafeWrapper::visualizationDisconnectCallback()
{
   enable_visualization_pub = false;
}
void VisualSafeWrapper::visualizationPublishImage(const cv::Mat& image){
    if(enable_visualization_pub){
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        wrapper_visualization_pub.publish(msg);
    }
}
void VisualSafeWrapper::publishResult(mm_robot_decision::VisualAppResponse& response){
    
    wrapper_result_pub.publish(response);
    
    std::stringstream ss;
    ss <<"\tsuccess: "<<(response.success ? "True": "False")<<" \n";
    ss <<"\tframe_id: "<<response.frame_id<<" \n";
    ss <<"\tstate: "<< response.pose.state<< ", x: "<<response.pose.x<<", y: "<<response.pose.y<<", z: "<<response.pose.z<<", a: "<<response.pose.a<<
                                    ", b: " << response.pose.b<<", c: "<<response.pose.c<<", w: "<<response.pose.w<<"\n"; 	
    ss <<"\tobject_status: [";
    for(unsigned int i=0; i<response.object_status.size(); i++){
        ss<<(int)(response.object_status[i])<<" ";
    }
    ss<<"]";
    ROS_INFO("published response: \n %s \n", ss.str().c_str());
}
int VisualSafeWrapper::getCurrentState(){
    return current_state;
}
void VisualSafeWrapper::spinOnce(){
    member_queue.callAvailable(ros::WallDuration());
    for(auto it = node_caller_ptr_map.begin(); it != node_caller_ptr_map.end(); it ++){
        it->second->spinOnce();
    }
 }
bool VisualSafeWrapper::getBasePosition(){
    try
    {
        geometry_msgs::TransformStamped transform_map_to_base = tfBuffer.lookupTransform("base_imu", "map", ros::Time(0), ros::Duration(2.0));
        base_pose_in_map(0) = transform_map_to_base.transform.translation.x;
        base_pose_in_map(1) = transform_map_to_base.transform.translation.y;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(transform_map_to_base.transform.rotation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        base_pose_in_map(2) = yaw;
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ROS_ERROR("cannot get base position in map");
        return false;
    }
}
bool VisualSafeWrapper::checkBaseMovedFromInitalPose(){
    #ifdef DEBUG_DONT_CHECK_WHETHER_BASE_MOVED
    return false;

    #else
    if(!getBasePosition()) return true;
    if((inital_base_pose_in_map - base_pose_in_map).norm() > 0.01) return true;
    else return false;

    #endif
}
void VisualSafeWrapper::requestCabinetPoseCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg){
    assert(msg->equipment_id == visual_cabinet_ptr->cabinet_id && msg->equipment_type == visual_cabinet_ptr->cabinet_type_id);
    geometry_msgs::TransformStamped transform_map_to_base = tfBuffer.lookupTransform("base_link", "map", ros::Time(0), ros::Duration(2.0));
    Eigen::Matrix4d T_map_to_base, T_map_to_cabinet;
    transformToMatrix(transform_map_to_base, T_map_to_base);
    visual_cabinet_ptr->getCabinetGlobalPose(T_map_to_base, T_map_to_cabinet);
    mm_visual_postion::EquipmentPose response;
    response.pose.position.x = T_map_to_cabinet(0,3);
    response.pose.position.y = T_map_to_cabinet(1,3);
    response.pose.position.z = T_map_to_cabinet(2,3);
    Eigen::Quaterniond q(T_map_to_cabinet.block<3,3>(0,0));
    response.pose.orientation.x = q.x();
    response.pose.orientation.y = q.y();
    response.pose.orientation.z = q.z();
    response.pose.orientation.w = q.w();

    response.equipment_id = msg->equipment_id;
    response.equipment_type = msg->equipment_type;
    response.equipment_name = visual_cabinet_ptr->cabinet_name;
    ROS_INFO("send pos for equipment: %s" ,visual_cabinet_ptr->cabinet_name.c_str());
    wrapper_cabinet_pose_pub.publish(response);
}
bool VisualSafeWrapper::checkTypeNameExist(const std::string name){
    if(visual_cabinet_ptr == nullptr || !visual_cabinet_ptr->initialized){
        ROS_ERROR("Visual Cabinet is not initialized");
        return false;
    }
    if(visual_cabinet_ptr->id_type_map.find(name) == visual_cabinet_ptr->id_type_map.end()){
        ROS_ERROR("[%s] is not existed in the current visual cabinet", name.c_str());
        return false;
    }
    return true;
}

void VisualSafeWrapper::getPoseWithoutOffsetCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg){
    if( !(msg->equipment_id == visual_cabinet_ptr->cabinet_id && msg->equipment_type == visual_cabinet_ptr->cabinet_type_id)){
        ROS_ERROR("Received the cmd, but invalid, You should check the equipment_id and equipment_type are correct!");
        return;
    }
    if(!checkTypeNameExist(msg->object_name)){
        return;
    }

    assert(visual_cabinet_ptr->initialized);
    std::vector<mm_visual_postion::AppInnerRequest> request_cmd_vec(1);
    std::vector<AppResponsesWithTransform> responses_msg_in_endeffector_with_transform_vec;

    ROS_INFO("Start to get the pose of %s", msg->object_name.c_str());
    visual_cabinet_ptr->getFundamentalInnerRequest(msg->object_unique_id_on_equipment, request_cmd_vec[0]);
    if(!getResponseForApps(request_cmd_vec, responses_msg_in_endeffector_with_transform_vec, true, true, true, true, true)){
        ROS_WARN("failed to find the %s", msg->object_name.c_str());
        return;
    }

    responses_msg_in_endeffector_with_transform_vec[0].averageInBase(responses_msg_without_offset_in_base_for_teaching);
    ROS_INFO("Successfully get the pose of %s from visual apps", msg->object_name.c_str());
    Eigen::Matrix4d T_endeffector_to_base, T_base_to_obj, T_endeffector_to_obj;
    getTransform(T_endeffector_to_base, ros::Time(0));
    msgToT_frame_to_obj(responses_msg_without_offset_in_base_for_teaching, T_base_to_obj, BASE_FRAME_NAME);


    Eigen::Matrix4d T_visual_pose_to_z_back_pose = Eigen::Matrix4d::Identity();
    T_visual_pose_to_z_back_pose(2, 3) = z_offset_of_teaching_mode;
    T_endeffector_to_obj = T_endeffector_to_base * T_base_to_obj * T_visual_pose_to_z_back_pose;
    
    if(msg->object_name == HANDCART_SWITCH_NAME){
        Eigen::Matrix4d T_endeffector_to_plane, T_z_rot; double z_rot;
        seperateZ_rotation(T_endeffector_to_obj, T_endeffector_to_plane, T_z_rot, z_rot);
        T_endeffector_to_obj = T_endeffector_to_plane;
        T_base_to_obj = T_base_to_obj.eval() * T_z_rot.inverse();
        transformToPose4WithQuaternion(T_base_to_obj, responses_msg_without_offset_in_base_for_teaching.pose);
        // wait for endmotor to arrive
        double z_rot_deg = -z_rot * 180.0 / M_PI;
        endmotor_commander.sendCmd(z_rot_deg);
        endmotor_commander.waitToFinish(10.0, abort_execution);
    }

    Eigen::Quaterniond q(T_endeffector_to_obj.block<3,3>(0,0));

    ROS_INFO("Move to the object (with offset z= %f mm",z_offset_of_teaching_mode);   
    
    moveArmWithTimeoutConstraintInToolSpace(T_endeffector_to_obj(0,3), T_endeffector_to_obj(1,3), 
                            T_endeffector_to_obj(2,3), q.x(), q.y(), q.z(), q.w(), 30.0);
    
    ROS_INFO("Please move the endeffector into the correct place and then send msg to [mm_visual/wrapper/save_correct_offset] ");
}
void VisualSafeWrapper::saveCurrentPoseOffsetCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg){
    assert(msg->equipment_id == visual_cabinet_ptr->cabinet_id && msg->equipment_type == visual_cabinet_ptr->cabinet_type_id);
    assert(visual_cabinet_ptr->initialized);

    if(!checkTypeNameExist(msg->object_name)){
        ROS_WARN("Ignore this command");
        return;
    }
    ROS_INFO("Start to correct the offset of %s", msg->object_name.c_str());
    Eigen::Matrix4d T_operation_to_base, T_base_to_obj;
    getTransform(T_operation_to_base, ros::Time(0));
    
    msgToT_frame_to_obj(responses_msg_without_offset_in_base_for_teaching, T_base_to_obj, BASE_FRAME_NAME);

    Eigen::Matrix4d T_obj_to_operation = (T_operation_to_base * T_base_to_obj).inverse();
    Eigen::Matrix4d T_obj_to_tool, T_operation_to_tool;
    T_operation_to_tool.setIdentity();
    T_operation_to_tool(2,3) = - tool_to_endeffector_mm; 
    T_obj_to_tool =  T_obj_to_operation * T_operation_to_tool; 

    visual_cabinet_ptr->visual_objs_in_cabinet_map.at(msg->object_unique_id_on_equipment).result_offset_translate_mm = T_obj_to_tool.block<3,1>(0,3);
    visual_cabinet_ptr->visual_objs_in_cabinet_map.at(msg->object_unique_id_on_equipment).result_offset_euler_deg
             = T_obj_to_tool.block<3,3>(0,0).eulerAngles(0,1,2) * 180.0 / M_PI;
    visual_cabinet_ptr->saveCabinetParams();
    ROS_INFO("saved params");
}
void VisualSafeWrapper::requestCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg){
    ROS_INFO("VisualSafeWrapper receives the request: %s", msg->object_name.c_str());

    if(visual_cabinet_ptr != nullptr && visual_cabinet_ptr->initialized){
        if(visual_cabinet_ptr->visual_objs_in_cabinet_map.at(msg->object_unique_id_on_equipment).object_name != msg->object_name){
            ROS_WARN("the unique id [%d] does not match the object_name [%s]", msg->object_unique_id_on_equipment, msg->object_name.c_str());
            return;
        }
    }
    if(node_caller_ptr_map.find(msg->object_name) == node_caller_ptr_map.end()){
        ROS_WARN("unknown command [%s], ignore it", msg->object_name.c_str());
        return;
    }
    registered_cmd = *msg;
    if(registered_cmd.object_unique_id_on_equipment == 0){
        assert(registered_cmd.object_name == QRCODE_NAME);
        if(current_state != STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_FIRST_QRCODE && current_state != STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD){
            abort_execution = true;
            ROS_INFO("abort tasks in current state.");
        }
        current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE;
    }
    else if(registered_cmd.object_unique_id_on_equipment == 1){
        assert(registered_cmd.object_name == QRCODE_NAME);
        if(current_state != STATE_WAIT_FOR_SPECIFIC_CMD && specific_cmd_wait_for.object_name == registered_cmd.object_name && specific_cmd_wait_for.object_unique_id_on_equipment == registered_cmd.object_unique_id_on_equipment){
            abort_execution = true;
            ROS_INFO("abort tasks in current state.");
        }
        current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_NEXT_QRCODE;
    }
    else{
        if(current_state != STATE_WAIT_FOR_OTHER_CMD){
            abort_execution = true;
            ROS_INFO("abort tasks in current state.");
        }
        current_state = STATE_WAIT_FOR_OTHER_CMD;
    }        
}



void VisualSafeWrapper::grabTemplateCallback(const mm_visual_postion::AppInnerRequest::ConstPtr& msg){
    if(! visual_cabinet_ptr->initialized){
        ROS_WARN("You should firstly initialize the visual cabinet, abort the task...");
        return;
    }
    if(msg->additional_text_info.empty()){
        ROS_WARN("You should put the current status of [%s] in the 'addition_text_info', which could be 'status_0' or 'status_1'", msg->object_name.c_str());
        return;
    }
    if(!checkTypeNameExist(msg->object_name)){
        ROS_WARN("Ignore this command");
        return;
    }
    ROS_INFO("start to grab the template for [%s] ", msg->object_name.c_str());
    assert(visual_cabinet_ptr->visual_objs_in_cabinet_map.at(msg->object_unique_id_on_equipment).object_name == msg->object_name);
    if(!adjustArmToCaptureObject(msg->object_unique_id_on_equipment, false)) return;

    mm_visual_postion::AppInnerRequest inner_request;
    visual_cabinet_ptr->getFundamentalInnerRequest(msg->object_unique_id_on_equipment, inner_request);
    Eigen::Matrix4d T_endeffector_to_base;
    getTransform(T_endeffector_to_base, ros::Time(0));
    cv::Rect left_roi, right_roi;
    visual_cabinet_ptr->getROI(msg->object_unique_id_on_equipment, T_endeffector_to_base, left_roi, right_roi, true);
    ROICvToMsg(left_roi, inner_request.left_roi);
    ROICvToMsg(right_roi, inner_request.right_roi);   

    inner_request.additional_text_info = msg->additional_text_info;

    //cv::Mat left_image, right_image;
    //grabImages(left_image, right_image, ros::Time(0));
    //visual_cabinet_ptr->visualizeObject(msg->object_name, left_image, right_image, T_endeffector_to_base);
    //cv::imshow("object_roi", left_image);
    //cv::waitKey(20);

    node_caller_ptr_map.at(msg->object_name)->sendCmd(inner_request);
    if(node_caller_ptr_map.at(msg->object_name)->waitToFinish(5, abort_execution)){
       ROS_INFO("succesfully grab the template for [%s]", msg->object_name.c_str());
    }

}
void VisualSafeWrapper::grabDataCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg){
    if(! visual_cabinet_ptr->initialized){
        ROS_WARN("You should firstly initialize the visual cabinet, abort the task...");
        return;
    }
    if(!checkTypeNameExist(msg->object_name)){
        ROS_WARN("Ignore this command");
        return;
    }
    ROS_INFO("start to grab data for [%s] ", msg->object_name.c_str());
    std::string save_path = ros::package::getPath("mm_visual_postion");
    save_path = save_path + "/dataset/" + msg->object_name + "/" + std::to_string(msg->equipment_id) + "_" + std::to_string(msg->equipment_type) + "_" + std::to_string(msg->object_unique_id_on_equipment);
    if(!isDirExist(save_path)){
        makePath(save_path);
    }
    ROS_INFO("the data is saved in [%s]", save_path.c_str());

    std::ofstream roi_csv_file;
    roi_csv_file.open(save_path + std::string("/roi.csv"));
    roi_csv_file<<"id,left_roi_x,left_roi_y,left_roi_width,left_roi_height,right_roi_x,right_roi_y,right_roi_width,right_roi_height\n";

    Eigen::Matrix4d T_endeffector_to_base; 
    cv::Mat left_image, right_image;
    cv::Rect left_roi, right_roi;

    if(!adjustArmToCaptureObject(msg->object_unique_id_on_equipment, false)) return;
    int i=0;
    Eigen::Vector3d last_move = Eigen::Vector3d::Zero();
    const int grab_num = 20;
    do{
        getTransform(T_endeffector_to_base, ros::Time(0));

        if(!getROI(msg->object_unique_id_on_equipment, T_endeffector_to_base, left_roi, right_roi, false)) return;
        grabImages(left_image, right_image, ros::Time(0));


        roi_csv_file <<i<<","<<left_roi.x<<","<<left_roi.y<<","<<left_roi.width<<","<<left_roi.height<<","<<right_roi.x<<","<<right_roi.y<<","<<right_roi.width<<","<<right_roi.height<<"\n";
        cv::imwrite(save_path + "/left_"+std::to_string(i)+".png", left_image);
        cv::imwrite(save_path + "/right_"+std::to_string(i)+".png", right_image);

        //visual_cabinet_ptr->visualizeObject(msg->object_name, left_image, right_image, T_endeffector_to_base); //left_image will be modified
        //cv::imshow("object_roi", left_image);
        //cv::waitKey(20);

        ROS_INFO("%d / %d data is grabed", i, grab_num);
        // randomly move the arm
        Eigen::Vector3d current_move, relative_move;
        generateRandomMoveCmd(current_move(0), current_move(1), current_move(2), false);
        relative_move = current_move - last_move;
        last_move = current_move;
        if(!moveArmWithTimeoutConstraintInToolSpace(relative_move(0), relative_move(1), relative_move(2), 0,0,0,1.0, 10.0)) return;
        i ++;
    }while(i <= grab_num);
    ROS_INFO("%d pairs of images are grabed sucessfully", grab_num);
    return;
}
void VisualSafeWrapper::refineVisualCabinetCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg){
    if(refineVisualCabinet(msg->object_unique_id_on_equipment ,msg->equipment_id, msg->equipment_type))
        ROS_INFO("[%s] of equipment id: %d, equipment type: %d is successfully refined", msg->object_name.c_str(), msg->equipment_id, msg->equipment_type);
}
void VisualSafeWrapper::reReadDatabaseCallback(const mm_robot_decision::VisualAppRequest::ConstPtr& msg){
    visual_cabinet_ptr->visual_objs_in_cabinet_map.clear();
    visual_cabinet_ptr->id_type_map.clear();
    visual_cabinet_ptr->retrieveCabinetParams();
    ROS_INFO("params were updated from the database");
}
void VisualSafeWrapper::manuelPointOutRect(const cv::Mat& left_image_roi, const cv::Rect& left_rect_roi, const Eigen::Matrix4d& T_cam_to_base, VisualObject& visual_obj_in_base){
    IteractFindCorners iteract_finder(left_image_roi);
    iteract_finder.start();
    std::vector<cv::Point2f> corners_in_pixel_roi;
    iteract_finder.getCorners(corners_in_pixel_roi);

    // use known cabinet plane to findout the corners in 3d
    Eigen::Vector4d center, plane_params;
    Eigen::Matrix4d rough_T_cabinet_to_obj, rough_T_cam_to_obj;
    visual_cabinet_ptr->visual_objs_in_cabinet_map.at(visual_obj_in_base.object_unique_id_on_equipment).getTransformFromPresentedFrameToObj(rough_T_cabinet_to_obj);
	rough_T_cam_to_obj = T_cam_to_base * visual_cabinet_ptr->T_base_to_cabinet * rough_T_cabinet_to_obj;
    getPlane(rough_T_cam_to_obj, center, plane_params);

    Eigen::Matrix4d M;
    M.block<3,4>(0,0) = model_ptr->left_camera.projection_mat;
    M.block<1,4>(3,0) = plane_params.transpose();
    Eigen::Matrix4d M_inv = M.inverse();

    
    std::vector<Eigen::Vector4d> point_in_plane_vec(4); // in camera coord
    center.setZero();
    for(unsigned int i=0; i<4; i++){
        Eigen::Vector4d point_in_image;
        point_in_image << corners_in_pixel_roi[i].x + left_rect_roi.x , corners_in_pixel_roi[i].y + left_rect_roi.y, 1, 0;
        point_in_plane_vec[i] = M_inv * point_in_image;
        point_in_plane_vec[i](0) /= point_in_plane_vec[i](3);
        point_in_plane_vec[i](1) /= point_in_plane_vec[i](3);
        point_in_plane_vec[i](2) /= point_in_plane_vec[i](3);
        point_in_plane_vec[i](3) = 1.0;

        center += point_in_plane_vec[i];
    }
    
    // we assume the plane (rotation) is correct
    center /= 4.0;
    double width = (point_in_plane_vec[1](0) - point_in_plane_vec[0](0) +  point_in_plane_vec[2](0) - point_in_plane_vec[3](0))/2.0;  //x-axis in camera coord
    double height = (point_in_plane_vec[3](1) - point_in_plane_vec[0](1) + point_in_plane_vec[2](1) - point_in_plane_vec[1](1))/2.0; // y-axis
    Eigen::Vector4d center_in_base = T_cam_to_base.inverse() * center;
    visual_obj_in_base.pos = center_in_base.head<3>();
    visual_obj_in_base.width = width;
    visual_obj_in_base.height = height;
}


bool VisualSafeWrapper::manuelRefine(const int& object_unique_id_on_equipment, VisualObject& visual_obj_in_base){
    // adjust the arm into best capture pose according to the database
    if(!adjustArmToCaptureObject(object_unique_id_on_equipment, false)) return false;
    Eigen::Matrix4d T_endeffector_to_base, T_cam_to_base;
    // save current position of arm
    getTransform(T_endeffector_to_base, ros::Time(0));
    cv::Rect left_roi,right_roi;
    if(!getROI(object_unique_id_on_equipment, T_endeffector_to_base, left_roi, right_roi, enable_teaching)) return false;
    cv::Mat left_image, right_image;
    grabImages(left_image, right_image, ros::Time(0));
    T_cam_to_base = model_ptr->endeffector_to_cam_transform.inverse() * T_endeffector_to_base;

    visual_obj_in_base = visual_cabinet_ptr->visual_objs_in_cabinet_map.at(object_unique_id_on_equipment);// copy all the things
    Eigen::Matrix4d T_cabinet_to_obj, T_base_to_obj;
    visual_obj_in_base.getTransformFromPresentedFrameToObj(T_cabinet_to_obj);
    T_base_to_obj = visual_cabinet_ptr->T_base_to_cabinet * T_cabinet_to_obj;
    visual_obj_in_base.setTransformFromPresentedFrameToObj(T_base_to_obj);
    visual_obj_in_base.presented_frame = VOBJ_BASE_FRAME;

    manuelPointOutRect(left_image(left_roi), left_roi, T_cam_to_base, visual_obj_in_base);

    // visualize
    //visual_cabinet_ptr->visualizeObject(visual_obj_in_base, left_image, right_image, T_endeffector_to_base);
    //cv::imshow("object_roi", right_image);
    //cv::waitKey(20);
    return true;
}
bool VisualSafeWrapper::refineVisualCabinet(const int& object_unique_id_on_equipment, int cabinet_id, int cabinet_type_id){
    ROS_INFO("Start to refine the visual cabinet.");
    assert(visual_cabinet_ptr->initialized && "Visual cabinet should be firstly initialized with roughly measured data");
    std::vector<VisualObject> refine_visual_obj_in_base_vec;
    if(visual_cabinet_ptr->visual_objs_in_cabinet_map.find(object_unique_id_on_equipment) == visual_cabinet_ptr->visual_objs_in_cabinet_map.end()){
        ROS_WARN("This object doesn't exist on this cabinet, ignore this command");
        return true;
    }
    std::string object_name = visual_cabinet_ptr->visual_objs_in_cabinet_map.at(object_unique_id_on_equipment).object_name;
    if(object_name == QRCODE_NAME){
        assert(qr_codes_responses_msg_in_endeffector_with_transform_vec.size() == visual_cabinet_ptr->id_type_map.at(QRCODE_NAME).size());
        for(unsigned int i=0; i<qr_codes_responses_msg_in_endeffector_with_transform_vec.size(); i++){
            if(qr_codes_responses_msg_in_endeffector_with_transform_vec[i].object_unique_id_on_equipment == object_unique_id_on_equipment){
                mm_robot_decision::VisualAppResponse avg_responses_msg_in_base;
                qr_codes_responses_msg_in_endeffector_with_transform_vec[i].averageInBase(avg_responses_msg_in_base);
                VisualObject visual_obj_in_base;
                visual_obj_in_base.fromVisualAppResponseInBase(avg_responses_msg_in_base);
                refine_visual_obj_in_base_vec.push_back(visual_obj_in_base);
                break;
            }
        }
    }
    else if( ! visual_cabinet_ptr->visual_objs_in_cabinet_map.at(object_unique_id_on_equipment).can_auto_refine){
        // cannot automatically refine, use the manual point out method
        VisualObject visual_obj_in_base;
        if(!manuelRefine(object_unique_id_on_equipment, visual_obj_in_base)) return false;
        refine_visual_obj_in_base_vec.push_back(visual_obj_in_base);
    }
    else{
        
        // params for getResponseForApps
        std::vector<mm_visual_postion::AppInnerRequest> request_cmd_vec(1);
        mm_robot_decision::VisualAppResponse avg_responses_msg_in_base;
        std::vector<AppResponsesWithTransform> responses_msg_in_endeffector_with_transform_vec;

        ROS_INFO("Start to refine the %s", object_name.c_str());
        visual_cabinet_ptr->getFundamentalInnerRequest(object_unique_id_on_equipment, request_cmd_vec[0]);
        if(!getResponseForApps(request_cmd_vec, responses_msg_in_endeffector_with_transform_vec, true, true, true, true, true)){
            ROS_WARN("failed to find the %s", object_name.c_str());
            return false;
        }
        responses_msg_in_endeffector_with_transform_vec[0].averageInBase(avg_responses_msg_in_base);



        if(object_name == HANDCART_SWITCH_NAME){
            // we do not store the z-axis rotation of handcart switch
            Eigen::Matrix4d T_base_to_obj, T_cabinet_to_obj_real, T_cabinet_to_obj_origin;
            msgToT_frame_to_obj(avg_responses_msg_in_base, T_base_to_obj, BASE_FRAME_NAME);
            T_cabinet_to_obj_real = visual_cabinet_ptr->T_base_to_cabinet.inverse() * T_base_to_obj;
            visual_cabinet_ptr->visual_objs_in_cabinet_map.at(object_unique_id_on_equipment).getTransformFromPresentedFrameToObj(T_cabinet_to_obj_origin);
            Eigen::Matrix4d T_obj_real_to_obj_origin = T_cabinet_to_obj_real.inverse() * T_cabinet_to_obj_origin;
            Eigen::Vector3d x_unit_vector = T_obj_real_to_obj_origin.block<3,3>(0,0) * Eigen::Vector3d::UnitX();
            double gamma; // angle around z-axis
            gamma = std::atan2(x_unit_vector(1), x_unit_vector(0));
            Eigen::Matrix4d T_z_rot = Eigen::Matrix4d::Identity();
            T_z_rot.block<3,3>(0,0) = Eigen::AngleAxisd(-gamma, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            //Eigen::Matrix4d T_obj_real_to_obj_origin_without_z_rot =  T_z_rot * T_obj_real_to_obj_origin;
            
            
            T_base_to_obj =T_base_to_obj.eval() * T_z_rot.inverse();
            Eigen::Quaterniond new_q(T_base_to_obj.block<3,3>(0,0));
            new_q.normalize();
            avg_responses_msg_in_base.pose.a = new_q.x();
            avg_responses_msg_in_base.pose.b = new_q.y();
            avg_responses_msg_in_base.pose.c = new_q.z();
            avg_responses_msg_in_base.pose.w = new_q.w();   
        }
        
        VisualObject visual_obj_in_base;
        visual_obj_in_base.fromVisualAppResponseInBase(avg_responses_msg_in_base);
        refine_visual_obj_in_base_vec.push_back(visual_obj_in_base);
    
    }
    visual_cabinet_ptr->updateVisualObjInCabinet(refine_visual_obj_in_base_vec);
    visual_cabinet_ptr->saveCabinetParams();
    std::stringstream ss;
    ss<<"You have successfully refined ";
    for(unsigned int i=0; i<refine_visual_obj_in_base_vec.size(); i++){
        ss<<"["<<refine_visual_obj_in_base_vec[i].object_name<<"] ";
    }
    ss<<".";
    std::string print_info = ss.str();
    ROS_INFO_STREAM(print_info);
    return true;
}

bool VisualSafeWrapper::checkRelatedPoseWithDataBase(std::vector<AppResponsesWithTransform>& responses_in_endeffector_with_transform_vec)
{
    if(visual_cabinet_ptr == nullptr){
        ROS_ERROR("visual cabinet is not loaded from the database");
        return false;    
    }

    for(unsigned int i=0; i<responses_in_endeffector_with_transform_vec.size(); i++){
        if(!responses_in_endeffector_with_transform_vec[i].isAverageValid())
            responses_in_endeffector_with_transform_vec[i].averageInBase();
    }

    for(unsigned int i=0; i<responses_in_endeffector_with_transform_vec.size() -1; i++){
        for(unsigned int j=i+1; j<responses_in_endeffector_with_transform_vec.size(); j++){
            mm_robot_decision::VisualAppResponse avg_msg_in_base_i, avg_msg_in_base_j;
            responses_in_endeffector_with_transform_vec[i].averageInBase(avg_msg_in_base_i);
            responses_in_endeffector_with_transform_vec[j].averageInBase(avg_msg_in_base_j);
            // check whether T_obji_to_objj is valid by comparing to the database
            Eigen::Matrix4d T_obj_i_to_obj_j_in_database;

            if(!visual_cabinet_ptr->getTransformFromObj1ToObj2(
                avg_msg_in_base_i.object_unique_id_on_equipment, avg_msg_in_base_j.object_unique_id_on_equipment, 
                T_obj_i_to_obj_j_in_database)){
                ROS_ERROR("cannot find the transfrom from %s to %s in database", avg_msg_in_base_i.object_name.c_str(),  avg_msg_in_base_j.object_name.c_str());
                return false;
            }
            Eigen::Matrix4d T_base_to_obj_i, T_base_to_obj_j;
            msgToT_frame_to_obj(avg_msg_in_base_i, T_base_to_obj_i, "base");
            msgToT_frame_to_obj(avg_msg_in_base_j, T_base_to_obj_j, "base");

            // compare T_obj_i_to_obj_j_in_database and T_obji_to_objj
            Eigen::Matrix4d T_obj_i_to_obj_j_real =  T_base_to_obj_i.inverse() * T_base_to_obj_j;
            //std::cout<<"T_obj_i_to_obj_j: "<< (T_base_to_obj_j.inverse() * T_base_to_obj_i)<<std::endl;
            // T_mul should be enough close to Identity
            if(enable_teaching){
                if(! isTransfromSimilaire(T_obj_i_to_obj_j_real, T_obj_i_to_obj_j_in_database, 100, 0.05)){
                    std::cout<<"In database, T_obj_i_to_j = "<<T_obj_i_to_obj_j_in_database<<std::endl;
                    std::cout<<"however, T_obj_i_to_obj_j_real = "<<T_obj_i_to_obj_j_real<<std::endl;
                    return false;
                }
            }
            else{
                if(! isTransfromSimilaire(T_obj_i_to_obj_j_real, T_obj_i_to_obj_j_in_database, 100, 0.05)) return false;
            }
        }

    }
    return true;

    
}
bool VisualSafeWrapper::checkGlobalPoseWithDataBase(std::vector<AppResponsesWithTransform>& responses_in_endeffector_with_transform_vec)
{
    if(visual_cabinet_ptr == nullptr){
        ROS_ERROR("visual cabinet is not loaded from the database");
        return false;    
    }
    if(!visual_cabinet_ptr->initialized){
        ROS_ERROR("visual cabinet is not initialized");
        return false;
    }
    for(unsigned int i=0; i<responses_in_endeffector_with_transform_vec.size(); i++){
        if(responses_in_endeffector_with_transform_vec[i].msg_vec[0].pose.state == "null") continue;
        mm_robot_decision::VisualAppResponse avg_msg_in_base_i;
        responses_in_endeffector_with_transform_vec[i].averageInBase(avg_msg_in_base_i);
        // check whether T_obji_to_objj is valid by comparing to the database
        Eigen::Matrix4d T_obj_i_to_cabinet_in_database;
        visual_cabinet_ptr->visual_objs_in_cabinet_map.at(avg_msg_in_base_i.object_unique_id_on_equipment).getTransformFromObjToPresentedFrame(T_obj_i_to_cabinet_in_database);
        
        
        Eigen::Matrix4d T_base_to_obj_i;
        msgToT_frame_to_obj(avg_msg_in_base_i, T_base_to_obj_i, "base");

        // compare T_obj_i_to_obj_j_in_database and T_obji_to_objj
        Eigen::Matrix4d T_base_to_obj_i_in_database = visual_cabinet_ptr->T_base_to_cabinet * T_obj_i_to_cabinet_in_database.inverse();
        
        // T_mul should be enough close to Identity
        if(enable_teaching){
            if(avg_msg_in_base_i.object_name == HANDCART_SWITCH_NAME){
                if(! isCenterPlaneSimilaire(T_base_to_obj_i, T_base_to_obj_i_in_database, 50, 10)) return false;
            }
            else{
                if(! isTransfromSimilaire(T_base_to_obj_i, T_base_to_obj_i_in_database, 50, 0.02)) return false;
            }

        }
        else{
            if(avg_msg_in_base_i.object_name == HANDCART_SWITCH_NAME){
                if(! isCenterPlaneSimilaire(T_base_to_obj_i, T_base_to_obj_i_in_database, 50, 10)) return false;
            }
            else{
                if(! isTransfromSimilaire(T_base_to_obj_i, T_base_to_obj_i_in_database, 20, 0.02)) return false;
            }
        }
        

    }
    return true;
}


void VisualSafeWrapper::processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE(){
    // retrive params from database


    assert(current_state == STATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE);
    try_num = 0;
    assert(registered_cmd.object_name == QRCODE_NAME && registered_cmd.object_unique_id_on_equipment == 0);
    ROS_INFO("initialization for the equipment(cabinet) id=%d , equipment type = %d", registered_cmd.equipment_id, registered_cmd.equipment_type);
    visual_cabinet_ptr = std::make_shared<VisualCabinet>(registered_cmd.equipment_id, registered_cmd.equipment_type);
    qr_codes_responses_msg_in_endeffector_with_transform_vec.clear();
    visual_cabinet_ptr->retrieveCabinetParams();

    current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_FIRST_QRCODE;

    #ifdef DEBUG
        visual_cabinet_ptr->T_base_to_cabinet<<  0.0306837, -0.010662, -0.999472 , -980.562,
 0.999459, 0.0121845, 0.0305533,  -115.458,
0.0118523, -0.999869, 0.0110301 ,   744.26,
        0 ,        0 ,        0  ,       1;
        visual_cabinet_ptr->isInitialized();
        ROS_WARN("You fill the T_base_to_cabinet with the hack method!");
        current_state = STATE_WAIT_FOR_OTHER_CMD;
        registered_cmd.object_name.clear();
    #endif

    return;
    
}
void VisualSafeWrapper::processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_QRCODE(){
    // retrive params from database
    assert(current_state == STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_FIRST_QRCODE || current_state == STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_NEXT_QRCODE);
    try_num = 0;
    assert(registered_cmd.object_name == QRCODE_NAME);
    assert(visual_cabinet_ptr != nullptr);
    assert(visual_cabinet_ptr->cabinet_id == registered_cmd.equipment_id && visual_cabinet_ptr->cabinet_type_id == registered_cmd.equipment_type);
    ROS_INFO("Detect for the qr code (equipment(cabinet) id=%d , equipment type = %d)", registered_cmd.equipment_id, registered_cmd.equipment_type);
    
    try_num ++;

    mm_robot_decision::VisualAppResponse response_in_endeffector;
    bool has_result;
    
    mm_robot_decision::VisualAppResponse avg_responses_msg_in_base;
    std::vector<mm_visual_postion::AppInnerRequest> request_cmd_vec(1);
    std::vector<AppResponsesWithTransform> qr_code_responses_msg_in_endeffector_with_transform;
    
    visual_cabinet_ptr->getFundamentalInnerRequest(registered_cmd.object_unique_id_on_equipment, request_cmd_vec[0]);

    has_result = getResponseForApps(request_cmd_vec, qr_code_responses_msg_in_endeffector_with_transform, false, false, true , true, false);
    
    if(!abort_execution){
        if(has_result){

            Eigen::Matrix4d T_endeffector_to_base;
            if(!getTransform(T_endeffector_to_base, ros::Time(0))){
                ROS_ERROR("cannot get the transformation from endeffector to base");
                has_result = false;
            }
            qr_code_responses_msg_in_endeffector_with_transform[0].averageInBase(avg_responses_msg_in_base);
            std::cout<<"avg_responses_msg_in_base: \n"<<avg_responses_msg_in_base<<std::endl;
            convertMsgInBaseToMsgInEndeffector(avg_responses_msg_in_base, T_endeffector_to_base, response_in_endeffector);
            // converte result presented in base to result presented in endeffector
            qr_codes_responses_msg_in_endeffector_with_transform_vec.push_back(qr_code_responses_msg_in_endeffector_with_transform[0]);

            response_in_endeffector.object_name = registered_cmd.object_name;
            response_in_endeffector.success = true;
            addOffsetToResultInEndeffector(response_in_endeffector);
            
            
            if(registered_cmd.object_name == QRCODE_NAME && visual_cabinet_ptr->hasUniqueIdInType(registered_cmd.object_unique_id_on_equipment +1, QRCODE_NAME)){ 
                specific_cmd_wait_for.object_name = QRCODE_NAME;
                specific_cmd_wait_for.equipment_id = registered_cmd.equipment_id;
                specific_cmd_wait_for.equipment_type = registered_cmd.equipment_type;
                specific_cmd_wait_for.object_unique_id_on_equipment = registered_cmd.object_unique_id_on_equipment +1 ;
                current_state = STATE_WAIT_FOR_SPECIFIC_CMD;
                ROS_INFO("the cmd [%s] has been successfully excuted. wait for cmd [%s] with object_unique_id_on_equipment [%d]", registered_cmd.object_name.c_str(), QRCODE_NAME, registered_cmd.object_unique_id_on_equipment +1);
                publishResult(response_in_endeffector);
                registered_cmd.object_name.clear();
            }
            else if(registered_cmd.object_name == QRCODE_NAME && registered_cmd.object_unique_id_on_equipment +1 == (int) visual_cabinet_ptr->id_type_map.at(QRCODE_NAME).size()){
                current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_COMPUTE;
                last_qr_response_in_endeffector = response_in_endeffector;
            }
            else{
                assert(false && "invalid case");
            }

        }
        else{
            current_state = STATE_INNER_FAILURE;
        }
    }
    return;
}

void VisualSafeWrapper::processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_COMPUTE(){
    assert(current_state == STATE_INITIALIZATION_OF_VISUAL_CABINET_COMPUTE);
    try_num++;
    // check related pose constraint
    //if(!checkRelatedPoseWithDataBase(qr_codes_responses_msg_in_endeffector_with_transform_vec)){
    //    ROS_WARN("Initialization of visual cabinet failed, two qr codes do not satisfy the relative pose constraint");
    //    current_state = STATE_INNER_FAILURE;
    //    return;
    //}

    // check whether the base moved
    if(checkBaseMovedFromInitalPose()){
        ROS_WARN("Initialization of visual cabinet failed, Base has been moved during initialization!");
        current_state = STATE_ABORT_TASK;
        return ;
    } 

    // solve the transform from arm base to (equipment) cabinet
    mm_robot_decision::VisualAppResponse avg_qr_code_0_msg_in_base, avg_qr_code_1_msg_in_base;
    assert(qr_codes_responses_msg_in_endeffector_with_transform_vec.size() == 2);
    assert(qr_codes_responses_msg_in_endeffector_with_transform_vec[0].object_unique_id_on_equipment == 0);
    assert(qr_codes_responses_msg_in_endeffector_with_transform_vec[1].object_unique_id_on_equipment == 1);
    
    qr_codes_responses_msg_in_endeffector_with_transform_vec[0].averageInBase(avg_qr_code_0_msg_in_base);
    qr_codes_responses_msg_in_endeffector_with_transform_vec[1].averageInBase(avg_qr_code_1_msg_in_base);

    std::vector<VisualObject> visual_objs_in_base(2);
    visual_objs_in_base[0].fromVisualAppResponseInBase(avg_qr_code_0_msg_in_base);
    visual_objs_in_base[1].fromVisualAppResponseInBase(avg_qr_code_1_msg_in_base);
    
    visual_cabinet_ptr->updateTransformBaseToCabinetByCorners(qr_codes_responses_msg_in_endeffector_with_transform_vec);
    ROS_INFO("visual cabinet (id: %d, type: %d) is successfully initialized", visual_cabinet_ptr->cabinet_id, visual_cabinet_ptr->cabinet_type_id);
    registered_cmd.object_name.clear();
    std::cout<<"T_base_to_cabinet: \n"<< visual_cabinet_ptr->T_base_to_cabinet<<std::endl;
    current_state = STATE_WAIT_FOR_OTHER_CMD;
    publishResult(last_qr_response_in_endeffector);
    return ;
}
bool VisualSafeWrapper::adjustArmToCaptureObject(const int& object_unique_id_on_equipment, bool add_random_move){
    // you should move the arm close enough to the object's capture position.
    Eigen::Matrix4d T_endeffector_to_base, T_endeffector_to_obj;
    
    if(!getTransform(T_endeffector_to_base, ros::Time(0))){
        ROS_ERROR("cannot get tf from endeffector to base");
        return false;
    }
    visual_cabinet_ptr->getTransformFromEndeffectorToObj(object_unique_id_on_equipment, T_endeffector_to_base, T_endeffector_to_obj);
    
    // move the endeffector to the center (only in x,y) of the object, keep z-axis distance equals z_distance_mm
    
    Eigen::Matrix4d T_obj_to_best_capture = Eigen::Matrix4d::Identity();
    T_obj_to_best_capture.block<3,1>(0,3) = (*visual_cabinet_ptr)[object_unique_id_on_equipment].translate_from_obj_to_best_capture_point;
    Eigen::Matrix4d T_endeffector_to_best_capture = T_endeffector_to_obj * T_obj_to_best_capture;

    Eigen::Matrix3d rot_shot = T_endeffector_to_best_capture.block<3,3>(0,0);// * (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix());
    Eigen::Quaterniond q_shot(rot_shot);
    ROS_INFO("adjust the capture position for %s [unique_id_on_equipment = %d]", (*visual_cabinet_ptr)[object_unique_id_on_equipment].object_name.c_str(), object_unique_id_on_equipment);
    ROS_INFO("wait for the arm to arrive...");
    
    double random_x_move= 0;
    double random_y_move= 0;
    double random_z_move= 0;
    if(add_random_move)
        generateRandomMoveCmd(random_x_move, random_y_move, random_z_move);

    if(!(moveArmWithTimeoutConstraintInToolSpace(T_endeffector_to_best_capture(0,3) + random_x_move, T_endeffector_to_best_capture(1,3) + random_y_move, 
                            T_endeffector_to_best_capture(2,3) + random_z_move, q_shot.x(), q_shot.y(), q_shot.z(), q_shot.w(), 30.0)))// wait for 30 sec
    {
        return false;
    }
    ROS_INFO("arm arrived.");
    return true;
}
bool VisualSafeWrapper::moveArmWithTimeoutConstraintInToolSpace(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double seconds){
    moveArmByToolSpacePlanningQuaternion(x, y, 
                            z, q_x, q_y, q_z, q_w);
    if(!arm_commander.waitToArrive(seconds, abort_execution)) // wait for "seconds" sec
    {
        ROS_ERROR("arm moving is time out (%f secs)... ", seconds);
        return false;
    }
    return true;
}

bool VisualSafeWrapper::moveArmWorkSpacePlanningWithTimeoutConstraintInToolSpace(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double seconds){
    arm_commander.moveByWorkSpacePlanning(x, y, 
                            z, q_x, q_y, q_z, q_w);
    if(!arm_commander.waitToArrive(seconds, abort_execution)) // wait for "seconds" sec
    {
        ROS_ERROR("arm moving is time out (%f secs)... ", seconds);
        return false;
    }
    return true;
}


bool VisualSafeWrapper::getROI(const int& object_unique_id_on_equipment, Eigen::Matrix4d& T_endeffector_to_base, cv::Rect& left_roi, cv::Rect& right_roi, bool larger_roi){
    
    bool flag = visual_cabinet_ptr->getROI(object_unique_id_on_equipment, T_endeffector_to_base, left_roi, right_roi, larger_roi);

    return flag;
}




bool VisualSafeWrapper::oneTryForApps(std::vector<mm_visual_postion::AppInnerRequest>& request_cmd_vec, 
                    std::vector<AppResponsesWithTransform>& responses_msg_in_endeffector_with_transform_vec, bool use_roi){
    // In a static position, send request cmd to several apps to take one try.
    
    
    for(unsigned int i=0; i<request_cmd_vec.size(); i++){
        mm_robot_decision::VisualAppResponse response_msg;   
        mm_visual_postion::AppInnerRequest& request_i = request_cmd_vec[i]; 
        response_msg.frame_id = ENDEFFECTOR_FRAME_NAME;
        response_msg.object_name = request_i.object_name;
        response_msg.object_unique_id_on_equipment = request_i.object_unique_id_on_equipment;
        Eigen::Matrix4d T_endeffector_to_base;
        // save current position of arm
        getTransform(T_endeffector_to_base, ros::Time(0));

        // send command to apps
        cv::Rect left_roi, right_roi; 
        if(use_roi)
        {
            // get ROI of the point meters/lights in left/right images
              
            if(!getROI(request_i.object_unique_id_on_equipment, T_endeffector_to_base, left_roi, right_roi, enable_teaching)) return false;
            //cv::Mat left_image, right_image;
            //grabImages(left_image, right_image, ros::Time(0));
            //visual_cabinet_ptr->visualizeObject(request_i.object_name, left_image, right_image, T_endeffector_to_base);
            //cv::imshow("object_roi", left_image);
            //cv::waitKey(20);
        }
        else{
            cv::Rect roi(0,0,model_ptr->left_camera.image_size.width,model_ptr->left_camera.image_size.height);
            left_roi = roi;
            right_roi = roi;
        }
        ROICvToMsg(left_roi, request_i.left_roi);
        ROICvToMsg(right_roi, request_i.right_roi);   
        VisualAppNodeCallerPtr node_caller_ptr = node_caller_ptr_map.at(request_i.object_name);     
        node_caller_ptr->sendCmd(request_i);
        
        
        // wait to finish
        if(!node_caller_ptr->waitToFinish(10.0, abort_execution)) return false;
        if(visual_cabinet_ptr != nullptr){
            assert(node_caller_ptr->last_recv_msg.object_name == visual_cabinet_ptr->visual_objs_in_cabinet_map.at(node_caller_ptr->last_recv_msg.object_unique_id_on_equipment).object_name);
        }
        
        // check succeed
        if(!node_caller_ptr->last_recv_msg.success){
            ROS_WARN("detection for %s failed.", request_i.object_name.c_str());
            return false;
        }
        if(responses_msg_in_endeffector_with_transform_vec.size() <= i){
            responses_msg_in_endeffector_with_transform_vec.push_back(AppResponsesWithTransform(request_i.object_name, request_i.object_unique_id_on_equipment));
        }
        assert(responses_msg_in_endeffector_with_transform_vec[i].request_cmd == request_i.object_name);
        assert(node_caller_ptr->last_recv_msg.frame_id == ENDEFFECTOR_FRAME_NAME);
        std::cout<<"last_recv_msg : "<<node_caller_ptr->last_recv_msg.object_name<<std::endl;
        Eigen::Matrix4d T_base_to_endeffector = T_endeffector_to_base.inverse();
        responses_msg_in_endeffector_with_transform_vec[i].insert(node_caller_ptr->last_recv_msg, T_base_to_endeffector);
    }
    return true;
}
bool VisualSafeWrapper::getResponseForApps(std::vector<mm_visual_postion::AppInnerRequest>& request_cmd_vec,
                        std::vector<AppResponsesWithTransform>& responses_msg_in_endeffector_with_transform_vec,
                        bool move_to_best_capture_pose, bool use_roi,
                        bool check_pose_coherency, bool check_status_coherency, bool check_database_coherency){
    
    if(move_to_best_capture_pose){
        // adjust the arm into best capture pose according to the database
        if(!adjustArmToCaptureObject(request_cmd_vec[0].object_unique_id_on_equipment, try_num > 1)) return false;
    }
    bool one_try_ret;
    for(unsigned int i=0; i<3;i++){
        // try 3 times in the same place
        one_try_ret = oneTryForApps(request_cmd_vec, responses_msg_in_endeffector_with_transform_vec, use_roi);
        if(one_try_ret) break;
    }
    if(!one_try_ret){
        ROS_WARN("Failed on first try for visual appliactions");
        return false;
    }
    
    if(enable_teaching){
        // slightly move the arms to take second shot
        double x_move, y_move, z_move;
        generateRandomMoveCmd(x_move, y_move, z_move);
        if(!moveArmWithTimeoutConstraintInToolSpace(x_move, y_move, z_move, 0,0,0,1.0, 10.0)) return false;
        
        for(unsigned int i=0; i<3;i++){
            // try 3 times in the same place
            one_try_ret = oneTryForApps(request_cmd_vec, responses_msg_in_endeffector_with_transform_vec, use_roi);
            if(one_try_ret) break;
        }
        if(!one_try_ret){
            ROS_WARN("Failed on second try for visual appliactions");
            return false;
        }
            
        for(unsigned int i=0; i<responses_msg_in_endeffector_with_transform_vec.size(); i++){
            if(!responses_msg_in_endeffector_with_transform_vec[i].checkCoherent(check_pose_coherency, check_status_coherency)){
                ROS_WARN("Two positioning for %s are not coherent", responses_msg_in_endeffector_with_transform_vec[i].request_cmd.c_str());
                return false;
            }
        }
    }
    if(check_pose_coherency)
    {
        for(unsigned int i=0; i<responses_msg_in_endeffector_with_transform_vec.size(); i++){
            if(responses_msg_in_endeffector_with_transform_vec[i].request_cmd == HANDCART_SWITCH_NAME){
                // only check the pos and the plane, not check the rotation around z-axis
            if(!responses_msg_in_endeffector_with_transform_vec[i].checkGlobalConstraintOnlyCenterAndPlane(300, 300, 250, 1200, 5.0)){
                    ROS_WARN("Invalid pose! The pose solved for HANDCART_SWITCH_NAME is too far from the endeffector.");
                    return false;
            }
            }
            else{
                if(!responses_msg_in_endeffector_with_transform_vec[i].checkGlobalConstraint(300,300,200,1200, 0.3)){
                    ROS_WARN("Invalid pose! The pose solved for %s is too far from the endeffector.", 
                            responses_msg_in_endeffector_with_transform_vec[i].request_cmd.c_str());
                    return false;
                }
            }
        }
    }

    if(check_database_coherency && !checkGlobalPoseWithDataBase(responses_msg_in_endeffector_with_transform_vec)){
        ROS_WARN("pose is not coherent with the database");
        return false;
    }

    
    return true;
}
void VisualSafeWrapper::addOffsetToResultInEndeffector(mm_robot_decision::VisualAppResponse& res_in_endeffector){
    VisualObject& visual_obj = (*visual_cabinet_ptr)[res_in_endeffector.object_unique_id_on_equipment];
    Eigen::Matrix4d T_obj_to_real_obj = Eigen::Matrix4d::Identity();
    T_obj_to_real_obj.block<3,3>(0,0) = (Eigen::AngleAxisd(visual_obj.result_offset_euler_deg(0) / 180.0 * M_PI , Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(visual_obj.result_offset_euler_deg(1) / 180.0 * M_PI, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(visual_obj.result_offset_euler_deg(2) / 180.0 * M_PI, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    T_obj_to_real_obj.block<3,1>(0,3) = visual_obj.result_offset_translate_mm;

    Eigen::Matrix4d T_endeffector_to_obj = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q_origin(res_in_endeffector.pose.w, res_in_endeffector.pose.a, res_in_endeffector.pose.b, res_in_endeffector.pose.c);
    T_endeffector_to_obj.block<3,3>(0,0) = Eigen::Matrix3d(q_origin);
    T_endeffector_to_obj(0,3) = res_in_endeffector.pose.x * 1000.0;
    T_endeffector_to_obj(1,3) = res_in_endeffector.pose.y * 1000.0;
    T_endeffector_to_obj(2,3) = res_in_endeffector.pose.z * 1000.0;
    Eigen::Matrix4d T_endeffector_to_real_obj = T_endeffector_to_obj * T_obj_to_real_obj;
    Eigen::Quaterniond q_result(T_endeffector_to_real_obj.block<3,3>(0,0));

    res_in_endeffector.pose.x = T_endeffector_to_real_obj(0,3) / 1000.0;
    res_in_endeffector.pose.y = T_endeffector_to_real_obj(1,3) / 1000.0;
    res_in_endeffector.pose.z = T_endeffector_to_real_obj(2,3) / 1000.0;

    res_in_endeffector.pose.a = q_result.x();
    res_in_endeffector.pose.b = q_result.y();
    res_in_endeffector.pose.c = q_result.z();
    res_in_endeffector.pose.w = q_result.w();
}

void VisualSafeWrapper::processForSTATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD(){
    try_num = 0;
    assert(current_state == STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD);
    if(registered_cmd.object_name.empty()) return; // no cmd received, go on waiting...
    
    // received new cmd
    if(registered_cmd.object_name == QRCODE_NAME && registered_cmd.object_unique_id_on_equipment == 0){
        current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE;
        ROS_INFO("received the command for initialization of visual cabinet, start to detect the first qr code");
    }
    else{
        ROS_WARN("you should firstly call %s before sending other command, abort the task", QRCODE_NAME);
        current_state = STATE_ABORT_TASK;
    }
    return;
}
void VisualSafeWrapper::processForSTATE_WAIT_FOR_SPECIFIC_CMD()
{
    try_num = 0;
    assert(current_state == STATE_WAIT_FOR_SPECIFIC_CMD);
    if(registered_cmd.object_name.empty()) return; // no cmd received, go on waiting...


    // received new cmd
    if(registered_cmd.equipment_id != specific_cmd_wait_for.equipment_id){
        ROS_INFO("a new equipment id [%d] is input, abort the task", specific_cmd_wait_for.equipment_id);
        current_state = STATE_ABORT_TASK;
    }
    if(registered_cmd.equipment_type != specific_cmd_wait_for.equipment_type){
        ROS_INFO("a new equipment type [%d] is input, abort the task", specific_cmd_wait_for.equipment_type);
        current_state = STATE_ABORT_TASK;
    }

    if(registered_cmd.object_name == specific_cmd_wait_for.object_name){
        ROS_INFO("received the waited specific command [%s]", specific_cmd_wait_for.object_name.c_str());
        
        if(specific_cmd_wait_for.object_name == QRCODE_NAME){
            current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_NEXT_QRCODE;
        }
        else{
            ROS_ERROR("You have not indicated the behavior once received the command [%s]", specific_cmd_wait_for.object_name.c_str());
        }
    }
}
void VisualSafeWrapper::processForSTATE_WAIT_FOR_OTHER_CMD(){
    assert(current_state == STATE_WAIT_FOR_OTHER_CMD);
    try_num = 0;

    // visualization of visual cabinet
    if(enable_visualization_pub)
    {
        Eigen::Matrix4d T_endeffector_to_base;
        getTransform(T_endeffector_to_base, ros::Time(0));
        cv::Mat left_image, right_image;
        grabImages(left_image, right_image, ros::Time(0));
        visual_cabinet_ptr->visualizeCabinet(left_image, right_image, T_endeffector_to_base);
        cv::Mat concat_image;
        cv::hconcat(left_image, right_image, concat_image);
        visualizationPublishImage(concat_image);
    }

    if(visual_cabinet_ptr == nullptr || !(visual_cabinet_ptr->initialized)){
        ROS_WARN("You should firstly intiliazed the visual cabinet before sending other cmd");
        current_state = STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD;
        return;
    } 
    if(checkBaseMovedFromInitalPose()){
        ROS_WARN("base has been moved since last initialization of visual cabinet, need to re-initialize the visual cabinet!");
        current_state = STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD;
        return;
    } 
    
    if(registered_cmd.object_name.empty()) return; // no cmd received, go on waiting...

    // received new cmd
    if(registered_cmd.object_name == HANDCART_SWITCH_NAME || registered_cmd.object_name == KNIFE_SWITCH_NAME || 
        registered_cmd.object_name == LIGHTS_NAME || registered_cmd.object_name == POINT_METERS_NAME || 
        registered_cmd.object_name == DIGITAL_METERS_NAME || registered_cmd.object_name == REMOTE_SWITCH_NAME){
        current_state = STATE_PROCESS_OTHER_CMD;
        ROS_INFO("received the command for detection of %s", registered_cmd.object_name.c_str());
    }
    else if(registered_cmd.object_name == QRCODE_NAME){
        current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE;
        ROS_INFO("received the command for initialization of visual cabinet");
    }
    else{
        ROS_WARN("unknown command, abort the task %s", registered_cmd.object_name.c_str());
        current_state = STATE_ABORT_TASK;
    }
    return;
}


void VisualSafeWrapper::processForSTATE_PROCESS_OTHER_CMD(){
    assert(current_state == STATE_PROCESS_OTHER_CMD);        
    if(visual_cabinet_ptr == nullptr || !(visual_cabinet_ptr->isInitialized())){
        ROS_ERROR("You should firstly initlialized the visual cabinet");
        current_state = STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD;
        return;
    }
    
    try_num ++;

    if(visual_cabinet_ptr->cabinet_id != registered_cmd.equipment_id || 
        visual_cabinet_ptr->cabinet_type_id != registered_cmd.equipment_type){
        current_state = STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD;
        ROS_WARN("The request is not correspond with the same visual cabinet initialized (request cabinet_id: %d, cabinet_type: %d, intialized cabinet_id: %d, cabinet_type: %d",
                visual_cabinet_ptr->cabinet_id, visual_cabinet_ptr->cabinet_type_id, registered_cmd.equipment_id, registered_cmd.equipment_type);
    }
    mm_robot_decision::VisualAppResponse response_in_base, response_in_endeffector;
    bool has_result;


    std::vector<mm_visual_postion::AppInnerRequest> request_cmd_vec(1);
    mm_robot_decision::VisualAppResponse responses_msg_in_base;
    std::vector<AppResponsesWithTransform> responses_msg_in_endeffector_with_transform_vec;
    
    visual_cabinet_ptr->getFundamentalInnerRequest(registered_cmd.object_unique_id_on_equipment, request_cmd_vec[0]);
    if(registered_cmd.object_name == LIGHTS_NAME){
        has_result = getResponseForApps(request_cmd_vec, responses_msg_in_endeffector_with_transform_vec, true, true, false , true, false);
    }else{
        has_result = getResponseForApps(request_cmd_vec, responses_msg_in_endeffector_with_transform_vec, true, true, true , true, true);

    }
    if(has_result){
        assert(responses_msg_in_endeffector_with_transform_vec.size() == 1);
        responses_msg_in_endeffector_with_transform_vec[0].averageInBase(responses_msg_in_base);
        Eigen::Matrix4d T_endeffector_to_base;
        if(!getTransform(T_endeffector_to_base, ros::Time(0))){
            ROS_ERROR("cannot get the transformation from endeffector to base");
            has_result = false;
        }
        convertMsgInBaseToMsgInEndeffector(responses_msg_in_base, T_endeffector_to_base, response_in_endeffector);
        // convert result presented in base to result presented in endeffector
    }
    if(!abort_execution){
        if(has_result){
            response_in_endeffector.object_name = registered_cmd.object_name;
            response_in_endeffector.success = true;

            if(response_in_endeffector.object_name == HANDCART_SWITCH_NAME){
                Eigen::Matrix4d T_endeffector_to_obj, T_endeffector_to_plane, T_z_rot; double z_rot;
                msgToT_frame_to_obj(response_in_endeffector, T_endeffector_to_obj, ENDEFFECTOR_FRAME_NAME);
                seperateZ_rotation(T_endeffector_to_obj, T_endeffector_to_plane, T_z_rot, z_rot);
                transformToPose4WithQuaternion(T_endeffector_to_plane, response_in_endeffector.pose);
                // wait for endmotor to arrive
                double z_rot_deg = -z_rot * 180.0 / M_PI;
                ROS_INFO("handcart degree: %f",z_rot_deg);
                ROS_INFO("handcart degree: %d",abs(int(z_rot_deg)%90));
                if (abs(int(z_rot_deg)%90)<10)
                {
                    current_state = STATE_INNER_FAILURE;
                    return;
                }
                endmotor_commander.sendCmd(z_rot_deg);
                ROS_INFO("wait for endmotor to arrive");
                endmotor_commander.waitToFinish(10.0, false);
            }

            addOffsetToResultInEndeffector(response_in_endeffector);
            publishResult(response_in_endeffector);
            ROS_INFO("the cmd [%s] has been successfully excuted.", registered_cmd.object_name.c_str());
            registered_cmd.object_name.clear();
            current_state = STATE_WAIT_FOR_OTHER_CMD;
        }
        else{
            current_state = STATE_INNER_FAILURE;
        }
    }
    return;
}
void VisualSafeWrapper::generateRandomMoveCmd(double&x, double& y, double &z, bool normalized){
    std::uniform_real_distribution<> dis_x(-1.0, 1.0);
    std::uniform_real_distribution<> dis_y(-1.0, 1.0);
    std::uniform_real_distribution<> dis_z(-1.0, 1.0);
    x = dis_x(gen);
    y = dis_y(gen);
    z = dis_z(gen);
    // normalize
    double norm;
    if(normalized)
        norm = std::sqrt(x*x + y * y + z*z);
    else norm = 1.0;

    const double move_distance = 15;//mm
    x = move_distance * x/norm;
    y = move_distance * y/norm;
    z = move_distance * z/norm;
}
void VisualSafeWrapper::processForSTATE_INNER_FAILURE(){
    assert(current_state == STATE_INNER_FAILURE);
    if(try_num > 3){
        ROS_WARN("Have failed for 3 times, abort the task[%s]", registered_cmd.object_name.c_str());
        current_state = STATE_ABORT_TASK;
    }
    else{
        ROS_WARN("retry command: %s", registered_cmd.object_name.c_str());
        ROS_WARN("slightly move the arm");
        double x_move, y_move, z_move;
        generateRandomMoveCmd(x_move, y_move, z_move);
        if(!moveArmWithTimeoutConstraintInToolSpace(x_move, y_move, z_move,0,0,0,1.0, 20.0)){
            ROS_ERROR("Arm timeout..., retry...");
            try_num ++;
            return;
        }
        if(!abort_execution){
            if(registered_cmd.object_name == QRCODE_NAME){
                
                current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_FIRST_QRCODE;
            }
            else if(registered_cmd.object_name == QRCODE_NAME){
                
                current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_NEXT_QRCODE;
            }
            else{
                current_state = STATE_PROCESS_OTHER_CMD;
            }
        }
    }
}
void VisualSafeWrapper::processForSTATE_ABORT_TASK(){
    assert(current_state == STATE_ABORT_TASK);
    // abort the task

    mm_robot_decision::VisualAppResponse response;
    response.object_name = registered_cmd.object_name;
    response.success = false;

    publishResult(response);
    ROS_WARN("Abort the task [%s]", registered_cmd.object_name.c_str());
    ROS_WARN("Please try to re-initialize the visual cabinet to retry.");
    
    visual_cabinet_ptr = nullptr;
    registered_cmd.object_name.clear();
    current_state = STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD;
}
void VisualSafeWrapper::run(){
    
    // start state:
    current_state = STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD;
    #ifdef DEBUG
    current_state = STATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE;
    registered_cmd.object_name = QRCODE_NAME;
    registered_cmd.object_unique_id_on_equipment = 0;
    registered_cmd.equipment_id = 3;
    registered_cmd.equipment_type = 1;
    
    #endif  
    while(ros::ok()){
        spinOnce();
        ros::spinOnce();
        step();
        if(abort_execution){
            abort_execution = false; //fresh the abort flag
            try_num = 0;
        }
        
        
        ros::Duration(0.02).sleep();
    }
}

void VisualSafeWrapper::step(){
    switch(current_state){
        case STATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD: {
            processForSTATE_WAIT_FOR_INIT_VISUAL_CABINET_CMD();
            break;
        }
        case STATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE: {
            processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_LOAD_DATABASE();
            break;
        }
        case STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_FIRST_QRCODE: {
            processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_QRCODE();
            break;
        }
        case STATE_WAIT_FOR_SPECIFIC_CMD: {
            processForSTATE_WAIT_FOR_SPECIFIC_CMD();
            break;
        }
        
        case STATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_NEXT_QRCODE: {
            processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_DETECT_QRCODE();
            break;
        }
        case STATE_INITIALIZATION_OF_VISUAL_CABINET_COMPUTE: {
            processForSTATE_INITIALIZATION_OF_VISUAL_CABINET_COMPUTE();
            break;
        }
        case STATE_WAIT_FOR_OTHER_CMD: {
            processForSTATE_WAIT_FOR_OTHER_CMD();
            break;
        }
        case STATE_PROCESS_OTHER_CMD: {
            processForSTATE_PROCESS_OTHER_CMD();
            break;
        }
        case STATE_INNER_FAILURE: {
            processForSTATE_INNER_FAILURE();
            break;
        }
        case STATE_ABORT_TASK: {
            processForSTATE_ABORT_TASK();
            break;
        }
        default:
        {
            assert("Unknown state!");
        }
        
    }
}
