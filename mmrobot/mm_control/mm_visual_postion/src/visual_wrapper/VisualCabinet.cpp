#include "mm_visual_postion/visual_wrapper/VisualCabinet.h"



VisualCabinet::VisualCabinet(int cabinet_id_input, int cabinet_type_id_input){
    initialized = false;
    cabinet_id = cabinet_id_input;
    cabinet_type_id = cabinet_type_id_input;
    T_base_to_cabinet.setZero();
    model_ptr = std::make_shared<StereoCameraArmModel>();
    model_ptr->loadDefaultParams();
}
bool VisualCabinet::isInitialized(){
    initialized = (visual_objs_in_cabinet_map.size() > 0  && T_base_to_cabinet.norm() > 0);
    return initialized;
}
bool VisualCabinet::retrieveCabinetParams(){
    // get visual_objs from mysql
    // frame should be VOBJ_CABINET_FRAME
    MySQLInterface sql_interface("127.0.0.1", "mrobot", "123456", "Power_distribution_room");

    std::unique_ptr<sql::Statement> stmt(sql_interface.con->createStatement());
    std::stringstream ss_cmd;

    std::stringstream table_name;
    table_name << "equipment_type_"<<cabinet_type_id<<"_details";

    ss_cmd << "SELECT "<<table_name.str()<<".* , object_name_on_equipment.object_type_name, "
    << "equipment.name FROM equipment, "
    <<table_name.str()<<", object_name_on_equipment WHERE "<<
    table_name.str()<<".object_type_id = object_name_on_equipment.object_type_id AND " <<table_name.str()<<".equipment_id = "<<cabinet_id
    <<" AND equipment.id = "<< table_name.str()<<".equipment_id ;";
    std::cout<<"sql command :"<<ss_cmd.str()<<std::endl;

    std::unique_ptr< sql::ResultSet > res(stmt->executeQuery(ss_cmd.str()));
    parseSqlResult(res);
    assert(visual_objs_in_cabinet_map.size()>0 && "No result received from database");
    isInitialized();
    return true;
}
bool VisualCabinet::retrieveCabinetStandardParams(){
    // get visual_objs from mysql
    // frame should be VOBJ_CABINET_FRAME+
    MySQLInterface sql_interface("127.0.0.1", "mrobot", "123456", "Power_distribution_room");

    std::unique_ptr<sql::Statement> stmt(sql_interface.con->createStatement());
    std::stringstream ss_cmd;
    std::stringstream table_name;
    table_name << "equipment_type_"<<cabinet_type_id<<"_standard";
    ss_cmd << "SELECT "<<table_name.str()<<".* , object_name_on_equipment_type_"<<cabinet_type_id<<".object_type_name FROM "
    <<table_name.str()<<", object_name_on_equipment_type_"<<cabinet_type_id<<" WHERE "<<
    table_name.str()<<".object_type_id = object_name_on_equipment_type_"<<cabinet_type_id<<".object_type_id;";

    std::unique_ptr< sql::ResultSet > res(stmt->executeQuery(ss_cmd.str()));
    std::cout<<"sql command :"<<ss_cmd.str()<<std::endl;
    parseSqlResult(res);
    assert(visual_objs_in_cabinet_map.size()>0 && "No result received from database");
    isInitialized();
    return true;
}

void VisualCabinet::parseSqlResult(std::unique_ptr< sql::ResultSet >& res){
    while(res->next()){
        VisualObject obj;
        // parse cabinet name
        cabinet_name = res->getString("name");
        // parse type
        std::string object_type_name = res->getString("object_type_name");
        if(object_type_name == KNIFE_SWITCH_NAME||
            object_type_name == REMOTE_SWITCH_NAME|| 
            object_type_name == POINT_METERS_NAME || 
            object_type_name == DIGITAL_METERS_NAME ||
            object_type_name == LIGHTS_NAME || 
            object_type_name == HANDCART_SWITCH_NAME ||
            object_type_name == QRCODE_NAME)
           obj.object_name = object_type_name;
                
        else assert(false && (std::string("ERROR: unknow object type ")+ object_type_name+" in the result returned by sql,  make sure it is one of the " 
                                + KNIFE_SWITCH_NAME +" ,"+ 
                                REMOTE_SWITCH_NAME +" ,"+ 
                                HANDCART_SWITCH_NAME +" ,"+ 
                                DIGITAL_METERS_NAME +" ,"+
                                POINT_METERS_NAME + " or " +
                                LIGHTS_NAME + ", "+
                                QRCODE_NAME
                                ).c_str());

        obj.object_name_id = res->getInt("object_type_id");
        obj.object_unique_id_on_equipment = res->getInt("object_unique_id_on_equipment");
        // parse pose
        obj.pos(0) = res->getDouble("x");
        obj.pos(1) = res->getDouble("y");
        obj.pos(2) = res->getDouble("z");

        obj.q.x() = res->getDouble("q_x");
        obj.q.y() = res->getDouble("q_y");
        obj.q.z() = res->getDouble("q_z");
        obj.q.w() = res->getDouble("q_w");
        assert((fabs(obj.q.norm() - 1.0) < 1e-5) && "ERROR: the quaternion read sql is not normalized");
        obj.q.normalized();

        // parse width and height
        obj.width = res->getDouble("width");
        obj.height = res->getDouble("height");

        // parse dist_to_endeffector_for_capture
        obj.translate_from_obj_to_best_capture_point(0) = res->getDouble("x_from_obj_to_best_capture_point");
        obj.translate_from_obj_to_best_capture_point(1) = res->getDouble("y_from_obj_to_best_capture_point");
        obj.translate_from_obj_to_best_capture_point(2) = res->getDouble("z_from_obj_to_best_capture_point");

        // parse the  offset
        obj.result_offset_euler_deg(0) = res->getDouble("result_offset_euler_x_deg");
        obj.result_offset_euler_deg(1) = res->getDouble("result_offset_euler_y_deg");
        obj.result_offset_euler_deg(2) = res->getDouble("result_offset_euler_z_deg");
        obj.result_offset_translate_mm(0) = res->getDouble("result_offset_x_mm");
        obj.result_offset_translate_mm(1) = res->getDouble("result_offset_y_mm");
        obj.result_offset_translate_mm(2) = res->getDouble("result_offset_z_mm");

        obj.can_auto_refine = res->getBoolean("can_auto_refine");
        // parse the addition_string_info
        obj.additional_text_info = res->getString("additional_text_info");

        // parse the radius and trunk_square_size
        obj.radius = res->getDouble("radius");
        obj.trunk_square_size = res->getDouble("trunk_square_size");
        if(obj.object_name == HANDCART_SWITCH_NAME){
            assert(obj.radius > 0 && "you should enter a positive radius for handcart switch");
            assert(obj.trunk_square_size > 0&&"you should enter a positive trunk square size for handcart switch");
        }
        // all the data in sql should be presented in CABINET frame
        obj.presented_frame = VOBJ_CABINET_FRAME;
        assert((visual_objs_in_cabinet_map.size() == 0 ||  visual_objs_in_cabinet_map.find(obj.object_unique_id_on_equipment) == visual_objs_in_cabinet_map.end()) && "the object_unique_id_on_equipment is already existed");

        visual_objs_in_cabinet_map[obj.object_unique_id_on_equipment] = obj;

        if(id_type_map.find(obj.object_name) == id_type_map.end()){
            id_type_map[obj.object_name] = std::vector<int>();
        }
        id_type_map.at(obj.object_name).push_back(obj.object_unique_id_on_equipment);
    }
    if(id_type_map.size() == 0){
        ROS_ERROR("nothing is returned from the database");
        return;
    }
    // the QRCODE id should be the minimum id
    assert(id_type_map.at(QRCODE_NAME).size() >= 1);
    for(unsigned int i=0 ; i<id_type_map.at(QRCODE_NAME).size(); i++){   
        if(!(visual_objs_in_cabinet_map.at(i).object_name == QRCODE_NAME && visual_objs_in_cabinet_map.at(1).object_name == QRCODE_NAME)){
            std::string error =  "the " +std::to_string(i)+" th qr code should have object_unique_id_on_equipment == " + std::to_string(i);
            ROS_ERROR("%s", error.c_str());
        }
        assert(visual_objs_in_cabinet_map.at(0).object_name == QRCODE_NAME && visual_objs_in_cabinet_map.at(1).object_name == QRCODE_NAME);
    }

}
bool VisualCabinet::getCabinetGlobalPose(Eigen::Matrix4d& T_start_point_to_current_base, Eigen::Matrix4d& T_start_point_to_cabinet){
    // attention, this base is the base of arm, not the base_link
    MySQLInterface sql_interface("127.0.0.1", "mrobot", "123456", "Power_distribution_room");
    assert(initialized);
    std::vector<Eigen::Matrix4d> T_cabinet_to_qrcode_vec(2);
    visual_objs_in_cabinet_map.at(0).getTransformFromPresentedFrameToObj(T_cabinet_to_qrcode_vec[0]);
    visual_objs_in_cabinet_map.at(1).getTransformFromPresentedFrameToObj(T_cabinet_to_qrcode_vec[1]);
    Eigen::Matrix4d T_cabinet_to_cabinet_rough_center;
    averageTransform(T_cabinet_to_qrcode_vec, T_cabinet_to_cabinet_rough_center);
    T_start_point_to_cabinet = T_start_point_to_current_base * T_base_to_cabinet * T_cabinet_to_cabinet_rough_center;
    return true;
}
bool VisualCabinet::writeALineToSql(MySQLInterface& mysql_interface, VisualObject& obj){
    // UPDATE equipment_type_0_details 
    // SET x=obj.pos(0), y=obj.pos(1), z=obj.pos(2), q_x = obj.q.x(), q_y = obj.q.y(), q_z = obj.q.z(), q_w = obj.q.w(),
    //     width = obj.width, height = obj.height
    // WHERE equipment_id = cabinet_id AND object_type_id = object_type_id;
    std::unique_ptr<sql::Statement> stmt(mysql_interface.con->createStatement());
    std::stringstream table_name;
    table_name<< "equipment_type_"<<cabinet_type_id<<"_details";
    std::stringstream ss_cmd;
    ss_cmd << "UPDATE "<<table_name.str()
        << " SET x="<<obj.pos(0)<<", y="<<obj.pos(1)<<", z="<<obj.pos(2)
        <<", q_x="<<obj.q.x()<<", q_y="<<obj.q.y()<<", q_z="<<obj.q.z()<<", q_w="<<obj.q.w()
        <<", width="<<obj.width <<", height="<<obj.height;
    if(obj.object_name == HANDCART_SWITCH_NAME){
        ss_cmd << ", radius="<<obj.radius <<", trunk_square_size="<<obj.trunk_square_size;
    }
       
     ss_cmd   <<", result_offset_x_mm="<<obj.result_offset_translate_mm(0)
        <<", result_offset_y_mm="<<obj.result_offset_translate_mm(1)
        <<", result_offset_z_mm="<<obj.result_offset_translate_mm(2)
        <<", result_offset_euler_x_deg="<<obj.result_offset_euler_deg(0)
        <<", result_offset_euler_y_deg="<<obj.result_offset_euler_deg(1)
        <<", result_offset_euler_z_deg="<<obj.result_offset_euler_deg(2)

         << " WHERE equipment_id = "<<cabinet_id<<" AND object_type_id = "<<obj.object_name_id<<" AND object_unique_id_on_equipment = " <<obj.object_unique_id_on_equipment<<";";
    /*
    <<", x_from_obj_to_best_capture_point="<<obj.translate_from_obj_to_best_capture_point(0)
        <<", y_from_obj_to_best_capture_point="<<obj.translate_from_obj_to_best_capture_point(1)
        <<", z_from_obj_to_best_capture_point="<<obj.translate_from_obj_to_best_capture_point(2)
    */
    stmt->executeUpdate(ss_cmd.str());
    return true;
}

bool VisualCabinet::saveCabinetParams(){ //this will be only used at teaching period.
    // save to sql
    MySQLInterface sql_interface("127.0.0.1", "mrobot", "123456", "Power_distribution_room");
    for(auto it=visual_objs_in_cabinet_map.begin(); it != visual_objs_in_cabinet_map.end(); it++){
        writeALineToSql(sql_interface, it->second);
    }
    return true;
}
void VisualCabinet::updateVisualObjInCabinet(const std::vector<VisualObject>& visual_objs_in_base){
    //assuming that T_base_to_cabinet is correct, update visual_objs_in_cabinet by visual_objs_in_base.
    assert(visual_objs_in_base.size() > 0);
    assert(initialized);
    Eigen::Matrix4d T_origin_cabinet_to_qrcode_0;
    assert(visual_objs_in_cabinet_map.at(0).object_name == QRCODE_NAME && "the first qr code should have object_unique_id_on_equipment = 0");
    visual_objs_in_cabinet_map.at(0).getTransformFromPresentedFrameToObj(T_origin_cabinet_to_qrcode_0);
    for(auto it=visual_objs_in_base.begin(); it != visual_objs_in_base.end(); it++){
        Eigen::Matrix4d T_obj_to_base, T_obj_to_cabinet;
        it->getTransformFromObjToPresentedFrame(T_obj_to_base);
        T_obj_to_cabinet = T_obj_to_base * T_base_to_cabinet;
        Eigen::Matrix4d T_obj_to_cabinet_origin;
        VisualObject& obj_in_db = visual_objs_in_cabinet_map.at(it->object_unique_id_on_equipment);
        obj_in_db.getTransformFromObjToPresentedFrame(T_obj_to_cabinet_origin);

        // check if T_obj_to_cabinet is similaire enough to ori  gin one.
        assert(isTransfromSimilaire(T_obj_to_cabinet_origin, T_obj_to_cabinet, 200, 0.2));
        obj_in_db.setTransformFromObjToPresentedFrame(T_obj_to_cabinet);

        // check if width height are valid
        assert(it->width > 0 && it->height > 0);
        obj_in_db.width = it->width;
        obj_in_db.height = it->height;
        obj_in_db.additional_numerical_info.resize(2);
        if(it->additional_numerical_info.size() == 2 && it->object_name == HANDCART_SWITCH_NAME && obj_in_db.object_name == HANDCART_SWITCH_NAME){  
            // update radius, square_trunk_size 
            obj_in_db.additional_numerical_info[0] = it->additional_numerical_info[0];
            obj_in_db.additional_numerical_info[1] = it->additional_numerical_info[1];
        }
    }
    // always set qrcode_0 as the origin
    Eigen::Matrix4d T_new_cabinet_to_qrcode_0 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_new_cabinet_to_origin_cabinet = T_new_cabinet_to_qrcode_0 * T_origin_cabinet_to_qrcode_0.inverse();
    for(auto it=visual_objs_in_cabinet_map.begin(); it != visual_objs_in_cabinet_map.end(); it++){
        Eigen::Matrix4d T_origin_cabinet_to_obj, T_new_cabinet_to_obj;
        it->second.getTransformFromPresentedFrameToObj(T_origin_cabinet_to_obj);
        T_new_cabinet_to_obj = T_new_cabinet_to_origin_cabinet * T_origin_cabinet_to_obj;
        it->second.setTransformFromPresentedFrameToObj(T_new_cabinet_to_obj);
    }
    T_base_to_cabinet = T_base_to_cabinet.eval() * T_new_cabinet_to_origin_cabinet.inverse();
}

void VisualCabinet::visualizeCabinet(cv::Mat& left_image, cv::Mat& right_image, Eigen::Matrix4d& T_endeffector_to_base){
    assert(initialized);
    for(auto it=visual_objs_in_cabinet_map.begin(); it != visual_objs_in_cabinet_map.end(); it++){
        visualizeObject(it->second.object_unique_id_on_equipment, left_image, right_image, T_endeffector_to_base);
    }
}
void VisualCabinet::visualizeObject(const int& object_unique_id_on_equipment, cv::Mat& left_image, cv::Mat& right_image, Eigen::Matrix4d& T_endeffector_to_base){
    std::vector<cv::Point2d> corners_in_left_image, corners_in_right_image;
    if(getObjCornersInImage(object_unique_id_on_equipment, T_endeffector_to_base, corners_in_left_image, corners_in_right_image))
    {    
        cv::line(left_image, corners_in_left_image[0], corners_in_left_image[1], cv::Scalar(255, 0, 0), 2);
        cv::line(left_image, corners_in_left_image[1], corners_in_left_image[2], cv::Scalar(255, 0, 0), 2);
        cv::line(left_image, corners_in_left_image[2], corners_in_left_image[3], cv::Scalar(255, 0, 0), 2);
        cv::line(left_image, corners_in_left_image[3], corners_in_left_image[0], cv::Scalar(255, 0, 0), 2);
        std::string object_name = visual_objs_in_cabinet_map[object_unique_id_on_equipment].object_name;
        cv::putText(left_image, object_name + "(" + std::to_string(object_unique_id_on_equipment) + ")", corners_in_left_image[0], cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 255, 0), 3);

        cv::line(right_image, corners_in_right_image[0], corners_in_right_image[1], cv::Scalar(255, 0, 0), 2);
        cv::line(right_image, corners_in_right_image[1], corners_in_right_image[2], cv::Scalar(255, 0, 0), 2);
        cv::line(right_image, corners_in_right_image[2], corners_in_right_image[3], cv::Scalar(255, 0, 0), 2);
        cv::line(right_image, corners_in_right_image[3], corners_in_right_image[0], cv::Scalar(255, 0, 0), 2);
        cv::putText(right_image,object_name + "(" + std::to_string(object_unique_id_on_equipment) + ")", corners_in_right_image[0], cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 255, 0), 3);
    }
}
void VisualCabinet::visualizeObject(const VisualObject& visual_obj_in_base, cv::Mat& left_image, cv::Mat& right_image, Eigen::Matrix4d& T_endeffector_to_base){
    Eigen::Matrix4d T_obj_to_cabinet, T_obj_to_base;
    visual_obj_in_base.getTransformFromObjToPresentedFrame(T_obj_to_base);
    T_obj_to_cabinet = T_obj_to_base * T_base_to_cabinet;
    std::vector<cv::Point2d> corners_in_left_image, corners_in_right_image;
    if(getObjCornersInImage(T_obj_to_cabinet, T_endeffector_to_base, visual_obj_in_base.width, visual_obj_in_base.height, corners_in_left_image, corners_in_right_image))
    {
        cv::line(left_image, corners_in_left_image[0], corners_in_left_image[1], cv::Scalar(255, 0, 0), 2);
        cv::line(left_image, corners_in_left_image[1], corners_in_left_image[2], cv::Scalar(255, 0, 0), 2);
        cv::line(left_image, corners_in_left_image[2], corners_in_left_image[3], cv::Scalar(255, 0, 0), 2);
        cv::line(left_image, corners_in_left_image[3], corners_in_left_image[0], cv::Scalar(255, 0, 0), 2);
        
        cv::line(right_image, corners_in_right_image[0], corners_in_right_image[1], cv::Scalar(255, 0, 0), 2);
        cv::line(right_image, corners_in_right_image[1], corners_in_right_image[2], cv::Scalar(255, 0, 0), 2);
        cv::line(right_image, corners_in_right_image[2], corners_in_right_image[3], cv::Scalar(255, 0, 0), 2);
        cv::line(right_image, corners_in_right_image[3], corners_in_right_image[0], cv::Scalar(255, 0, 0), 2);
        cv::putText(right_image, visual_obj_in_base.object_name, corners_in_right_image[0], cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 255, 0), 3);
        cv::putText(left_image, visual_obj_in_base.object_name, corners_in_left_image[0], cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 255, 0), 3);
    }
}

void VisualCabinet::updateTransformBaseToCabinet(const std::vector<VisualObject>& visual_objs_in_base){
    assert(visual_objs_in_base.size() > 0);
    
    std::vector<Eigen::Vector3d>  pos_from_base_to_cabinet_vec(visual_objs_in_base.size());
    std::vector<Eigen::Quaterniond> q_from_base_to_cabinet_vec(visual_objs_in_base.size());
    int update_times = 0;
    for(auto it=visual_objs_in_base.begin(); it != visual_objs_in_base.end(); it++){
        
        Eigen::Matrix4d T_base_to_cabinet_local, T_base_to_obj, T_obj_to_cabinet;
        it->getTransformFromPresentedFrameToObj(T_base_to_obj);
        
        // find the same object presented in cabinet and in base
        visual_objs_in_cabinet_map.at(it->object_unique_id_on_equipment).getTransformFromObjToPresentedFrame(T_obj_to_cabinet);
        T_base_to_cabinet_local = T_base_to_obj * T_obj_to_cabinet;
        getTranslateAndQuat(T_base_to_cabinet_local, pos_from_base_to_cabinet_vec[update_times], q_from_base_to_cabinet_vec[update_times]);
        update_times ++;
    }

    // average the translation
    Eigen::Vector3d avg_pos_from_base_to_cabinet = Eigen::Vector3d::Zero();
    for(int i=0; i<update_times; i++){
        avg_pos_from_base_to_cabinet += pos_from_base_to_cabinet_vec[i];
    }
    avg_pos_from_base_to_cabinet /= update_times;

    // average the quaternions
    Eigen::Quaterniond avg_q_from_base_to_cabinet;
    averageQuaternion(q_from_base_to_cabinet_vec, avg_q_from_base_to_cabinet);

    // update T_base_to_cabinet
    getTransformMatrix(avg_pos_from_base_to_cabinet, avg_q_from_base_to_cabinet, T_base_to_cabinet);

    // check for initialization
    isInitialized();
}
inline void parseObjectsPosition(const std::vector<double>& additional_numerical_info, const Eigen::Matrix4d& T_base_to_endeffector, std::vector<Eigen::Vector3d>& points){
    assert( (additional_numerical_info.size() % 3) == 0);
    for(unsigned int i=0; i<additional_numerical_info.size(); i+=3){
        Eigen::Vector4d point_in_endeffector, point_in_base;
        point_in_endeffector << additional_numerical_info[i], additional_numerical_info[i+1], additional_numerical_info[i+2] , 1.0;
        point_in_base = T_base_to_endeffector * point_in_endeffector;
        points.push_back(point_in_base.head<3>());
    }
}

void VisualCabinet::updateTransformBaseToCabinetByCorners(const std::vector<AppResponsesWithTransform>& objs_in_enedeffector_with_transform_vec){
    assert(objs_in_enedeffector_with_transform_vec.size() > 0);
    
    std::vector<Eigen::Vector3d> real_points_in_base;
    std::vector<Eigen::Vector3d> database_points_in_cabinet;
    std::vector<Eigen::Vector4d> database_points_in_cabinet_homo;
    int update_times = 0;
    for(auto it=objs_in_enedeffector_with_transform_vec.begin(); it != objs_in_enedeffector_with_transform_vec.end(); it++){
        for (unsigned int j=0; j < it->T_base_to_endeffector_vec.size(); j++){
            parseObjectsPosition(it->msg_vec[j].additional_numerical_info, it->T_base_to_endeffector_vec[j], real_points_in_base);
            Eigen::Matrix4d T_obj_to_cabinet;
            // find the same object presented in cabinet and in base
            VisualObject& obj_in_database = visual_objs_in_cabinet_map.at(it->msg_vec[j].object_unique_id_on_equipment);
            obj_in_database.getTransformFromObjToPresentedFrame(T_obj_to_cabinet);
            std::vector<Eigen::Vector4d> current_obj_database_points_in_cabinet;
            getCorners3dFromRectInCabinet(T_obj_to_cabinet, obj_in_database.width, obj_in_database.height, current_obj_database_points_in_cabinet);
            database_points_in_cabinet_homo.insert(database_points_in_cabinet_homo.end(), current_obj_database_points_in_cabinet.begin(), current_obj_database_points_in_cabinet.end());

        }
        
        update_times ++;
    }

    // from eigen vector4d to vector3d
    database_points_in_cabinet.resize(database_points_in_cabinet_homo.size());
    for(unsigned int i=0; i< database_points_in_cabinet_homo.size(); i++){
        database_points_in_cabinet[i] = database_points_in_cabinet_homo[i].head<3>();
    }
    RigidTransformSolver transform_solver;
    transform_solver.setPointSets(real_points_in_base, database_points_in_cabinet);
    double scale_ratio;

    transform_solver.solveTransform(T_base_to_cabinet, scale_ratio);
    std::cout<<"scale_ratio : "<<scale_ratio<<std::endl;

    for(unsigned int i=0; i<real_points_in_base.size(); i++){
        std::cout<<real_points_in_base[i].transpose() << " "<< (T_base_to_cabinet*database_points_in_cabinet_homo[i]).transpose()<<std::endl;
    }
     
    // check for initialization
    isInitialized(); 
}

void VisualCabinet::updateStatus(int& object_unique_id_on_equipment, std::vector<signed char>& status){
    visual_objs_in_cabinet_map.at(object_unique_id_on_equipment).status = status;
}

bool VisualCabinet::getTransformFromObj1ToObj2(const int obj_unique_id_1, const int& obj_unique_id_2, Eigen::Matrix4d& T_obj1_to_obj2){

    assert(visual_objs_in_cabinet_map.at(obj_unique_id_1).presented_frame == visual_objs_in_cabinet_map.at(obj_unique_id_2).presented_frame);
    Eigen::Matrix4d T_frame_to_obj2, T_obj1_to_frame;
    visual_objs_in_cabinet_map.at(obj_unique_id_1).getTransformFromObjToPresentedFrame(T_obj1_to_frame);
    visual_objs_in_cabinet_map.at(obj_unique_id_2).getTransformFromPresentedFrameToObj(T_frame_to_obj2);
    T_obj1_to_obj2 = T_obj1_to_frame * T_frame_to_obj2;
    return true;
}

void VisualCabinet::convertVisualObjBaseToCabinet(const VisualObject& visual_obj_in_base, VisualObject& visual_obj_in_cabinet){
    assert(initialized && visual_obj_in_base.presented_frame == VOBJ_BASE_FRAME);
    visual_obj_in_cabinet = visual_obj_in_base;
    Eigen::Matrix4d T_obj_to_base, T_cabinet_to_obj;
    visual_obj_in_base.getTransformFromObjToPresentedFrame(T_obj_to_base);
    T_cabinet_to_obj = (T_obj_to_base * T_base_to_cabinet).inverse();

    getTranslateAndQuat(T_cabinet_to_obj, visual_obj_in_cabinet.pos, visual_obj_in_cabinet.q);
    visual_obj_in_cabinet.presented_frame = VOBJ_CABINET_FRAME;
}


void VisualCabinet::convertVisualObjCabinetToBase(const VisualObject& visual_obj_in_cabinet, VisualObject& visual_obj_in_base){
    assert(initialized && visual_obj_in_cabinet.presented_frame == VOBJ_CABINET_FRAME);
    visual_obj_in_base = visual_obj_in_cabinet;
    Eigen::Matrix4d T_obj_to_cabinet, T_base_to_obj;
    visual_obj_in_cabinet.getTransformFromObjToPresentedFrame(T_obj_to_cabinet);
    T_base_to_obj =  T_base_to_cabinet * T_obj_to_cabinet.inverse();

    getTranslateAndQuat(T_base_to_obj, visual_obj_in_base.pos, visual_obj_in_base.q);
    visual_obj_in_base.presented_frame = VOBJ_BASE_FRAME;

}
void VisualCabinet::getFundamentalInnerRequest(const int& object_unique_id_on_equipment, mm_visual_postion::AppInnerRequest& inner_request)
{
    // roi is not included
    VisualObject& obj = visual_objs_in_cabinet_map.at(object_unique_id_on_equipment);
    inner_request.object_name = obj.object_name;
    inner_request.frame_id = ENDEFFECTOR_FRAME_NAME;
    inner_request.object_height = obj.height;
    inner_request.object_width = obj.width;
    inner_request.object_unique_id_on_equipment = object_unique_id_on_equipment;
    
    Eigen::Matrix4d T_cabinet_to_obj, T_base_to_obj;
    visual_objs_in_cabinet_map.at(object_unique_id_on_equipment).getTransformFromPresentedFrameToObj(T_cabinet_to_obj);
    T_base_to_obj = T_base_to_cabinet * T_cabinet_to_obj;
    matrixToTransform(T_base_to_obj, inner_request.transform_base_to_obj);
    inner_request.additional_text_info = obj.additional_text_info;
    if(obj.object_name == HANDCART_SWITCH_NAME){
        inner_request.additional_numerical_info.push_back(obj.radius);
        inner_request.additional_numerical_info.push_back(obj.trunk_square_size);
    }
    else if(obj.object_name == LIGHTS_NAME){
        inner_request.additional_text_info = "double";
    }
}

void VisualCabinet::updateObjPoseByTransformFromBaseToObj(const int& object_unique_id_on_equipment, Eigen::Matrix4d& T_base_to_obj){
    assert(initialized);
    Eigen::Matrix4d T_cabinet_to_obj = T_base_to_cabinet.inverse() * T_base_to_obj;
    assert(visual_objs_in_cabinet_map.at(object_unique_id_on_equipment).presented_frame == VOBJ_CABINET_FRAME);
    visual_objs_in_cabinet_map.at(object_unique_id_on_equipment).setTransformFromPresentedFrameToObj(T_cabinet_to_obj);
}

bool VisualCabinet::getObjCornersInImage(const int& object_unique_id_on_equipment, const Eigen::Matrix4d& T_endeffector_to_base, 
                std::vector<cv::Point2d>& corners_in_left_image, std::vector<cv::Point2d>& corners_in_right_image){

    auto obj_iter = visual_objs_in_cabinet_map.find(object_unique_id_on_equipment);
    assert(obj_iter != visual_objs_in_cabinet_map.end() && "The input object_unique_id_on_equipment isn't contained in visual_objs_in_cabinet_map");

    // get the roi of this object in left and right images
    
    
    Eigen::Matrix4d T_obj_to_cabinet;
    obj_iter->second.getTransformFromObjToPresentedFrame(T_obj_to_cabinet);
    return getObjCornersInImage(T_obj_to_cabinet, T_endeffector_to_base, obj_iter->second.width, obj_iter->second.height, corners_in_left_image, corners_in_right_image);
}
bool VisualCabinet::getObjCornersInImage(const Eigen::Matrix4d& T_obj_to_cabinet, const Eigen::Matrix4d& T_endeffector_to_base, const double width, const double height, 
                std::vector<cv::Point2d>& corners_in_left_image, std::vector<cv::Point2d>& corners_in_right_image){

    if(!getCornersInImageFromRectInCabinet(T_endeffector_to_base, T_obj_to_cabinet, 
                                    width, height, corners_in_left_image, LEFT_CAMERA)) return false;
    if(!getCornersInImageFromRectInCabinet(T_endeffector_to_base, T_obj_to_cabinet, 
                                    width, height, corners_in_right_image, RIGHT_CAMERA)) return false;
    return true;
}
bool VisualCabinet::getROI(const int& object_unique_id_on_equipment, const Eigen::Matrix4d& T_endeffector_to_base, cv::Rect& left_roi, cv::Rect& right_roi, bool larger_roi){

    std::vector<cv::Point2d> corners_in_left_image, corners_in_right_image;
    getObjCornersInImage(object_unique_id_on_equipment, T_endeffector_to_base, corners_in_left_image, corners_in_right_image);
    if(!boundingRect(corners_in_left_image, left_roi)){
        ROS_WARN("left roi for object_unique_id_on_equipment[%d] is not (completely) in image", object_unique_id_on_equipment);
        return false;
    }
    if(!boundingRect(corners_in_right_image, right_roi)){
        ROS_WARN("right roi for object_unique_id_on_equipment[%d] is not (completely) in image", object_unique_id_on_equipment);
        return false;
    } 

    if(larger_roi){
        left_roi.x -= 30;
        left_roi.y -= 30;
        right_roi.x -= 30;
        right_roi.y -= 30;
        left_roi.width += 60;
        left_roi.height += 60;
        right_roi.width += 60;
        right_roi.height += 60;
    }
    if(!isRectCompletelyInImage(left_roi, model_ptr->left_camera.image_size)){
        ROS_WARN("left roi for object_unique_id_on_equipment[%d] is not (completely) in image", object_unique_id_on_equipment);
        return false;
    }
    if(!isRectCompletelyInImage(right_roi, model_ptr->right_camera.image_size)){
        ROS_WARN("right roi for object_unique_id_on_equipment[%d] is not (completely) in image", object_unique_id_on_equipment);
        return false;
    }
    return true;
}
bool VisualCabinet::boundingRect(std::vector<cv::Point2d>& corners, cv::Rect& rect){
    for(unsigned int i=0; i<corners.size(); i++){
        if(corners[i].x <0 || corners[i].y <0)
            return false;
    }
    // change Point2d to Point2f (boundingRect cannot be used on Point2d set)
    std::vector<cv::Point2f> corners_float;
    corners_float.insert(corners_float.end(), corners.begin(), corners.end());
    rect = cv::boundingRect(corners_float);
    return true;
}
bool VisualCabinet::getCorners3dFromRectInCabinet(const Eigen::Matrix4d& T_obj_to_cabinet, double width, double height, std::vector<Eigen::Vector4d>& corners3d_in_cabinet)
{
    corners3d_in_cabinet.resize(4);

    // in left_up, right_up, right_down, left_down order
    Eigen::Vector4d point_in_obj, point_in_cabinet;
    Eigen::Matrix4d T_cabinet_to_obj = T_obj_to_cabinet.inverse();
    //left_up
    point_in_obj << -width/2.0, -height/2.0, 0.0, 1.0;
    point_in_cabinet = T_cabinet_to_obj * point_in_obj;
    corners3d_in_cabinet[0] = point_in_cabinet;
    //right_up
    point_in_obj << width/2.0, -height/2.0, 0.0, 1.0;
    point_in_cabinet = T_cabinet_to_obj*  point_in_obj;
    corners3d_in_cabinet[1] = point_in_cabinet;

    //right_down
    point_in_obj << width/2.0, height/2.0, 0.0, 1.0;
    point_in_cabinet = T_cabinet_to_obj * point_in_obj;
    corners3d_in_cabinet[2] = point_in_cabinet;

    //left_down
    point_in_obj << -width/2.0, height/2.0, 0.0, 1.0;
    point_in_cabinet = T_cabinet_to_obj *  point_in_obj;
    corners3d_in_cabinet[3] = point_in_cabinet;
    return true;

}
bool VisualCabinet::getCornersInImageFromRectInCabinet(const Eigen::Matrix4d& T_endeffector_to_base, 
        const Eigen::Matrix4d& T_obj_to_cabinet, double width, double height, std::vector<cv::Point2d>& corners_in_image, int camera_id){
    corners_in_image.resize(4);

    Eigen::Matrix4d T_cabinet_to_obj = T_obj_to_cabinet.inverse();
    Eigen::Matrix4d T_camera_to_obj = model_ptr->endeffector_to_cam_transform.inverse() * T_endeffector_to_base * T_base_to_cabinet * T_cabinet_to_obj;
    if(T_camera_to_obj(2,3) < 50) return false; // object is too close to the camera or behind the camera

    std::vector<Eigen::Vector4d> corners3d_in_cabinet;
    std::vector<Eigen::Vector3d> points_in_image;
    getCorners3dFromRectInCabinet(T_obj_to_cabinet, width, height, corners3d_in_cabinet);

    getPointInImageFromPointInCabinet(T_endeffector_to_base, corners3d_in_cabinet, points_in_image, camera_id);
    
    for(unsigned int i=0; i<points_in_image.size(); i++){
        cv::Point2d cv_point_in_image;
        translateToCvPoint(points_in_image[i], corners_in_image[i]);
    }
    
    return true;
}

Eigen::Vector3d VisualCabinet::getPointInImageFromPointInCabinet(const Eigen::Matrix4d& T_endeffector_to_base, 
            const Eigen::Vector4d& point_in_cabinet, int camera_id){
    assert(fabs(point_in_cabinet(3) - 1) < 1e-8 && "the last element of point_in_cabinet should be 1.0");

    Eigen::Vector3d point_in_image;
    Eigen::Vector4d point_in_camera;
    point_in_camera = model_ptr->endeffector_to_cam_transform.inverse() * T_endeffector_to_base * T_base_to_cabinet * point_in_cabinet;
    
    if(camera_id == LEFT_CAMERA)
        point_in_image = model_ptr->left_camera.projection_mat * point_in_camera;
    else if (camera_id == RIGHT_CAMERA)
        point_in_image = model_ptr->right_camera.projection_mat * point_in_camera;
    else
        assert((camera_id == 0 || camera_id == 1) && "invalid camera_id");
    normalizeVector3d(point_in_image);
    return point_in_image;
}
void VisualCabinet::getPointInImageFromPointInCabinet(const Eigen::Matrix4d& T_endeffector_to_base, 
            const std::vector<Eigen::Vector4d>& points_in_cabinet, std::vector<Eigen::Vector3d>& points_in_image, int camera_id){

    Eigen::Matrix4d T_cam_to_cabinet = model_ptr->endeffector_to_cam_transform.inverse() * T_endeffector_to_base * T_base_to_cabinet;
    for(unsigned int i=0; i<points_in_cabinet.size(); i++){
        Eigen::Vector3d point_in_image;
        Eigen::Vector4d point_in_camera = T_cam_to_cabinet * points_in_cabinet[i];
        if(camera_id == LEFT_CAMERA)
            point_in_image = model_ptr->left_camera.projection_mat * point_in_camera;
        else if (camera_id == RIGHT_CAMERA)
            point_in_image = model_ptr->right_camera.projection_mat * point_in_camera;
        else
            assert((camera_id == 0 || camera_id == 1) && "invalid camera_id");
        normalizeVector3d(point_in_image);
        points_in_image.push_back(point_in_image);
    }

}
void VisualCabinet::getTransformFromEndeffectorToObj(const int& object_unique_id_on_equipment, const Eigen::Matrix4d& T_endeffector_to_base, Eigen::Matrix4d& T_endeffector_to_obj){
    assert(initialized);
    Eigen::Matrix4d T_cabinet_to_obj;
    visual_objs_in_cabinet_map.at(object_unique_id_on_equipment).getTransformFromPresentedFrameToObj(T_cabinet_to_obj);
    T_endeffector_to_obj = T_endeffector_to_base * T_base_to_cabinet * T_cabinet_to_obj;
}

bool VisualCabinet::operator == (const VisualCabinet another_cabinet) const{
    
    if(!(cabinet_id == another_cabinet.cabinet_id
    && cabinet_type_id == another_cabinet.cabinet_type_id) )
        return false;
    if(visual_objs_in_cabinet_map.size() != another_cabinet.visual_objs_in_cabinet_map.size())
        return false;
    auto another_iter = another_cabinet.visual_objs_in_cabinet_map.begin();
    for(auto iter=visual_objs_in_cabinet_map.begin(); iter != visual_objs_in_cabinet_map.end(); iter++){
        if(!(iter->second == another_iter->second))
            return false;
        another_iter ++;
    }
    if((T_base_to_cabinet - another_cabinet.T_base_to_cabinet).norm() > 1e-8) return false;
    if(initialized != another_cabinet.initialized) return false;
    return true;
}


VisualObject& VisualCabinet::operator [](const int& object_unique_id_on_equipment){
    return visual_objs_in_cabinet_map.at(object_unique_id_on_equipment);
}
bool VisualCabinet::hasUniqueIdInType(const int object_unique_id_on_equipment, const std::string& object_name)
{
    std::vector<int>& ids = id_type_map.at(object_name);
    for(unsigned int i=0; i < ids.size(); i ++){
        if(ids[i] == object_unique_id_on_equipment) return true;
    }
    return false;
}
