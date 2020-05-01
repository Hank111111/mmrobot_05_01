#include "mm_visual_postion/utils/CoordinateTransformer.h"
void createTransformationMatrix(double trans_x, double trans_y, double trans_z, 
                                double rot_x, double rot_y, double rot_z, 
                                Eigen::Matrix4d& trans){
    Eigen::Matrix3d rot_mat = Eigen::AngleAxisd(rot_x, Eigen::Vector3d::UnitX()).toRotationMatrix()
                * Eigen::AngleAxisd(rot_y, Eigen::Vector3d::UnitY()).toRotationMatrix()
                * Eigen::AngleAxisd(rot_z, Eigen::Vector3d::UnitZ()).toRotationMatrix(); 
    trans.setIdentity();
    trans.block<3, 3>(0,0) = rot_mat;
    trans(0,3) = trans_x;
    trans(1,3) = trans_y;
    trans(2,3) = trans_z;
} 

void createTransformationMatrix(const std::vector<double>& xyzabc, Eigen::Matrix4d& trans){
    createTransformationMatrix(xyzabc[0], xyzabc[1], xyzabc[2], xyzabc[3], xyzabc[4], xyzabc[5], trans);
}
void createTransformationMatrix(double trans_x, double trans_y, double trans_z, 
                                double q_x, double q_y, double q_z, double q_w,
                                Eigen::Matrix4d& trans){
    Eigen::Quaterniond q(q_x, q_y, q_z, q_w);
    Eigen::Matrix3d rot_mat = q.toRotationMatrix();
    trans.setIdentity();
    trans.block<3, 3>(0,0) = rot_mat;
    trans(0,3) = trans_x;
    trans(1,3) = trans_y;
    trans(2,3) = trans_z;
} 

void CoordinateTransformer::setTransEndEffectorToBase(const geometry_msgs::TransformStamped& transform_endeffector_to_base){
       
        Eigen::Quaternion<double> q;
        Eigen::fromMsg(transform_endeffector_to_base.transform.rotation, q);
        T_endeffector_to_base.setIdentity();
        T_endeffector_to_base.block<3,3>(0,0) = q.normalized().toRotationMatrix();
        T_endeffector_to_base(0,3) = transform_endeffector_to_base.transform.translation.x * 1000.0; //millimiter
        T_endeffector_to_base(1,3) = transform_endeffector_to_base.transform.translation.y * 1000.0;
        T_endeffector_to_base(2,3) = transform_endeffector_to_base.transform.translation.z * 1000.0;
        T_endeffector_to_base(3,3) = 1.0;
        
        T_base_to_endeffector = T_endeffector_to_base.inverse();
        T_cam_to_base = T_cam_to_endeffector * T_endeffector_to_base;


}


CoordinateTransformer::CoordinateTransformer(Eigen::Matrix4d T_endeffector_to_cam, Eigen::Matrix<double, 3, 4> cam_projection_mat):
                                            T_endeffector_to_cam(T_endeffector_to_cam), cam_projection_mat(cam_projection_mat){
    T_cam_to_endeffector = T_endeffector_to_cam.inverse();
    initialized = true;

    setCamEndeffectorStaticBroadCaster();
}

void CoordinateTransformer::init(Eigen::Matrix4d T_endeffector_to_cam_, Eigen::Matrix<double, 3, 4> cam_projection_mat_){
    T_endeffector_to_cam = T_endeffector_to_cam_;
    cam_projection_mat = cam_projection_mat_;
    T_cam_to_endeffector = T_endeffector_to_cam.inverse();
    initialized = true;

    setCamEndeffectorStaticBroadCaster();
}


void CoordinateTransformer::setTransCamToObject(const Eigen::Matrix4d& trans_cam_to_object){
    T_cam_to_object = trans_cam_to_object;

    pubCamToObjStaticTransform();
}
void CoordinateTransformer::setTransCamToObject(const std::vector<double>& xyzabc_in_cam){
    Eigen::Vector3d translation(xyzabc_in_cam[0],xyzabc_in_cam[1],xyzabc_in_cam[2]);       
    Eigen::Matrix3d rot_mat = Eigen::AngleAxisd(xyzabc_in_cam[3], Eigen::Vector3d::UnitX()).toRotationMatrix()
                * Eigen::AngleAxisd(xyzabc_in_cam[4], Eigen::Vector3d::UnitY()).toRotationMatrix()
                * Eigen::AngleAxisd(xyzabc_in_cam[5], Eigen::Vector3d::UnitZ()).toRotationMatrix(); 
    T_cam_to_object.setIdentity();
    T_cam_to_object.block<3, 3>(0,0) = rot_mat;
    T_cam_to_object.block<3, 1>(0,3) = translation;

    //pubCamToObjStaticTransform();
}
void CoordinateTransformer::getPixelPosFromPosInBase(const double x_base, const double y_base, const double z_base, cv::Point2d& pixel_pos){
    Eigen::Vector4d pos_in_base_homogene;
    pos_in_base_homogene << x_base, y_base, z_base, 1.0;
    getPixelPosFromPosInBase(pos_in_base_homogene, pixel_pos);

}   
void CoordinateTransformer::getPixelPosFromPosInBase(const Eigen::Vector4d& pos_in_base_homogene, cv::Point2d& pixel_pos){
    Eigen::Vector3d pixel_pos_homogene;
    pixel_pos_homogene = cam_projection_mat * T_cam_to_base * pos_in_base_homogene;
    pixel_pos.x = pixel_pos_homogene[0];
    pixel_pos.y = pixel_pos_homogene[1];
}

void CoordinateTransformer::getPixelPosFromPosInObject(const Eigen::Vector4d& pos_in_object_homogene, cv::Point2d& pixel_pos){
    Eigen::Vector3d pixel_pos_homogene;
    Eigen::Vector4d pos_in_cam_homogene;
    pos_in_cam_homogene = T_cam_to_object * pos_in_object_homogene;
    pixel_pos_homogene = cam_projection_mat * pos_in_cam_homogene;
    //normalize
    pixel_pos_homogene(0) = pixel_pos_homogene(0)/ pixel_pos_homogene(2);
    pixel_pos_homogene(1) = pixel_pos_homogene(1)/ pixel_pos_homogene(2);
    pixel_pos_homogene(2) = 1.0;
     
    pixel_pos.x = pixel_pos_homogene[0];
    pixel_pos.y = pixel_pos_homogene[1];
}
void CoordinateTransformer::getPosInBaseFromPoseInObject(const Eigen::Vector4d& pos_in_object_homogene, Eigen::Vector4d& pos_in_base_homogene){
    pos_in_base_homogene = T_base_to_object * pos_in_object_homogene;
}
void CoordinateTransformer::getPosInBaseFromPosInCam(const Eigen::Vector4d& pos_in_cam_homogene, Eigen::Vector4d& pos_in_base_homogene){
    pos_in_base_homogene = T_cam_to_base.inverse() * pos_in_cam_homogene;
}
void CoordinateTransformer::getObjPoseInBaseFromObjPoseInCam(const std::vector<double>& xyzabc_in_cam, std::vector<double>& xyzabc_in_base){
    setTransCamToObject(xyzabc_in_cam);
    T_base_to_object = T_base_to_endeffector * T_endeffector_to_cam * T_cam_to_object;
    
    Eigen::Vector3d angles = T_base_to_object.block<3,3>(0,0).eulerAngles(0, 1, 2); 
    xyzabc_in_base.resize(6);
    xyzabc_in_base[0] = T_base_to_object(0,3);
    xyzabc_in_base[1] = T_base_to_object(1,3);
    xyzabc_in_base[2] = T_base_to_object(2,3);
    xyzabc_in_base[3] = angles[0];
    xyzabc_in_base[4] = angles[1];
    xyzabc_in_base[5] = angles[2];
}
void CoordinateTransformer::getObjPoseInCamFromObjPoseInBase(const std::vector<double>& xyzabc_in_base, std::vector<double>& xyzabc_in_cam){
    setTransBaseToObject(xyzabc_in_base);
    Eigen::Vector3d angles = T_cam_to_object.block<3,3>(0,0).eulerAngles(0, 1, 2); 
    xyzabc_in_cam.resize(6);
    xyzabc_in_cam[0] = T_cam_to_object(0,3);
    xyzabc_in_cam[1] = T_cam_to_object(1,3);
    xyzabc_in_cam[2] = T_cam_to_object(2,3);
    xyzabc_in_cam[3] = angles[0];
    xyzabc_in_cam[4] = angles[1];
    xyzabc_in_cam[5] = angles[2];
}

void CoordinateTransformer::setTransBaseToObject(const std::vector<double>& xyzabc_in_base){
    createTransformationMatrix(xyzabc_in_base[0], xyzabc_in_base[1], xyzabc_in_base[2], 
                                xyzabc_in_base[3], xyzabc_in_base[4], xyzabc_in_base[5], 
                                T_base_to_object);
    T_cam_to_object = T_cam_to_base * T_base_to_object;

    //pubCamToObjTransform();
}
void CoordinateTransformer::setTransBaseToObject(double trans_x, double trans_y, double trans_z, 
                                double rot_x, double rot_y, double rot_z){
    createTransformationMatrix(trans_x, trans_y, trans_z, 
                                rot_x, rot_y, rot_z, 
                                T_base_to_object);
    T_cam_to_object = T_cam_to_base * T_base_to_object;

    //pubCamToObjTransform();
}

void CoordinateTransformer::setTransBaseToObject(double trans_x, double trans_y, double trans_z, 
                                double q_x, double q_y, double q_z, double q_w){
    createTransformationMatrix(trans_x, trans_y, trans_z, 
                                q_x, q_y, q_z, q_w,
                                T_base_to_object);
    T_cam_to_object = T_cam_to_base * T_base_to_object;

    //pubCamToObjTransform();
}

void CoordinateTransformer::setCamEndeffectorStaticBroadCaster(){
    geometry_msgs::TransformStamped static_cam_endeffector_transformStamped;
    static_cam_endeffector_transformStamped.header.stamp = ros::Time::now();
    static_cam_endeffector_transformStamped.header.frame_id = "tool0_controller";
    static_cam_endeffector_transformStamped.child_frame_id = "left_camera";
    static_cam_endeffector_transformStamped.transform.translation.x = T_endeffector_to_cam(0,3) / 1000.0;
    static_cam_endeffector_transformStamped.transform.translation.y = T_endeffector_to_cam(1,3) / 1000.0;
    static_cam_endeffector_transformStamped.transform.translation.z = T_endeffector_to_cam(2,3) / 1000.0;
    Eigen::Quaternion<double> q(T_endeffector_to_cam.block<3,3>(0,0));
    static_cam_endeffector_transformStamped.transform.rotation.x = q.x();
    static_cam_endeffector_transformStamped.transform.rotation.y = q.y();
    static_cam_endeffector_transformStamped.transform.rotation.z = q.z();
    static_cam_endeffector_transformStamped.transform.rotation.w = q.w();
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static_broadcaster.sendTransform(static_cam_endeffector_transformStamped);
}




void CoordinateTransformer::pubCamToObjStaticTransform(){
    //debug usage
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "left_camera";
    transformStamped.child_frame_id = "init_object";
    transformStamped.transform.translation.x = T_cam_to_object(0,3) / 1000.0;
    transformStamped.transform.translation.y = T_cam_to_object(1,3) / 1000.0;
    transformStamped.transform.translation.z = T_cam_to_object(2,3) / 1000.0;

    Eigen::Quaternion<double> q(T_cam_to_object.block<3,3>(0,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    static_broadcaster.sendTransform(transformStamped);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base";
    transformStamped.child_frame_id = "endeffector";
    transformStamped.transform.translation.x = T_base_to_endeffector(0,3) / 1000.0;
    transformStamped.transform.translation.y = T_base_to_endeffector(1,3) / 1000.0;
    transformStamped.transform.translation.z = T_base_to_endeffector(2,3) / 1000.0;

    Eigen::Quaternion<double> q_base_to_endeffector(T_base_to_endeffector.block<3,3>(0,0));
    transformStamped.transform.rotation.x = q_base_to_endeffector.x();
    transformStamped.transform.rotation.y = q_base_to_endeffector.y();
    transformStamped.transform.rotation.z = q_base_to_endeffector.z();
    transformStamped.transform.rotation.w = q_base_to_endeffector.w();

    static_broadcaster.sendTransform(transformStamped);


    T_base_to_object = T_base_to_endeffector * T_endeffector_to_cam * T_cam_to_object;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base";
    transformStamped.child_frame_id = "init_object_from_base";
    transformStamped.transform.translation.x = T_base_to_object(0,3) / 1000.0;
    transformStamped.transform.translation.y = T_base_to_object(1,3) / 1000.0;
    transformStamped.transform.translation.z = T_base_to_object(2,3) / 1000.0;

    Eigen::Quaternion<double> q_base_to_obj(T_base_to_object.block<3,3>(0,0));
    transformStamped.transform.rotation.x = q_base_to_obj.x();
    transformStamped.transform.rotation.y = q_base_to_obj.y();
    transformStamped.transform.rotation.z = q_base_to_obj.z();
    transformStamped.transform.rotation.w = q_base_to_obj.w();

    static_broadcaster.sendTransform(transformStamped);

}

void CoordinateTransformer::pubCamToObjTransform(){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "left_camera";
    transformStamped.child_frame_id = "object";
    transformStamped.transform.translation.x = T_cam_to_object(0,3) / 1000.0;
    transformStamped.transform.translation.y = T_cam_to_object(1,3) / 1000.0;
    transformStamped.transform.translation.z = T_cam_to_object(2,3) / 1000.0;

    Eigen::Quaternion<double> q(T_cam_to_object.block<3,3>(0,0));
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}