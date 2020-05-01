#ifndef __VISUAL_CABINET_H__
#define __VISUAL_CABINET_H__
#include <vector>
#include "mm_visual_postion/visual_wrapper/VisualObject.h"
#include "mm_visual_postion/utils/RigidTransfomSolver.h"
#include "mm_visual_postion/utils/StereoCameraArmModel.h"
#include <opencv2/opencv.hpp>
#include <sstream>
// mysql connector
#include "mm_visual_postion/utils/MySQLInterface.h"
#include "mm_visual_postion/AppInnerRequest.h"

class AppResponsesWithTransform{
    private:

        bool avg_msg_valid_flag;
        
    public:
        std::vector<mm_robot_decision::VisualAppResponse> msg_vec; // for different shot with same object.
        // the presented frame should be endeffector
        std::vector<Eigen::Matrix4d> T_base_to_endeffector_vec;
        std::string request_cmd;
        int object_unique_id_on_equipment;
        mm_robot_decision::VisualAppResponse avg_msg_in_base;
        AppResponsesWithTransform(std::string request_cmd_input, int object_unique_id_on_equipment_input){
            avg_msg_valid_flag = false;
            request_cmd = request_cmd_input;
            object_unique_id_on_equipment = object_unique_id_on_equipment_input;
        }
        void insert(mm_robot_decision::VisualAppResponse& msg, Eigen::Matrix4d& T_base_to_endeffector){
            assert(request_cmd == msg.object_name);
            assert(object_unique_id_on_equipment == msg.object_unique_id_on_equipment);
            msg_vec.push_back(msg);
            T_base_to_endeffector_vec.push_back(T_base_to_endeffector);
            avg_msg_valid_flag = false;
        }
        
        bool checkGlobalConstraint(const double abs_dist_x_max, const double abs_dist_y_max,const double dist_z_min,  const double dist_z_max, 
                                  const double abs_dist_q_epsilon){
            // obj should be close enough to the endeffector
            /*
            const double dist_x_max = 300; //mm
            const double dist_y_max = 300; //mm
            const double dist_z_min = 600; //mm
            const double dist_z_max = 600; //mm

            const double dist_a_max = 30; //deg
            const double dist_b_max = 30; //deg
            const double dist_c_max = 30; //deg
            */
            for(unsigned int i=0; i<msg_vec.size(); i++){
                if(msg_vec[i].pose.state == "null") continue;
                Eigen::Matrix4d T_endeffector_to_obj;
                msgToT_frame_to_obj(msg_vec[i], T_endeffector_to_obj, ENDEFFECTOR_FRAME_NAME);
                if(fabs(T_endeffector_to_obj(0,3)) > abs_dist_x_max ||  
                    fabs(T_endeffector_to_obj(1,3)) > abs_dist_y_max) 
                    return false;
                if(T_endeffector_to_obj(2,3) > dist_z_max || 
                    T_endeffector_to_obj(2,3) < dist_z_min) 
                    return false;
                Eigen::Quaterniond q(T_endeffector_to_obj.block<3,3>(0,0));
                Eigen::Quaterniond q_Id(1,0,0,0);
                if(! isQuaternionSimilar(q,q_Id, abs_dist_q_epsilon)) return false;

            }
           return true;
        }
        bool checkGlobalConstraintOnlyCenterAndPlane(const double abs_dist_x_max, const double abs_dist_y_max, const double dist_z_min, const double dist_z_max, const double abs_dist_deg_max){
            for(unsigned int i=0; i<msg_vec.size(); i++){
                Eigen::Matrix4d T_endeffector_to_obj;
                msgToT_frame_to_obj(msg_vec[i], T_endeffector_to_obj, ENDEFFECTOR_FRAME_NAME);
                if(fabs(T_endeffector_to_obj(0,3)) > abs_dist_x_max ||  
                    fabs(T_endeffector_to_obj(1,3)) > abs_dist_y_max) 
                    return false;
                if(T_endeffector_to_obj(2,3) > dist_z_max || 
                    T_endeffector_to_obj(2,3) < dist_z_min) 
                    return false;
                Eigen::Vector4d plane, center;
                getPlane(T_endeffector_to_obj, plane, center);
                double cos_angle = plane.head<3>().dot(Eigen::Vector3d::UnitZ()) / (plane.head<3>().norm());
                if(cos_angle < std::cos(M_PI - abs_dist_deg_max /180.0 * M_PI)) return false;
            }
            return true;
        }
        bool checkCoherent(bool check_pose, bool check_status){
            assert(msg_vec.size() >= 1);
            assert(msg_vec.size() == T_base_to_endeffector_vec.size());
            assert(msg_vec[0].success && "Only the success response can be inserted into this vector");
            
            
            if(msg_vec.size() == 1){
                ROS_WARN("Only stores one msg, no need to check coherency.");
                return true;
            }
            
            Eigen::Matrix4d T_base_to_obj_cap_0, T_endeffector_to_obj_cap_0;
            
            msgToT_frame_to_obj(msg_vec[0], T_endeffector_to_obj_cap_0, ENDEFFECTOR_FRAME_NAME);
            T_base_to_obj_cap_0 = T_base_to_endeffector_vec[0] * T_endeffector_to_obj_cap_0;

            for(unsigned int i=1; i<msg_vec.size(); i++){
                assert(msg_vec[i].frame_id == msg_vec[0].frame_id);
                assert(msg_vec[i].object_name == msg_vec[0].object_name);
                assert(msg_vec[i].success && "Only the success response can be inserted into this vector");

                if(check_pose){
                    // check for pose
                    Eigen::Matrix4d T_base_to_obj_cap_i, T_endeffector_to_obj_cap_i;
                    msgToT_frame_to_obj(msg_vec[0], T_endeffector_to_obj_cap_i, ENDEFFECTOR_FRAME_NAME);
                    T_base_to_obj_cap_i = T_base_to_endeffector_vec[0] * T_endeffector_to_obj_cap_i;
                    if(!isTransfromSimilaire(T_base_to_obj_cap_0, T_base_to_obj_cap_i, 3, 0.01)){
                        ROS_WARN("Poses are not coherent of %ld capture for request: %s", msg_vec.size(), msg_vec[0].object_name.c_str());
                        return false;

                    }
                }
                // check for status
                if(check_status){
                    assert(msg_vec[i].object_status.size() == msg_vec[0].object_status.size());
                    for(unsigned int j=0; j<msg_vec[0].object_status.size(); j++){
                        if(msg_vec[i].object_status[j] != msg_vec[0].object_status[j]){
                            ROS_WARN("Status are not coherent of %ld capture for request: %s", msg_vec.size(), msg_vec[0].object_name.c_str());
                            return false;
                        } 
                    }    
                }
            }

            return true;
        }
        void averageInBase(){
            std::vector<Eigen::Matrix4d> T_base_to_obj_vec;
            assert(msg_vec.size() == T_base_to_endeffector_vec.size());
            for(unsigned int i=0; i< msg_vec.size(); i++){
                Eigen::Matrix4d T_endeffector_to_obj;
                msgToT_frame_to_obj(msg_vec[i], T_endeffector_to_obj, ENDEFFECTOR_FRAME_NAME);
                T_base_to_obj_vec.push_back(T_base_to_endeffector_vec[i] * T_endeffector_to_obj);
            }
            Eigen::Matrix4d avg_T_base_to_obj;
            averageTransform(T_base_to_obj_vec, avg_T_base_to_obj);
            Eigen::Quaterniond q(avg_T_base_to_obj.block<3,3>(0,0));
            Eigen::Vector3d translate = avg_T_base_to_obj.block<3,1>(0,3);
            avg_msg_in_base = msg_vec[0]; // status information will be also copied
            avg_msg_in_base.frame_id = "base";
            avg_msg_in_base.pose.x = translate(0)/1000.0;
            avg_msg_in_base.pose.y = translate(1)/1000.0;
            avg_msg_in_base.pose.z = translate(2)/1000.0;
            avg_msg_in_base.pose.a = q.x();
            avg_msg_in_base.pose.b = q.y();
            avg_msg_in_base.pose.c = q.z();
            avg_msg_in_base.pose.w = q.w();    
            avg_msg_valid_flag = true;     
        }
        void averageInBase(mm_robot_decision::VisualAppResponse& avg_msg_in_base_output){
            if(avg_msg_valid_flag){
                avg_msg_in_base_output = avg_msg_in_base;
            }
            else{
                averageInBase();
                avg_msg_in_base_output = avg_msg_in_base;
            }   
        }
        bool isAverageValid(){
            return avg_msg_valid_flag;
        }
};


class VisualCabinet{
public:
    int cabinet_id;
    int cabinet_type_id;
    std::string cabinet_name;
    std::map< int, VisualObject > visual_objs_in_cabinet_map; // in cabinet coordinate
    std::map< std::string, std::vector<int> > id_type_map;
    Eigen::Matrix4d T_base_to_cabinet; // transform from base to cabinet
    bool initialized;
    std::shared_ptr<StereoCameraArmModel> model_ptr;
    VisualCabinet(int cabinet_id, int cabinet_type_id);
    bool isInitialized();
    bool retrieveCabinetParams(); //retrieve cabinet params (visual_objs) from sql database
    bool retrieveCabinetStandardParams(); // only used for unit test
    bool saveCabinetParams(); //this will be only used at teaching period.
    // set visual objs in base, and this function will automatically compute the T_base_to_cabinet in average.
    void updateTransformBaseToCabinet(const std::vector<VisualObject>& visual_objs_in_base);  // this is not as accurate as updateTransformBaseToCabinetByCorners
    
    // you should offer the four corners (in 3d) of object in its additional_numerical_info
    // in the order of (left up corner, right up corner, right down corner, left down corner)
    void updateTransformBaseToCabinetByCorners(const std::vector<AppResponsesWithTransform>& objs_in_enedeffector_with_transform_vec);

    // update status of object
    void updateStatus(int& object_unique_id_on_equipment, std::vector<signed char>& status);
    // convert visual obj in base coordinate to visual obj in cabinet coordinate.
    // Attention: make sure you have already called retrieveCabinetParams and updateTransformBaseToCabinet.
    void convertVisualObjBaseToCabinet(const VisualObject& visual_obj_in_base, VisualObject& visual_obj_in_cabinet);
    
    // convert visual obj in cabinet coordinate to visual obj in base coordinate.
    // Attention: make sure you have already called retrieveCabinetParams and updateTransformBaseToCabinet.
    void convertVisualObjCabinetToBase(const VisualObject& visual_obj_in_cabinet, VisualObject& visual_obj_in_base);

    // if the roi is not completely in the left and right images, then this function will return false
    bool getROI(const int& object_unique_id_on_equipment, const Eigen::Matrix4d& T_endeffector_to_base, cv::Rect& left_roi, cv::Rect& right_roi, bool larger_roi);
    
    void getFundamentalInnerRequest(const int& object_unique_id_on_equipment, mm_visual_postion::AppInnerRequest& inner_request);


    // Attention: the obj corners_in_image could be outside of the image  
    bool getObjCornersInImage(const int& object_unique_id_on_equipment, const Eigen::Matrix4d& T_endeffector_to_base, 
                std::vector<cv::Point2d>& corners_in_left_image, std::vector<cv::Point2d>& corners_in_right_image);
    
    void getTransformFromEndeffectorToObj(const int& object_unique_id_on_equipment, const Eigen::Matrix4d& T_endeffector_to_base, Eigen::Matrix4d& T_endeffector_to_obj);
    // get transfrom from obj1 to obj2 based on the data retrieved from dataset.
    bool getTransformFromObj1ToObj2(const int obj_unique_id_1, const int& obj_unique_id_2, Eigen::Matrix4d& T_obj1_to_obj2);
    
    // update visual object precised position from T_base_to_obj
    void updateObjPoseByTransformFromBaseToObj(const int& object_unique_id_on_equipment, Eigen::Matrix4d& T_base_to_obj);

    // helper functions
    Eigen::Vector3d inline getPointInImageFromPointInCabinet(const Eigen::Matrix4d& T_endeffector_to_base, 
            const Eigen::Vector4d& point_in_cabinet, int camera_id);
    void getPointInImageFromPointInCabinet(const Eigen::Matrix4d& T_endeffector_to_base, 
            const std::vector<Eigen::Vector4d>& points_in_cabinet, std::vector<Eigen::Vector3d>& points_in_image, int camera_id);

    bool getCorners3dFromRectInCabinet(const Eigen::Matrix4d& T_obj_to_cabinet, double width, double height, std::vector<Eigen::Vector4d>& corners3d_in_cabinet);

    // Attention: the corners_in_image could be outside of image  
    // corners_in_image are presented in left_up, right_up, right_down, left_down order
    bool getCornersInImageFromRectInCabinet(const Eigen::Matrix4d& T_endeffector_to_base, 
        const Eigen::Matrix4d& T_cabinet_to_obj, double width, double height, std::vector<cv::Point2d>& corners_in_image, int camera_id);

    void parseSqlResult(std::unique_ptr< sql::ResultSet >& res);
    bool writeALineToSql(MySQLInterface& mysql_interface, VisualObject& obj);
    bool getIndexOfObj(std::string obj_type_name, int& index);
    bool operator == (const VisualCabinet another_cabinet) const;
    VisualObject& operator [](const int& object_unique_id_on_equipment);
    bool boundingRect(std::vector<cv::Point2d>& corners, cv::Rect& rect);
    void updateVisualObjInCabinet(const std::vector<VisualObject>& visual_objs_in_base);
    void visualizeCabinet(cv::Mat& left_image, cv::Mat& right_image, Eigen::Matrix4d& T_endeffector_to_base);
    void visualizeObject(const int& object_unique_id_on_equipment, cv::Mat& left_image, cv::Mat& right_image, Eigen::Matrix4d& T_endeffector_to_base);
    void visualizeObject(const VisualObject& visual_obj_in_base, cv::Mat& left_image, cv::Mat& right_image, Eigen::Matrix4d& T_endeffector_to_base);
    bool getObjCornersInImage(const Eigen::Matrix4d& T_obj_to_cabinet, const Eigen::Matrix4d& T_endeffector_to_base, const double width, const double height, 
                std::vector<cv::Point2d>& corners_in_left_image, std::vector<cv::Point2d>& corners_in_right_image);
    bool hasUniqueIdInType(const int object_unique_id_on_equipment, const std::string& object_name);
    bool getCabinetGlobalPose(Eigen::Matrix4d& T_start_point_to_current_base, Eigen::Matrix4d& T_start_point_to_cabinet);

};

#endif