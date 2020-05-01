#ifndef __VISUAL_OBJECT_H__
#define __VISUAL_OBJECT_H__

#include <Eigen/Dense>
#include "mm_visual_postion/utils/utils.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/utils/ObjectDefinition.h"


#define VOBJ_BASE_FRAME 0
#define VOBJ_CABINET_FRAME 1
class VisualObject{
public:
    std::vector<signed char> status;
    int presented_frame; // presented frame, can be VOBJ_BASE_FRAME or VOBJ_CABINET_FRAME.

    Eigen::Vector3d pos;
    Eigen::Quaterniond q; // in presented_frame's coordinate

    
    double width; // in millimeter
    double height;// in millimeter

    Eigen::Vector3d translate_from_obj_to_best_capture_point; //in millimeter
    
    Eigen::Vector3d result_offset_translate_mm; // in millimeter (directly add to the result)
    Eigen::Vector3d result_offset_euler_deg; // in deg

    std::string object_name; // the Type of visual object, can be `knifeSwitch`, `remoteSwitch`,`handcartSwitch`,`pointMeter`,or `light`
    std::string additional_text_info; // stores the additional infomation, e.g. for qr code, it store the qr code's content
    std::vector<double> additional_numerical_info;
    double radius;  // used for handcart
    double trunk_square_size; // only used for handcart
    
    int object_name_id; 
    int object_unique_id_on_equipment; // to identify the objects (even they are the same type) on the same equipment
    bool can_auto_refine;
    VisualObject(){
        presented_frame = VOBJ_CABINET_FRAME;
        pos.setZero();
        q = Eigen::Quaterniond(1,0,0,0);
        width = 0.0;
        height = 0.0;
        translate_from_obj_to_best_capture_point.setZero();
        result_offset_translate_mm.setZero();
        result_offset_euler_deg.setZero();
        radius=0.0;
        trunk_square_size = 0.0;
        object_name_id = -1;
        object_unique_id_on_equipment = -1;
        can_auto_refine = false;
    }

    bool operator== (const VisualObject another_object) const{
        if(presented_frame != another_object.presented_frame) return false;
        if(status.size() != another_object.status.size()) return false;
        for(unsigned int i=0; i<status.size(); i++){
            if(status[i] != another_object.status[i]) return false;
        }
        if((pos - another_object.pos).norm() > 1e-8) return false;
        if(fabs(q.dot(another_object.q)) > 1-1e-8) return false;
        if(width != another_object.width || height != another_object.height) return false;
        if(object_name != another_object.object_name) return false;
        if(object_name_id != another_object.object_name_id) return false;
        if(additional_text_info  != another_object.additional_text_info) return false;
        
        return true;
    }
    void getTransformFromObjToPresentedFrame(Eigen::Matrix4d& T) const{
        // T is the transform matrix from object to presented_frame
        Eigen::Matrix4d T_frame_to_obj;
        getTransformMatrix(pos, q, T_frame_to_obj);
        T = T_frame_to_obj.inverse();
    }
    void getTransformFromPresentedFrameToObj(Eigen::Matrix4d& T) const{
        // T is the transform matrix from presented_frame to object
        getTransformMatrix(pos, q, T);
    }
    void setTransformFromPresentedFrameToObj(Eigen::Matrix4d& T){
        pos = T.block<3,1>(0,3);
        q = T.block<3,3>(0,0);
    }
    void setTransformFromObjToPresentedFrame(Eigen::Matrix4d& T){
        Eigen::Matrix4d T_obj_to_presented_frame = T.inverse();
        setTransformFromPresentedFrameToObj(T_obj_to_presented_frame);
    }
    void fromVisualAppResponseInBase(mm_robot_decision::VisualAppResponse& response){
        assert(response.frame_id == "base");
        object_name = response.object_name;
        presented_frame = VOBJ_BASE_FRAME;
        pos(0) = response.pose.x * 1000.0;
        pos(1) = response.pose.y *1000.0;
        pos(2) = response.pose.z * 1000.0;
        q.x() = response.pose.a;
        q.y() = response.pose.b;
        q.z() = response.pose.c;
        q.w() = response.pose.w;
        q.normalized();
        status = response.object_status;
        object_unique_id_on_equipment = response.object_unique_id_on_equipment;
        additional_numerical_info = response.additional_numerical_info;
        additional_text_info = response.additional_text_info;
        width = response.width;
        height = response.height;
    }
    void getPlaneInPresentedFrame(Eigen::Vector4d& plane){
        Eigen::Vector4d normal;
        normal << 0,0,1,0; //z-axis
        Eigen::Matrix4d T_presentedframe_to_obj;
        getTransformFromPresentedFrameToObj(T_presentedframe_to_obj);
        Eigen::Vector4d plane_normal = T_presentedframe_to_obj * normal;
        plane = plane_normal;

        std::cout<<"transform: "<<T_presentedframe_to_obj<<std::endl; 
        Eigen::Vector4d center = T_presentedframe_to_obj.block<4,1>(0,3);
        
        plane(3) = - plane_normal.transpose() * center;
        
        // normalize plane (assuming plane(3) == 1)
        plane /= plane(3);
        plane(3) = 1.0;
    }
};



#endif //__VISUAL_OBJECT_H__