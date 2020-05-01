#include "mm_visual_postion/visual_wrapper/VisualCabinet.h"

/*
// you should firstly execute scripts/test_file.sql
INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (0,'knifeSwitch');  
INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (1,'remoteSwitch');  
INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (2,'handcartSwitch');  
INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (3,'pointMeter');  
INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (4,'light');  

INSERT INTO `equipment_type_0_standard` (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`) 
                                 VALUES (0, 300,300,10,0,0,0,1,100,100); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_standard` (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`) 
                                 VALUES (1, 100,300,10,0,0,0,1,100,100); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_standard` (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`) 
                                 VALUES (2, 0,0,10,0,0,0,1,100,100); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_standard` (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`) 
                                 VALUES (3, 300,500,10,0,0,0,1,100,100); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_standard` (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`) 
                                 VALUES (4, -100,300,10,0,0,0,1,100,100); -- mm for x,y,z,width,height
*/

VisualCabinet createGoldenDataForStandardVisualCabinet(){
    VisualCabinet golden_visual_cabinet(0,0);
    VisualObject obj;
    obj.object_name_id = 0;
    obj.object_name = KNIFE_SWITCH_NAME;
    obj.pos(0) = 300;    obj.pos(1) = 300;    obj.pos(2) = 10;
    obj.q.x() = 0;    obj.q.y() = 0;    obj.q.z() = 0;    obj.q.w() = 0;
    obj.width = 100; obj.height = 100;
    obj.presented_frame = VOBJ_CABINET_FRAME;
    golden_visual_cabinet.visual_objs_in_cabinet_map[obj.object_unique_id_on_equipment] = obj;
    
    obj.object_name_id = 1;
    obj.object_name = REMOTE_SWITCH_NAME;
    obj.pos(0) = 100;    obj.pos(1) = 300;    obj.pos(2) = 10;
    obj.q.x() = 0;    obj.q.y() = 0;    obj.q.z() = 0;    obj.q.w() = 0;
    obj.width = 100; obj.height = 100;
    obj.presented_frame = VOBJ_CABINET_FRAME;
    golden_visual_cabinet.visual_objs_in_cabinet_map[obj.object_unique_id_on_equipment] = obj;

    obj.object_name_id = 2;
    obj.object_name = HANDCART_SWITCH_NAME;
    obj.pos(0) = 0;    obj.pos(1) = 0;    obj.pos(2) = 10;
    obj.q.x() = 0;    obj.q.y() = 0;    obj.q.z() = 0;    obj.q.w() = 0;
    obj.width = 100; obj.height = 100;
    obj.presented_frame = VOBJ_CABINET_FRAME;
    golden_visual_cabinet.visual_objs_in_cabinet_map[obj.object_unique_id_on_equipment] = obj;

    obj.object_name_id = 3;
    obj.object_name = POINT_METERS_NAME;
    obj.pos(0) = 300;    obj.pos(1) = 500;    obj.pos(2) = 10;
    obj.q.x() = 0;    obj.q.y() = 0;    obj.q.z() = 0;    obj.q.w() = 0;
    obj.width = 100; obj.height = 100;
    obj.presented_frame = VOBJ_CABINET_FRAME;
    golden_visual_cabinet.visual_objs_in_cabinet_map[obj.object_unique_id_on_equipment] = obj;


    obj.object_name_id = 4;
    obj.object_name = LIGHTS_NAME;
    obj.pos(0) = -100;    obj.pos(1) = 300;    obj.pos(2) = 10;
    obj.q.x() = 0;    obj.q.y() = 0;    obj.q.z() = 0;    obj.q.w() = 0;
    obj.width = 100; obj.height = 100;
    obj.presented_frame = VOBJ_CABINET_FRAME;
    golden_visual_cabinet.visual_objs_in_cabinet_map[obj.object_unique_id_on_equipment] = obj;
    return golden_visual_cabinet;
}
void assumeT_base_to_cabinet(Eigen::Matrix4d& T_base_to_cabinet){
  double roll = 0.5;//rad
  double pitch = 0.2; //rad
  double yaw = 0.1; // rad
  double x_dist = 100; //mm
  double y_dist = 10; //mm
  double z_dist = 10; //mm
  getTransformMatrix(Eigen::Vector3d(x_dist, y_dist, z_dist),
                    Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())), T_base_to_cabinet);
}
VisualCabinet createInitializedVisualCabinet(){
  VisualCabinet visual_cabinet = createGoldenDataForStandardVisualCabinet();
  Eigen::Matrix4d T_base_to_cabinet;
  assumeT_base_to_cabinet(T_base_to_cabinet);
  visual_cabinet.T_base_to_cabinet = T_base_to_cabinet;
  visual_cabinet.isInitialized();

  return visual_cabinet;
}
