CREATE DATABASE IF NOT EXISTS Power_distribution_room;
USE Power_distribution_room ;
-- add name into object_name_on_equipment
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (0,'knifeSwitch');  
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (1,'remoteSwitch');  
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (2,'handcartSwitch');  
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (3,'pointMeter');  
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (4,'light');  

-- add some data into equipment_type_0_standard
INSERT INTO `equipment_type_0_standard` (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`) 
                                 VALUES (0, 300,300,10,0,0,0,1,100,100, 0,100,-500, 0, 0, 0, 0, 0, 0, NULL); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_standard`  (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`) 
                                 VALUES (1, 100,300,10,0,0,0,1,100,100, 0,100,-500, 0, 0, 0, 0, 0, 0, NULL); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_standard`  (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`) 
                                 VALUES (2, 0,0,10,0,0,0,1,100,100, 0,100,-500, 0, 0, 0, 0, 0, 0, NULL); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_standard`  (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`) 
                                 VALUES (3, 300,500,10,0,0,0,1,100,100, 0,100,-500, 0, 0, 0, 0, 0, 0, NULL); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_standard`  (`object_type_id`,`x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`) 
                                 VALUES (4, -100,300,10,0,0,0,1,100,100, 0,100,-500, 0, 0, 0, 0, 0, 0, NULL); -- mm for x,y,z,width,height
