CREATE DATABASE IF NOT EXISTS Power_distribution_room;
USE Power_distribution_room ;
-- add name into object_name_on_equipment
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (0,'knifeSwitch');  
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (1,'remoteSwitch');  
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (2,'handcartSwitch');  
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (3,'pointMeter');  
-- INSERT INTO `object_name_on_equipment` (`object_type_id`,`object_type_name`) VALUES (4,'light');  

-- add some data into equipment_type_0_details
INSERT INTO `equipment_type_0_details` (`equipment_id`,`object_type_id`, `object_unique_id_on_equipment`, `x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`, `radius`, `trunk_square_size`) 
                                 VALUES (1, 0, 0, -420,-720, 0,0,0,0,1, 50, 50,0,0,-500, 0, 0, 0, 0, 0, 0, '0_0_0', TRUE, NULL, NULL); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_details` (`equipment_id`,`object_type_id`, `object_unique_id_on_equipment`, `x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`, `radius`, `trunk_square_size`) 
                                 VALUES (1, 0, 1, 420,-750, 0,0,0,0,1, 50, 50,0,0,-500, 0, 0, 0, 0, 0, 0, '0_0_1', TRUE, NULL, NULL); -- mm for x,y,z,width,height

INSERT INTO `equipment_type_0_details` (`equipment_id`,`object_type_id`, `object_unique_id_on_equipment`, `x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`, `radius`, `trunk_square_size`) 
                                 VALUES (1, 1, 2,315, -720, -6, 0, 0, 0, 1, 48, 48, 0,100,-500, 1, 0, 0, 0, 0, 0, NULL, TRUE, NULL, NULL); -- mm for x,y,z,width,height, x_from_obj_to_best_capture_point,  y_from_obj_to_best_capture_point, z_from_obj_to_best_capture_point
INSERT INTO `equipment_type_0_details` (`equipment_id`,`object_type_id`, `object_unique_id_on_equipment`, `x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`, `radius`, `trunk_square_size`) 
                                 VALUES (1, 2, 3,175, -720, -6, 0, 0, 0, 1, 48, 48, 0,100,-500, 3, 0, 0, 0, 0, 0, NULL, TRUE, NULL, NULL); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_details` (`equipment_id`,`object_type_id`, `object_unique_id_on_equipment`, `x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`, `radius`, `trunk_square_size`) 
                                 VALUES (1, 3, 4,0,0, -3,0,0,1,0, 55, 55, 0,100,-400, 0, 2, 26 , 0, 0, 0, NULL, TRUE, 26, 25); -- mm for x,y,z,width,height, handcart is rotated by 180 around z-axis
INSERT INTO `equipment_type_0_details` (`equipment_id`,`object_type_id`, `object_unique_id_on_equipment`, `x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`, `radius`, `trunk_square_size`) 
                                 VALUES (1, 4, 5,225, -865, -18,0,0,0,1, 235, 80, 0, 100, -400, 0, 0, 0, 0, 0, 0, NULL, FALSE, NULL, NULL); -- mm for x,y,z,width,height
INSERT INTO `equipment_type_0_details` (`equipment_id`,`object_type_id`, `object_unique_id_on_equipment`, `x`,`y`,`z`,`q_x`,`q_y`,`q_z`,`q_w`,`width`,`height`,`x_from_obj_to_best_capture_point`, `y_from_obj_to_best_capture_point`, `z_from_obj_to_best_capture_point`, `result_offset_x_mm`, `result_offset_y_mm`, `result_offset_z_mm`, `result_offset_euler_x_deg`, `result_offset_euler_y_deg`, `result_offset_euler_z_deg`, `additional_text_info`, `can_auto_refine`, `radius`, `trunk_square_size`) 
                                 VALUES (1, 5, 6,-115,-720, -15,0,0,0,1, 348, 30,0,0,-500, 0, 0, 0, 0, 0, 0, NULL, FALSE, NULL, NULL); -- mm for x,y,z,width,height