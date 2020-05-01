CREATE DATABASE IF NOT EXISTS Power_distribution_room;
USE Power_distribution_room ;

CREATE TABLE IF NOT EXISTS object_name_on_equipment_type_0(
    object_type_id INT NOT NULL,
    object_type_name VARCHAR(255) NOT NULL,
    PRIMARY KEY (object_type_id));

-- This table stores object's positions on each cabinet equipment, because each of them might have slightly difference from the standard value.
--
CREATE TABLE IF NOT EXISTS equipment_type_0_details (
    id INT NOT NULL AUTO_INCREMENT,
    equipment_id INT NOT NULL,
    object_type_id INT NOT NULL,
    object_unique_id_on_equipment INT NOT NULL,
    x DOUBLE NOT NULL,
    y DOUBLE NOT NULL,
    z DOUBLE NOT NULL,

    q_x DOUBLE NOT NULL,
    q_y DOUBLE NOT NULL,
    q_z DOUBLE NOT NULL,
    q_w DOUBLE NOT NULL,

    width DOUBLE NOT NULL,
    height DOUBLE NOT NULL,
    
    x_from_obj_to_best_capture_point DOUBLE NOT NULL,
    y_from_obj_to_best_capture_point DOUBLE NOT NULL,
    z_from_obj_to_best_capture_point DOUBLE NOT NULL,

    result_offset_x_mm DOUBLE NOT NULL,
    result_offset_y_mm DOUBLE NOT NULL,
    result_offset_z_mm DOUBLE NOT NULL,
    result_offset_euler_x_deg DOUBLE NOT NULL,
    result_offset_euler_y_deg DOUBLE NOT NULL,
    result_offset_euler_z_deg DOUBLE NOT NULL,
    
    additional_text_info VARCHAR(255),
    can_auto_refine BOOLEAN NOT NULL,
    radius DOUBLE,
    trunk_square_size DOUBLE,
    FOREIGN KEY (equipment_id) REFERENCES equipment(id),
    FOREIGN KEY (object_type_id) REFERENCES object_name_on_equipment_type_0(object_type_id),
    PRIMARY KEY (ID));

-- add name into object_name_on_equipment

INSERT INTO `object_name_on_equipment_type_0` (`object_type_id`,`object_type_name`) VALUES (0,'QRCode');  
INSERT INTO `object_name_on_equipment_type_0` (`object_type_id`,`object_type_name`) VALUES (1,'knifeSwitch');  
INSERT INTO `object_name_on_equipment_type_0` (`object_type_id`,`object_type_name`) VALUES (2,'remoteSwitch');  
INSERT INTO `object_name_on_equipment_type_0` (`object_type_id`,`object_type_name`) VALUES (3,'handcartSwitch');  
INSERT INTO `object_name_on_equipment_type_0` (`object_type_id`,`object_type_name`) VALUES (4,'pointMeter');  
INSERT INTO `object_name_on_equipment_type_0` (`object_type_id`,`object_type_name`) VALUES (5,'light');  
