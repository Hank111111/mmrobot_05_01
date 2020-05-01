/*
This program is intended to process the generated data, which include the rectified image, the arm's transition matrix to base in each image, etc.
This program uses different method to compute the position of switch in each image, and generate a csv file, which includes the error information etc.
The generated csv file is intended to be analysed by a python script, visualizing data etc.
*/

#include "ros/ros.h"
#include <ros/package.h>

#include <vector>
#include <string>

#include "mm_visual_postion/switch/CoordinateCalculation.h"
#include "mm_visual_postion/experiment/DataProcessorBase.h"
#include "mm_visual_postion/utils/CheckCreatePath.h"
void debugPubStaticTransform(std::vector<double>& xyzabc, std::string origin_frame_name, std::string target_frame_name){
		geometry_msgs::TransformStamped static_cam_endeffector_transformStamped;
		Eigen::Matrix4d trans;
		createTransformationMatrix(xyzabc, trans);

		static_cam_endeffector_transformStamped.header.stamp = ros::Time::now();
		static_cam_endeffector_transformStamped.header.frame_id = origin_frame_name;
		static_cam_endeffector_transformStamped.child_frame_id = target_frame_name;
		static_cam_endeffector_transformStamped.transform.translation.x = trans(0,3) / 1000.0;
		static_cam_endeffector_transformStamped.transform.translation.y = trans(1,3) / 1000.0;
		static_cam_endeffector_transformStamped.transform.translation.z = trans(2,3) / 1000.0;
		Eigen::Quaternion<double> q(trans.block<3,3>(0,0));
		static_cam_endeffector_transformStamped.transform.rotation.x = q.x();
		static_cam_endeffector_transformStamped.transform.rotation.y = q.y();
		static_cam_endeffector_transformStamped.transform.rotation.z = q.z();
		static_cam_endeffector_transformStamped.transform.rotation.w = q.w();
		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		static_broadcaster.sendTransform(static_cam_endeffector_transformStamped);
		ros::spinOnce();
}
class SwitchDatasetProcessor:public DatasetProcessorBase{
private:

    CoordinateCalculation coord_cal;
    std::vector<cv::Mat> template_image_vec;
    cv::Vec4f template_margin;
    
    std::vector<cv::Point3d> cv_corners_in_base;
    std::vector<Eigen::Vector4d> corners_pos_in_base;
    std::vector<double> gt_xyzabc_in_base;


    void readGroundTruthPose(){
        std::string file_path = read_dir + std::string("/ground_truth/result.csv");
        std::ifstream  finput(file_path);
        if( !finput ){
            ROS_ERROR("cannot open the file %s", file_path.c_str());
            exit(-1);
        }

        std::string line;
        corners_pos_in_base.clear();
        while(std::getline(finput,line))
        {
            std::stringstream  lineStream(line);
            std::string        cell;
            int i = 0;
            Eigen::Vector4d tmp;
            while(std::getline(lineStream,cell,','))
            {   
                tmp(i)= std::stod(cell);
                i += 1;
            }
            tmp[i] = 1;
            if( i !=3 ){
                ROS_ERROR("groud truth corner's pos file doesn't have exactly 4 cols");
                exit(-1);
            }
            corners_pos_in_base.push_back(tmp);
        }
        if(corners_pos_in_base.size() != 4){
            ROS_ERROR("groud truth corner's pos file doesn't have exactly 4 rows");
            exit(-1);
        }

        cv_corners_in_base.resize(4);
        for(unsigned int i=0; i<4; i++){
            cv_corners_in_base[i].x = corners_pos_in_base[i](0);
            cv_corners_in_base[i].y = corners_pos_in_base[i](1);
            cv_corners_in_base[i].z = corners_pos_in_base[i](2);
        }
        double length;
        coord_cal.computeSwitchPose(cv_corners_in_base, gt_xyzabc_in_base, length); // in base coord
    }


    void loadTemplate(){
        std::string path = ros::package::getPath("mm_visual_postion");
        cv::Mat template_image = cv::imread(path + std::string("/template_images/switch_1_s1_v0.bmp")); 
        if (template_image.empty())
            ROS_ERROR("cannot load template image");
        template_image_vec.push_back(template_image.clone());
        template_image.release();

        template_image = cv::imread(path + std::string("/template_images/switch_1_s1_v1.bmp"));
        if (template_image.empty())
            ROS_ERROR("cannot load template image");
        
        template_image_vec.push_back(template_image.clone());
        
        template_margin = cv::Vec4f(0.0, 0.0, 0.0, 0.0);

    }

public:
    int test_size;
    void writeResultInfoVector(std::string name, std::vector<ResultInfo>& result_info_vec, std::vector<int>& id_vec){
        std::ofstream result_info_csv;
        result_info_csv.open(read_dir + std::string("/processed/")+name);
        result_info_csv<<"id"<<","<<"success"<<","
                    <<"left_corners_in_pixel_0_x"<<","<<"left_corners_in_pixel_0_y"<<","
                    <<"left_corners_in_pixel_1_x" <<","<<"left_corners_in_pixel_1_y"<<","
                    <<"left_corners_in_pixel_2_x"<<","<<"left_corners_in_pixel_2_y"<<","
                    <<"left_corners_in_pixel_3_x"<<","<<"left_corners_in_pixel_3_y"<<","
                    <<"right_corners_in_pixel_0_x"<<","<<"right_corners_in_pixel_0_y"<<","
                    <<"right_corners_in_pixel_1_x"<<","<<"right_corners_in_pixel_1_y"<<","
                    <<"right_corners_in_pixel_2_x"<<","<<"right_corners_in_pixel_2_y"<<","
                    <<"right_corners_in_pixel_3_x"<<","<<"right_corners_in_pixel_3_y"<<","
                    <<"corners_in_base_0_x"<<","<<"corners_in_base_0_y"<<","<<"corners_in_base_0_z"<<","
                    <<"corners_in_base_1_x"<<","<<"corners_in_base_1_y"<<","<<"corners_in_base_1_z"<<","
                    <<"corners_in_base_2_x"<<","<<"corners_in_base_2_y"<<","<<"corners_in_base_2_z"<<","
                    <<"corners_in_base_3_x"<<","<<"corners_in_base_3_y"<<","<<"corners_in_base_3_z"<<","
                    <<"xyz_abc_in_cam_x"<<","<<"xyz_abc_in_cam_y"<<","<<"xyz_abc_in_cam_z"<<","<<"xyz_abc_in_cam_a"<<","<<"xyz_abc_in_cam_b"<<","<<"xyz_abc_in_cam_c"<<","
                    <<"xyz_abc_in_tool_x"<<","<<"xyz_abc_in_tool_y"<<","<<"xyz_abc_in_tool_z"<<","<<"xyz_abc_in_tool_a"<<","<<"xyz_abc_in_tool_b"<<","<<"xyz_abc_in_tool_c"<<","
                    <<"xyz_abc_in_base_x"<<","<<"xyz_abc_in_base_y"<<","<<"xyz_abc_in_base_z"<<","<<"xyz_abc_in_base_a"<<","<<"xyz_abc_in_base_b"<<","<<"xyz_abc_in_base_c"<<","

                    <<"gt_xyz_abc_in_base_x"<<","<<"gt_xyz_abc_in_base_y"<<","<<"gt_xyz_abc_in_base_z"<<","<<"gt_xyz_abc_in_base_a"<<","<<"gt_xyz_abc_in_base_b"<<","<<"gt_xyz_abc_in_base_c"<<","
                    <<"gt_xyz_abc_in_cam_x"<<","<<"gt_xyz_abc_in_cam_y"<<","<<"gt_xyz_abc_in_cam_z"<<","<<"gt_xyz_abc_in_cam_a"<<","<<"gt_xyz_abc_in_cam_b"<<","<<"gt_xyz_abc_in_cam_c"
                    <<"\n";

        for(unsigned int i=0; i< result_info_vec.size(); i++){
            vector<Point2d> left_corners_in_pixel = result_info_vec[i].left_corners_in_pixel;
            vector<Point2d> right_corners_in_pixel = result_info_vec[i].right_corners_in_pixel;
            vector<Eigen::Vector4d> corners_in_base = result_info_vec[i].corners_in_base;
            vector<double> xyz_abc_in_cam = result_info_vec[i].xyz_abc_in_cam;
            vector<double> xyz_abc_in_tool = result_info_vec[i].xyz_abc_in_tool;
            vector<double> xyz_abc_in_base = result_info_vec[i].xyz_abc_in_base;

            bool success = result_info_vec[i].success;
            vector<double> gt_xyzabc_in_cam = result_info_vec[i].gt_xyz_abc_in_cam;
            result_info_csv<<id_vec[i]<<","<<success<<","
                        <<left_corners_in_pixel[0].x<<","<<left_corners_in_pixel[0].y<<","
                        <<left_corners_in_pixel[1].x<<","<<left_corners_in_pixel[1].y<<","
                        <<left_corners_in_pixel[2].x<<","<<left_corners_in_pixel[2].y<<","
                        <<left_corners_in_pixel[3].x<<","<<left_corners_in_pixel[3].y<<","
                        <<right_corners_in_pixel[0].x<<","<<right_corners_in_pixel[0].y<<","
                        <<right_corners_in_pixel[1].x<<","<<right_corners_in_pixel[1].y<<","
                        <<right_corners_in_pixel[2].x<<","<<right_corners_in_pixel[2].y<<","
                        <<right_corners_in_pixel[3].x<<","<<right_corners_in_pixel[3].y<<","
                        <<corners_in_base[0](0)<<","<<corners_in_base[0](1)<<","<<corners_in_base[0](2)<<","
                        <<corners_in_base[1](0)<<","<<corners_in_base[1](1)<<","<<corners_in_base[1](2)<<","
                        <<corners_in_base[2](0)<<","<<corners_in_base[2](1)<<","<<corners_in_base[2](2)<<","
                        <<corners_in_base[3](0)<<","<<corners_in_base[3](1)<<","<<corners_in_base[3](2)<<","
                        <<xyz_abc_in_cam[0]<<","<<xyz_abc_in_cam[1]<<","<<xyz_abc_in_cam[2]<<","<<xyz_abc_in_cam[3]<<","<<xyz_abc_in_cam[4]<<","<<xyz_abc_in_cam[5]<<","
                        <<xyz_abc_in_tool[0]<<","<<xyz_abc_in_tool[1]<<","<<xyz_abc_in_tool[2]<<","<<xyz_abc_in_tool[3]<<","<<xyz_abc_in_tool[4]<<","<<xyz_abc_in_tool[5]<<","
                        <<xyz_abc_in_base[0]<<","<<xyz_abc_in_base[1]<<","<<xyz_abc_in_base[2]<<","<<xyz_abc_in_base[3]<<","<<xyz_abc_in_base[4]<<","<<xyz_abc_in_base[5]<<","

                        <<gt_xyzabc_in_base[0]<<","<<gt_xyzabc_in_base[1]<<","<<gt_xyzabc_in_base[2]<<","<<gt_xyzabc_in_base[3]<<","<<gt_xyzabc_in_base[4]<<","<<gt_xyzabc_in_base[5]<<","
                        <<gt_xyzabc_in_cam[0]<<","<<gt_xyzabc_in_cam[1]<<","<<gt_xyzabc_in_cam[2]<<","<<gt_xyzabc_in_cam[3]<<","<<gt_xyzabc_in_cam[4]<<","<<gt_xyzabc_in_cam[5]
                        <<"\n";
        }
        result_info_csv.close();
    }

    SwitchDatasetProcessor(ros::NodeHandle& n, std::string read_dir_input)
        :DatasetProcessorBase(n, read_dir_input)
    {
        loadTemplate();
        coord_cal.updateCameraArmParams(model_ptr);
        readGroundTruthPose();
    }

    void computePosInBaseWithHough(int id, ResultInfo* result_info_ptr){
        std::vector<double> xyzabc_in_tool;
        cv::Mat left_image, right_image;
        loadImage(id, left_image, right_image);
        double match_score;
        coord_cal.calculateSwitchPosSimple(left_image, right_image, template_image_vec, template_margin,
                                    transform_vec[id],xyzabc_in_tool,METHOD_HOUGH,match_score,result_info_ptr);
        computeGroundTruthxyzabcInCam(id, result_info_ptr); // add ground truth into result_info_ptr

        saveImage(id, "hough", result_info_ptr->show_left, result_info_ptr->show_right);
        result_info_ptr->releaseShowImage();
    }
    void computePosInBaseWithMorphology(int id, ResultInfo* result_info_ptr){
        std::vector<double> xyzabc_in_tool;
        cv::Mat left_image, right_image;
        loadImage(id, left_image, right_image);
        double match_score;
        coord_cal.calculateSwitchPosSimple(left_image, right_image, template_image_vec, template_margin,
                                    transform_vec[id],xyzabc_in_tool,METHOD_MORPHOLOGY,match_score,result_info_ptr);
        computeGroundTruthxyzabcInCam(id, result_info_ptr);

        saveImage(id, "morphology", result_info_ptr->show_left, result_info_ptr->show_right);
        result_info_ptr->releaseShowImage();
    }
    void computePosInBaseWithCombineMethods(int id, ResultInfo* result_info_ptr){
        std::vector<double> xyzabc_in_tool;
        cv::Mat left_image, right_image;
        loadImage(id, left_image, right_image);
        double match_score;
        coord_cal.calculateSwitchPosSimple(left_image, right_image, template_image_vec, template_margin,
                                    transform_vec[id],xyzabc_in_tool,METHOD_COMBINE_HOUGH_MORPH,match_score,result_info_ptr);
        computeGroundTruthxyzabcInCam(id, result_info_ptr);

        saveImage(id, "combile", result_info_ptr->show_left, result_info_ptr->show_right);
        result_info_ptr->releaseShowImage();
    }
    void computePosInBaseWithParticleFilter(int update_step_num, ResultInfo* result_info_ptr){
        // the particle filter will be firstly reset.
        // id_seq is the sequence of id, whose images will be used to update the particle filter
        coord_cal.resetParticleFilter();
        std::vector<double> xyzabc_in_tool;
        std::vector<int> id_seq(update_step_num);
        std::random_device rd;
        std::default_random_engine gen(rd());
        std::uniform_int_distribution<int> dis(0, test_size-1);

        for(int i=0; i<update_step_num; i++){
            id_seq[i] = dis(gen);
                // check whether test_size[j] is already existed
            if(checkAlreadyExist(i, id_seq)){
                i--; //re-sample
                continue;
            }

            cv::Mat left_image, right_image;
            loadImage(id_seq[i], left_image, right_image);
            ResultInfo result_info_tmp;
            if(coord_cal.calculateSwitchPosParticleFilter(left_image, right_image, template_image_vec, template_margin,
                                        transform_vec[id_seq[i]],xyzabc_in_tool,METHOD_HOUGH, result_info_ptr) <0 ){
                // re-sample
                i--;
                continue;
            }
            computeGroundTruthxyzabcInCam(id_seq[i], result_info_ptr);

            std::cout<<"test"<<std::endl;
        }
    
        result_info_ptr->releaseShowImage();
    }
    bool checkAlreadyExist(int curr_index, std::vector<int>& vec){
        for(int check_j=0; check_j<curr_index; check_j++){
            if(vec[check_j] == vec[curr_index]) return true;
        }
        return false;
    }
    void testPosInBaseWithParticleFilter(int test_num, int update_step_num, std::vector<int>& id_vec, std::vector<ResultInfo>& result_info_vec){
        for(int i=0; i<test_num; i++){
            id_vec.push_back(i);
            ResultInfo tmp;
            computePosInBaseWithParticleFilter(update_step_num,&tmp);
            result_info_vec.push_back(tmp);
            std::cout<<"current test id: "<<i<<std::endl;
        }
    }


    void computeGroundTruthxyzabcInCam(int id, ResultInfo* result_info_ptr){
        result_info_ptr->gt_xyz_abc_in_cam.resize(6);
        coord_cal.left_transformer_ptr->setTransEndEffectorToBase(transform_vec[id]);
        coord_cal.left_transformer_ptr->getObjPoseInCamFromObjPoseInBase(gt_xyzabc_in_base, result_info_ptr->gt_xyz_abc_in_cam);
        result_info_ptr->gt_xyz_abc_in_base = gt_xyzabc_in_base;
        debugPubStaticTransform(gt_xyzabc_in_base, "base", "gt_object");

    }
    

};


int main(int argc, char** argv){
    ros::init(argc, argv, "dataset_processor");
	ros::NodeHandle nh;
    if(argc <= 1){
        ROS_ERROR("please input the data's directory name");
    }
    std::string path = ros::package::getPath("mm_visual_postion") + std::string("/dataset/");

    path.append(argv[1]);
    if(!isDirExist(path+"/dataset/processed")){
        makePath(path+"/dataset/processed");
    }
    SwitchDatasetProcessor dataset_processor(nh,path+"/dataset");
    std::vector<ResultInfo> result_info_vec;
    std::vector<int> id_vec;

    
    
    for(int i=0; i<dataset_processor.test_size; i++){
        ResultInfo tmp;
        dataset_processor.computePosInBaseWithHough(i, &tmp);
        result_info_vec.push_back(tmp);
        id_vec.push_back(i);
        //cv::waitKey(0);
    }
    dataset_processor.writeResultInfoVector("result_method_hough.csv", result_info_vec, id_vec);
 
    /*
    result_info_vec.clear();
    id_vec.clear();
    for(unsigned int i=0; i<dataset_processor.test_size; i++){
        ResultInfo tmp;
        dataset_processor.computePosInBaseWithMorphology(i, &tmp);
        result_info_vec.push_back(tmp);
        id_vec.push_back(i);
    }
    dataset_processor.writeResultInfoVector("result_method_morphology.csv", result_info_vec, id_vec);
    
    result_info_vec.clear();
    id_vec.clear();
    for(unsigned int i=0; i<dataset_processor.test_size; i++){
        ResultInfo tmp;
        dataset_processor.computePosInBaseWithCombineMethods(i, &tmp);
        result_info_vec.push_back(tmp);
        id_vec.push_back(i);
    }
    dataset_processor.writeResultInfoVector("result_method_combile.csv", result_info_vec, id_vec);
    */

    result_info_vec.clear();
    id_vec.clear();
    dataset_processor.testPosInBaseWithParticleFilter(dataset_processor.test_size, 5, id_vec, result_info_vec);
    dataset_processor.writeResultInfoVector("result_hough_particle_filter.csv", result_info_vec, id_vec);


    return 0;
}