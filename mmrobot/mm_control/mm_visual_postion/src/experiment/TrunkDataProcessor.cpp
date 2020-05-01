#include "mm_visual_postion/hand_trunck/HandTrunkSolver.h"
#include "mm_visual_postion/experiment/DataProcessorBase.h"
#include "mm_visual_postion/utils/utils.h"
#include <random>

class TrunkDatasetProcessor:public DatasetProcessorBase{
private:
    HandTrunkSolver solver;
    std::vector<ConcentricCircles3D> solved_circles_in_cam_vec, solved_circles_in_base_vec;
    std::vector<Eigen::Quaterniond> q_trunk_vec;
    std::vector<bool> success_vec;
    Eigen::Matrix<double,4,4> T_cam_to_endeffector, T_endeffector_to_cam;
    double estimate_circle_radius; double radius_threshold; double max_acceptable_cost_per_point;
    double estimate_trunk_length;
    void writeVector4dHeaders(std::ostream &f, std::string vector_name){
        f << vector_name + "_0"<<","
          << vector_name + "_1"<<","
          << vector_name + "_2"<<","
          << vector_name + "_3";
    }
    void writeConcentricCircleInfo(std::ofstream& csv_file, int id, ConcentricCircles3D& circle_in_cam, ConcentricCircles3D& circle_in_base, Eigen::Quaterniond q, bool success){
        csv_file<< id <<","<<circle_in_cam.center<<","<<circle_in_cam.plane<<","
                <<circle_in_base.center<<","<<circle_in_base.plane<<","
                <<circle_in_base.radius_inner<<","<<circle_in_base.radius_outer<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<","<<success<<"\n";
                //<<gt_concentric_circle_in_base.center<<","<<gt_concentric_circle_in_base.plane<<","
                //<<gt_concentric_circle_in_base.radius_inner<<","<<gt_concentric_circle_in_base.radius_outer<<","<<trunk_angle<<","<<success<<"\n";
    }

    void writeConcentricCircleHeader(std::ofstream& csv_file){
        csv_file<<"id"<<",";
        writeVector4dHeaders(csv_file, "center_in_cam");
        csv_file<<",";
        writeVector4dHeaders(csv_file, "plane_in_cam");
        csv_file<<",";
        writeVector4dHeaders(csv_file, "center_in_base");
        csv_file<<",";
        writeVector4dHeaders(csv_file, "plane_in_base");
        csv_file<<","<<"radius_inner"<<","<<"radius_outer"<<",";

        //writeVector4dHeaders(csv_file, "gt_center_in_base");
        //csv_file<<",";
        //writeVector4dHeaders(csv_file, "gt_plane_in_base");
        //csv_file<<","<<"gt_radius_inner"<<","<<"gt_radius_outer"<<",";
        csv_file<<"q_x"<<","<<"q_y"<<","<<"q_z"<<","<<"q_w"<<","<<"success"<<"\n";
    }
public:
    TrunkDatasetProcessor(ros::NodeHandle& n, std::string read_dir_input, double estimate_circle_radius_input, 
                            double radius_threshold_input, double max_acceptable_cost_per_point_input,
                            double estimate_trunk_length_input)
        :DatasetProcessorBase(n, read_dir_input), solver(model_ptr)
    {
        cv::namedWindow("show_result_image", 0);
        T_endeffector_to_cam = model_ptr->endeffector_to_cam_transform;
        T_cam_to_endeffector = T_endeffector_to_cam.inverse();
        radius_threshold = radius_threshold_input; //mm
        estimate_circle_radius = estimate_circle_radius_input; //mm
        max_acceptable_cost_per_point = max_acceptable_cost_per_point_input;
        estimate_trunk_length = estimate_trunk_length_input; //mm
    }
    void computeConcentricCircle(std::vector<int> id_vec){
        std::vector<cv::Mat> left_image_vec(id_vec.size());
        std::vector<cv::Mat> right_image_vec(id_vec.size());
        std::vector<cv::Rect> left_roi_vec(id_vec.size());
        std::vector<cv::Rect> right_roi_vec(id_vec.size());
        for(unsigned int i=0; i<id_vec.size(); i++){
            loadImage(id_vec[i], left_image_vec[i], right_image_vec[i]);
        }
        std::vector<Eigen::Matrix4d> transform_cam_to_base_vec(id_vec.size());
        for(unsigned int i=0; i<id_vec.size(); i++){
            Eigen::Matrix4d T_endeffector_to_base;
            transformToMatrix(transform_vec[id_vec[i]], T_endeffector_to_base);
            transform_cam_to_base_vec[i] = T_cam_to_endeffector * T_endeffector_to_base;

            left_roi_vec[i] = cv::Rect(0,0,left_image_vec[i].cols, left_image_vec[i].rows);
            right_roi_vec[i] = cv::Rect(0,0,right_image_vec[i].cols, right_image_vec[i].rows);
            
            ////////////debug
            //transform_cam_to_base_vec[i].setIdentity();
        }
        cv::Mat result_image;
        Circle3D circles_in_base;
        Eigen::Matrix4d T_base_to_trunk;
        if(solver.findTrunk(left_image_vec, right_image_vec, left_roi_vec, right_roi_vec, transform_cam_to_base_vec, estimate_circle_radius, radius_threshold, max_acceptable_cost_per_point, 
                            estimate_trunk_length, circles_in_base, T_base_to_trunk, &result_image))
        {   
            solved_circles_in_cam_vec.push_back(ConcentricCircles3D::transformConcentricCircle(circles_in_base, transform_cam_to_base_vec[0]));
            solved_circles_in_base_vec.push_back(circles_in_base);
            Eigen::Quaterniond q_eigen(T_base_to_trunk.block<3,3>(0,0));
            q_trunk_vec.push_back(q_eigen);
            success_vec.push_back(true);

            // visualization
            cv::imshow("show_result_image", result_image);
            cv::waitKey(5);
            saveImage(id_vec[0], "combile", result_image);
        }
        else{
            solved_circles_in_cam_vec.push_back(ConcentricCircles3D());
            solved_circles_in_base_vec.push_back(ConcentricCircles3D());
            
            q_trunk_vec.push_back(Eigen::Quaterniond());

            success_vec.push_back(false);
        }

    }

    void saveImage(int id, std::string method_name, cv::Mat& image){
        std::stringstream ss;
        ss << read_dir<<"/processed/" << method_name ;
        if(!isDirExist(ss.str())){
            makePath(ss.str());
        }
        ss << "/"<<id;

        std::string image_path = ss.str() + std::string(".jpg");
        cv::imwrite(image_path, image);
    }
    void writeResult(std::ofstream& csv_file){
        writeConcentricCircleHeader(csv_file);
        for(unsigned int i=0 ;i<solved_circles_in_base_vec.size(); i++){
            writeConcentricCircleInfo(csv_file, i, solved_circles_in_cam_vec[i], solved_circles_in_base_vec[i], q_trunk_vec[i], success_vec[i]);
        }
    }
};



int main(int argc, char** argv){
    ros::init(argc, argv, "trunk_dataset_processor");
	ros::NodeHandle nh;
    if(argc <= 1){
        ROS_ERROR("please input the data's directory name");
    }
    std::string path = ros::package::getPath("mm_visual_postion") + std::string("/dataset/trunk/");

    path.append(argv[1]);
    if(!isDirExist(path+"/dataset/processed")){
        makePath(path+"/dataset/processed");
    }
    TrunkDatasetProcessor dataset_processor(nh,path+"/dataset", 28, 3, 1000, 13); //mm
    std::vector<int> id_vec;

    
    std::ofstream result_csv_file;
    result_csv_file.open(path + std::string("/dataset/processed/result.csv"));
    
    std::random_device rd;
    //std::mt19937 gen(rd());
    std::mt19937 gen;
    gen.seed(0);
    std::uniform_int_distribution<> dis(0, dataset_processor.test_size-1);
    unsigned int fuse_num = 1;
    for(int i=0; i<dataset_processor.test_size; i++){
        std::vector<int> id_vec;
        while(id_vec.size() < fuse_num){
            bool continue_flag = false;
            int select_id = dis(gen);
            for(unsigned int id_i=0; id_i < id_vec.size(); id_i++){
                if(select_id == id_vec[id_i]){
                    continue_flag=true;
                    break;
                } 
            }
            if(!continue_flag)
                id_vec.push_back(select_id);
        }
        
        dataset_processor.computeConcentricCircle(id_vec);
    }
    dataset_processor.writeResult(result_csv_file);

    return 0;    
}