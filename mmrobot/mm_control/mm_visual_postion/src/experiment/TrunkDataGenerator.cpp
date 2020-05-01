#include "mm_visual_postion/experiment/DataGeneratorBase.h"
#include "mm_visual_postion/hand_trunck/HandTrunkSolver.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/utils/CheckCreatePath.h"
class TrunkDataGenerator:public DataGeneratorBase{
private:
    geometry_msgs::TransformStamped origin_transform_endeffector_to_base;
    Circle3D gt_circle_in_base;
    std::shared_ptr<HandTrunkSolver> solver_ptr;
public:
    TrunkDataGenerator(ros::NodeHandle& n, std::string save_dir):
        DataGeneratorBase(n, save_dir) 
    {
        solver_ptr = std::make_shared<HandTrunkSolver>(model_ptr);
    }


    void writeVector4dHeaders(std::ostream &f, std::string vector_name){
        f << vector_name + "_0"<<","
          << vector_name + "_1"<<","
          << vector_name + "_2"<<","
          << vector_name + "_3";
    }
    void writeCircleInfo(std::ofstream& csv_file, Circle3D& circle_in_cam, Circle3D& circle_in_base){
        csv_file<<circle_in_cam.center<<","<<circle_in_cam.plane<<","
                <<circle_in_base.center<<","<<circle_in_base.plane<<","
                <<circle_in_base.radius<<","
                <<gt_circle_in_base.center<<","<<gt_circle_in_base.plane<<","
                <<gt_circle_in_base.radius
                <<"\n";
    }
    void writeCircleHeader(std::ofstream& csv_file){
        writeVector4dHeaders(csv_file, "center_in_cam");
        csv_file<<",";
        writeVector4dHeaders(csv_file, "plane_in_cam");
        csv_file<<",";
        writeVector4dHeaders(csv_file, "center_in_base");
        csv_file<<",";
        writeVector4dHeaders(csv_file, "plane_in_base");
        csv_file<<","<<"radius"<<",";

        writeVector4dHeaders(csv_file, "gt_center_in_base");
        csv_file<<",";
        writeVector4dHeaders(csv_file, "gt_plane_in_base");
        csv_file<<","<<"gt_radius"<<"\n";
    }
    

    bool manuallyGetGroundTruthData(double estimate_radius, double radius_threshold, double max_acceptable_cost_per_point, double square_trunk_length){
        grabImagesAndTransform(latest_left_image, latest_right_image, origin_transform_endeffector_to_base ,latest_update_time);
            
        Circle3D circle;
        cv::Mat show_img;
        std::vector<cv::Mat> left_image_vec;
        left_image_vec.push_back(latest_left_image);
        std::vector<cv::Mat> right_image_vec;
        right_image_vec.push_back(latest_right_image);
        std::vector<Eigen::Matrix4d> dummy_transform;
        Eigen::Matrix4d dummy_T_base_to_trunk;
        cv::Rect dummy_roi(0,0,latest_left_image.cols, latest_left_image.rows);
        std::vector<cv::Rect> dummy_roi_vec;
        dummy_roi_vec.push_back(dummy_roi);
        if(!solver_ptr->findTrunk(left_image_vec, right_image_vec, dummy_roi_vec, dummy_roi_vec, dummy_transform, 
                                    estimate_radius, radius_threshold, 
                                    max_acceptable_cost_per_point, square_trunk_length, circle,
                                 dummy_T_base_to_trunk, &show_img)){
            ROS_WARN("Did not find the concentric circles..."); 
            return false;
        }
        cv::namedWindow("circle", 0);
        cv::imshow("circle", show_img);
        cv::waitKey(0);
        
        Eigen::Matrix4d T_cam_to_base, T_endeffector_to_base;
        transformToMatrix(origin_transform_endeffector_to_base, T_endeffector_to_base);
        T_cam_to_base = T_cam_to_endeffector * T_endeffector_to_base;
        gt_circle_in_base = Circle3D::transformCircle3D(circle, T_cam_to_base);
    

        // save ground left/right image corners
        std::ofstream gt_csv_file;
        gt_csv_file.open(save_dir + std::string("/ground_truth/ground_truth_concentric_circle.csv"));
        writeCircleHeader(gt_csv_file);
        writeCircleInfo(gt_csv_file,circle,gt_circle_in_base);
        gt_csv_file.close();
    
        return true;
        
    }
    virtual bool cmdIsValid(Eigen::Matrix4d& sample_T_endeffector_to_base){
        // check whether all of the four corners can be seen from two images by this command
        std::vector<Eigen::Vector4d> check_points_in_base(1);
        check_points_in_base[0] = gt_circle_in_base.center;
        if(canSeePointsAndIsSafe(sample_T_endeffector_to_base, 200, 200, check_points_in_base))
            return true;
        else return false;
    }
    
    void generateDataset(int sample_num){
        std::vector<double> xyzabc_cmd;
        int sample_i = 0;
        const double x_var = 200;
        const double y_var = 200;
        const double z_var = 200;
        const double q_var = 0.01;
        generateMoveCommand(origin_transform_endeffector_to_base, x_var, y_var, z_var, q_var, xyzabc_cmd);

        arm_commander.moveByBaseSpacePlanning(xyzabc_cmd[0] / 1000., xyzabc_cmd[1]/ 1000., xyzabc_cmd[2]/ 1000., 
                                xyzabc_cmd[3], xyzabc_cmd[4], xyzabc_cmd[5]);

        while(ros::ok()){

            if(arm_commander.isArrived()){
                // arm arrives the cmd position
                // capture the image
                sleep(1); // wait for camera to stablize

                grabTestData(sample_i);
                sample_i += 1;
                if(sample_i >= sample_num) break;
                generateMoveCommand(origin_transform_endeffector_to_base, x_var, y_var, z_var, q_var, xyzabc_cmd);
                arm_commander.moveByBaseSpacePlanning(xyzabc_cmd[0]/ 1000., xyzabc_cmd[1]/ 1000., xyzabc_cmd[2]/ 1000., 
                            xyzabc_cmd[3], xyzabc_cmd[4], xyzabc_cmd[5]);
                sleep(1); //wait for arm to move
            }
            else{
                bool time_out = arm_commander.isTimeOut();
                if(time_out){
                    generateMoveCommand(origin_transform_endeffector_to_base,  x_var, y_var, z_var, q_var, xyzabc_cmd);
                        arm_commander.moveByBaseSpacePlanning(xyzabc_cmd[0] / 1000., xyzabc_cmd[1]/ 1000., xyzabc_cmd[2]/ 1000., 
                                        xyzabc_cmd[3], xyzabc_cmd[4], xyzabc_cmd[5]);
                } 
            }
            sleep(1);
            ros::spinOnce();
        }
        writeTransformData(test_transform_data, "test_transform");
    }

};


int main(int argc, char** argv){
    ros::init(argc, argv, "trunk_dataset_generator_node");
	ros::NodeHandle nh;
    std::string path = ros::package::getPath("mm_visual_postion");

    // get current time string as the folder's name
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d_%m_%Y_%H_%M_%S");
    auto time_str = oss.str();

    path = path +std::string("/")+time_str +std::string("/");


    if(!isDirExist(path+"/dataset/ground_truth")){
        makePath(path+"/dataset/ground_truth");
    }
    TrunkDataGenerator dataset_generator(nh, path+"/dataset");

    dataset_generator.waitForManuallyMove();
    // compute ground truth pose of object from base
    dataset_generator.manuallyGetGroundTruthData(28, 3, 1000, 13); //mm

    // grab test dataset
    dataset_generator.generateDataset(200);

    return 0;
}