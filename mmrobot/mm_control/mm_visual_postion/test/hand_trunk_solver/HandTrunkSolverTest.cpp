#include "gtest/gtest.h"
#include "ros/ros.h"
#include "data/golden_data.h"
#include "mm_visual_postion/utils/utils.h"
#include "mm_visual_postion/hand_trunck/HandTrunkSolver.h"
#include <Eigen/Dense>
/*
TEST(HandTrunkSolverTest, findTrunk){
    HandTrunkSolverTestGoldenData golden_data(30);

    std::shared_ptr<StereoCameraArmModel> model_ptr = std::make_shared<StereoCameraArmModel>();
    model_ptr->loadDefaultParams();
    HandTrunkSolver solver(model_ptr);
    std::vector<cv::Mat> left_rectified_image_roi_vec, right_rectified_image_roi_vec;
    std::vector<cv::Rect> left_rect_roi_vec, right_rect_roi_vec;
    std::vector<Eigen::Matrix4d> transform_cam_to_base_vec;
    left_rectified_image_roi_vec.push_back(golden_data.left_image(golden_data.left_roi));
    right_rectified_image_roi_vec.push_back(golden_data.right_image(golden_data.right_roi));
    left_rect_roi_vec.push_back(golden_data.left_roi);
    right_rect_roi_vec.push_back(golden_data.right_roi);
    
    Eigen::Matrix4d T_random = Eigen::Matrix4d::Identity();
    T_random.block<3,3>(0,0) = Eigen::Matrix3d(Eigen::Quaterniond(0.3, 0.2, 0.1, 1.0).normalized());
    T_random.block<3,1>(0,3) << 345, 235, 65;
    transform_cam_to_base_vec.push_back(T_random);
           
    Circle3D result_circle; Eigen::Matrix4d T_base_to_trunk;
    
    bool flag = solver.findTrunk(left_rectified_image_roi_vec, right_rectified_image_roi_vec, left_rect_roi_vec, right_rect_roi_vec,
                      transform_cam_to_base_vec,
                      28, 3, 1000, 13, result_circle, T_base_to_trunk, &(golden_data.left_image));
    Eigen::Matrix4d T_cam_to_trunk = transform_cam_to_base_vec[0] * T_base_to_trunk;
    std::cout<<"T_cam_to_trunk: \n"<<T_cam_to_trunk<<std::endl;
    ASSERT_TRUE(flag);
    Eigen::Matrix4d T_compare = golden_data.T_cam_to_obj * T_cam_to_trunk.inverse();
    ASSERT_TRUE(isTransfromSimilaireToIdentity(T_compare, 5, 0.1));
}
*/
/*
TEST(HandTrunkSolverTest, findTrunkInRealCase){
    HandTrunkSolverRealTestGoldenData golden_data;

    std::shared_ptr<StereoCameraArmModel> model_ptr = std::make_shared<StereoCameraArmModel>();
    model_ptr->loadDefaultParams();
    HandTrunkSolver solver(model_ptr);
    std::vector<cv::Mat> left_rectified_image_roi_vec, right_rectified_image_roi_vec;
    std::vector<cv::Rect> left_rect_roi_vec, right_rect_roi_vec;
    std::vector<Eigen::Matrix4d> transform_cam_to_base_vec;
    left_rectified_image_roi_vec.push_back(golden_data.left_image(golden_data.left_roi));
    right_rectified_image_roi_vec.push_back(golden_data.right_image(golden_data.right_roi));
    left_rect_roi_vec.push_back(golden_data.left_roi);
    right_rect_roi_vec.push_back(golden_data.right_roi);
    
    Eigen::Matrix4d T_random = Eigen::Matrix4d::Identity();
    T_random.block<3,3>(0,0) = Eigen::Matrix3d(Eigen::Quaterniond(0.3, 0.2, 0.1, 1.0).normalized());
    T_random.block<3,1>(0,3) << 345, 235, 65;
    transform_cam_to_base_vec.push_back(T_random);
           
    Circle3D result_circle; Eigen::Matrix4d T_base_to_trunk;
    
    bool flag = solver.findTrunk(left_rectified_image_roi_vec, right_rectified_image_roi_vec, left_rect_roi_vec, right_rect_roi_vec,
                      transform_cam_to_base_vec,
                      20, 3, 1000, 13, result_circle, T_base_to_trunk, &(golden_data.left_image));
    Eigen::Matrix4d T_cam_to_trunk = transform_cam_to_base_vec[0] * T_base_to_trunk;
    std::cout<<"T_cam_to_trunk: \n"<<T_cam_to_trunk<<std::endl;
    ASSERT_TRUE(flag);
    Eigen::Matrix4d T_compare = golden_data.T_cam_to_obj * T_cam_to_trunk.inverse();
    ASSERT_TRUE(isTransfromSimilaireToIdentity(T_compare, 5, 0.1));
}
*/
TEST(HandTrunkSolverTest, findTrunkInRealCaseWithoutROI){
    HandTrunkSolverRealTestGoldenData golden_data(2);

    std::shared_ptr<StereoCameraArmModel> model_ptr = std::make_shared<StereoCameraArmModel>();
    model_ptr->loadDefaultParams();
    HandTrunkSolver solver(model_ptr);
    std::vector<cv::Mat> left_rectified_image_roi_vec, right_rectified_image_roi_vec;
    std::vector<cv::Rect> left_rect_roi_vec, right_rect_roi_vec;
    std::vector<Eigen::Matrix4d> transform_cam_to_base_vec;
    left_rectified_image_roi_vec.push_back(golden_data.left_image);
    right_rectified_image_roi_vec.push_back(golden_data.right_image);
    cv::Rect roi(0,0,golden_data.left_image.cols, golden_data.right_image.rows);
    left_rect_roi_vec.push_back(roi);
    right_rect_roi_vec.push_back(roi);
    
    Eigen::Matrix4d T_random = Eigen::Matrix4d::Identity();
    T_random.block<3,3>(0,0) = Eigen::Matrix3d(Eigen::Quaterniond(0.3, 0.2, 0.1, 1.0).normalized());
    T_random.block<3,1>(0,3) << 345, 235, 65;
    transform_cam_to_base_vec.push_back(T_random);
           
    Circle3D result_circle; Eigen::Matrix4d T_base_to_trunk;
    
    bool flag = solver.findTrunk(left_rectified_image_roi_vec, right_rectified_image_roi_vec, left_rect_roi_vec, right_rect_roi_vec,
                      transform_cam_to_base_vec,
                      20, 3, 1000, 13, result_circle, T_base_to_trunk, &(golden_data.left_image));
    Eigen::Matrix4d T_cam_to_trunk = transform_cam_to_base_vec[0] * T_base_to_trunk;
    std::cout<<"T_cam_to_trunk: \n"<<T_cam_to_trunk<<std::endl;
    ASSERT_TRUE(flag);
    //Eigen::Matrix4d T_compare = golden_data.T_cam_to_obj * T_cam_to_trunk.inverse();
    //ASSERT_TRUE(isTransfromSimilaireToIdentity(T_compare, 5, 0.1));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "handcart_solver_test_node");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
