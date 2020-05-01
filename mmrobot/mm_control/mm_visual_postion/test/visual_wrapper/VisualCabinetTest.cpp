#include "gtest/gtest.h"
#include "ros/ros.h"
#include "mm_visual_postion/visual_wrapper/VisualCabinet.h"
#include "mm_visual_postion/test/GoldenDataFoVisualCabinet.h"
#include <random>
#include <opencv2/core/eigen.hpp>


TEST(VisualCabinetTest, RetriveCabinetStandardParams){
    VisualCabinet visual_cabinet(0,0);
    visual_cabinet.retrieveCabinetStandardParams();
    // golden data
    VisualCabinet golden_visual_cabinet = createGoldenDataForStandardVisualCabinet();
    ASSERT_EQ(visual_cabinet, golden_visual_cabinet) <<"visual_cabinet doesn't equals to golden data";
}

TEST(VisualCabinetTest, UpdateTransfromBaseToCabinet){
    VisualCabinet visual_cabinet(0,0);
    visual_cabinet.retrieveCabinetStandardParams();
    double roll = 0.5;//rad
    double pitch = 0.2; //rad
    double yaw = 0.1; // rad
    double x_dist = 100; //mm
    double y_dist = 10; //mm
    double z_dist = 10; //mm
    
    VisualObject obj;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> pos_dis(-2.0, 2.0);
    std::uniform_real_distribution<> angle_dis(-0.001, 0.001);
    std::vector<VisualObject> visual_objs_in_base_vec;

    int add_count = 0;
    for(auto iter=visual_cabinet.visual_objs_in_cabinet_map.begin(); 
            iter != visual_cabinet.visual_objs_in_cabinet_map.end(); iter++){
      Eigen::Matrix4d T_base_to_cabinet_test;
      getTransformMatrix(Eigen::Vector3d(x_dist + pos_dis(gen), y_dist + pos_dis(gen), z_dist + pos_dis(gen)),
                        Eigen::Quaterniond(Eigen::AngleAxisd(roll + angle_dis(gen), Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(pitch + angle_dis(gen), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(yaw + angle_dis(gen), Eigen::Vector3d::UnitZ())), T_base_to_cabinet_test);
      Eigen::Matrix4d T_cabinet_to_obj, T_base_to_obj;
      obj = iter->second;
      obj.getTransformFromPresentedFrameToObj(T_cabinet_to_obj);
      T_base_to_obj = T_base_to_cabinet_test * T_cabinet_to_obj;
      obj.setTransformFromPresentedFrameToObj(T_base_to_obj);
      obj.presented_frame = VOBJ_BASE_FRAME;
      visual_objs_in_base_vec.push_back(obj);
      add_count ++;
      if(add_count >=2) break;
    }
    visual_cabinet.updateTransformBaseToCabinet(visual_objs_in_base_vec);
    ASSERT_TRUE(visual_cabinet.isInitialized());
    Eigen::Vector3d euler = visual_cabinet.T_base_to_cabinet.block<3,3>(0,0).eulerAngles(0,1,2);
    ASSERT_TRUE(fabs(x_dist - visual_cabinet.T_base_to_cabinet(0,3)) < 2.0);
    ASSERT_TRUE(fabs(y_dist - visual_cabinet.T_base_to_cabinet(1,3)) < 2.0);
    ASSERT_TRUE(fabs(z_dist - visual_cabinet.T_base_to_cabinet(2,3)) < 2.0);
    ASSERT_TRUE(fabs(roll - euler(0)) < 0.001);
    ASSERT_TRUE(fabs(pitch - euler(1)) < 0.001);
    ASSERT_TRUE(fabs(yaw - euler(2)) < 0.001);
}
/* 
TEST(VisualCabinetTest, updateStatus){
  VisualCabinet visual_cabinet(0,0);
  visual_cabinet.retrieveCabinetStandardParams();
  std::vector<signed char> status;
  status.push_back(1);
  status.push_back(0);
  status.push_back(0);
  status.push_back(1);
  std::string cmd = LIGHTS_NAME;
  visual_cabinet.updateStatus(cmd, status);
  
  VisualObject light_obj = visual_cabinet.visual_objs_in_cabinet_map.at(cmd);
  ASSERT_TRUE(light_obj.status.size() == status.size());
  ASSERT_TRUE(status == light_obj.status);
}

TEST(VisualCabinetTest, convertVisualObjBaseToCabinetAndconvertVisualObjCabinetToBase){
  VisualCabinet visual_cabinet = createInitializedVisualCabinet();
  VisualObject visual_object_in_cabinet = (visual_cabinet.visual_objs_in_cabinet_map.begin()->second);
  VisualObject visual_object_in_base;

  visual_cabinet.convertVisualObjCabinetToBase(visual_object_in_cabinet, visual_object_in_base);
  ASSERT_TRUE(visual_object_in_base.presented_frame == VOBJ_BASE_FRAME);
  
  
  Eigen::Matrix4d T_object_to_cabinet, T_object_to_base;
  visual_object_in_base.getTransformFromObjToPresentedFrame(T_object_to_base);
  visual_object_in_cabinet.getTransformFromObjToPresentedFrame(T_object_to_cabinet);
  Eigen::Matrix4d T_base_to_cabinet = T_object_to_base.inverse() * T_object_to_cabinet;
  ASSERT_TRUE( (T_base_to_cabinet - visual_cabinet.T_base_to_cabinet).norm() < 1e-8);

  VisualObject visual_object_in_cabinet_1;
  visual_cabinet.convertVisualObjBaseToCabinet(visual_object_in_base,visual_object_in_cabinet_1);
  ASSERT_TRUE(visual_object_in_cabinet_1.presented_frame == VOBJ_CABINET_FRAME);
  Eigen::Matrix4d T_object_in_cabinet_1;
  visual_object_in_cabinet_1.getTransformFromObjToPresentedFrame(T_object_in_cabinet_1);
  ASSERT_TRUE( (T_object_to_cabinet - T_object_in_cabinet_1).norm() < 1e-8);

}



bool rectEqual(cv::Rect& rect_0, cv::Rect& rect_1, double epsilon){
  if(fabs(rect_0.width - rect_1.width) < epsilon 
      && fabs(rect_0.height - rect_1.height) < epsilon
      && fabs(rect_0.x - rect_1.x) < epsilon
      && fabs(rect_0.y - rect_1.y) < epsilon)
    return true;
  else return false;
}


TEST(VisualCabinetTest, getROI){
  VisualCabinet visual_cabinet = createInitializedVisualCabinet();
  Eigen::Matrix4d T_endeffector_to_base;
  // make left_roi be the center of left image;
  cv::Point2d point;

  // suppose the img points
  cv::Rect supposed_left_rect(1200,600,100,100);
  std::vector<cv::Point2d> left_img_points;
  getPointsFromRect(supposed_left_rect, left_img_points);
  
  std::vector<cv::Point3d> obj_points;
  cv::Point3d obj_point;
  double width = visual_cabinet[REMOTE_SWITCH_NAME].width;
  double height = visual_cabinet[REMOTE_SWITCH_NAME].width;

  obj_point.x = -width/2.0; obj_point.y = -height/2.0; obj_point.z = 0;
  obj_points.push_back(obj_point);
  obj_point.x = width/2.0; obj_point.y = -height/2.0; obj_point.z = 0;
  obj_points.push_back(obj_point);
  obj_point.x = width/2.0; obj_point.y = height/2.0; obj_point.z = 0;
  obj_points.push_back(obj_point);
  obj_point.x = -width/2.0; obj_point.y = height/2.0; obj_point.z = 0;
  obj_points.push_back(obj_point);
  
  cv::Mat rvec, tvec; 
  cv::Mat cv_transform_obj_to_cam;
  Eigen::Matrix4d T_obj_to_cam;
  
  bool ret = solvePnP(obj_points, left_img_points, visual_cabinet.model_ptr->left_camera.intrinsic_mat, std::vector<double>(), rvec, tvec);
  ASSERT_TRUE(ret);
  RVecTVecToHomogenousMatrix(rvec, tvec, cv_transform_obj_to_cam);  
  cv2eigen(cv_transform_obj_to_cam, T_obj_to_cam);

  // compute supposed right roi
  Eigen::Vector4d left_up_point_in_obj;
  left_up_point_in_obj << -width/2.0, -height/2.0, 0,1.0;
  Eigen::Vector3d left_up_point = visual_cabinet.model_ptr->right_camera.projection_mat * T_obj_to_cam.inverse() * left_up_point_in_obj;
  normalizeVector3d(left_up_point);

  Eigen::Vector4d right_down_point_in_obj;
  right_down_point_in_obj << width/2.0, height/2.0, 0,1.0;
  Eigen::Vector3d right_down_point = visual_cabinet.model_ptr->right_camera.projection_mat * T_obj_to_cam.inverse() * right_down_point_in_obj;
  normalizeVector3d(right_down_point);

  cv::Rect supposed_right_rect(cv::Point(left_up_point(0), left_up_point(1)), cv::Point(right_down_point(0), right_down_point(1)));

  Eigen::Matrix4d T_cabinet_to_obj;
  visual_cabinet[REMOTE_SWITCH_NAME].getTransformFromPresentedFrameToObj(T_cabinet_to_obj);
  T_endeffector_to_base = 
                        visual_cabinet.model_ptr->endeffector_to_cam_transform
                        * T_obj_to_cam.inverse() * T_cabinet_to_obj.inverse() * visual_cabinet.T_base_to_cabinet.inverse();
  
  cv::Rect left_roi, right_roi;
  std::string obj_name = REMOTE_SWITCH_NAME;
  bool success = visual_cabinet.getROI(obj_name, T_endeffector_to_base, left_roi, right_roi, false);

  ASSERT_TRUE(success);
  ASSERT_TRUE(rectEqual(left_roi, supposed_left_rect, 5)) << "left roi is wrong";
  ASSERT_TRUE(rectEqual(right_roi, supposed_right_rect, 5)) << "right roi is wrong";
}

*/
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "visual_cabinet_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
