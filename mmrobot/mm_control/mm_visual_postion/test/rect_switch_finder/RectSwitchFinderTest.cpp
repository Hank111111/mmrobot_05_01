#include "gtest/gtest.h"
#include "ros/ros.h"
#include "data/golden_data.h"
#include "mm_visual_postion/switch/RectSwitchFinder.h"
#include "mm_visual_postion/utils/utils.h"

#include <Eigen/Dense>
TEST(RectSwitchFinderTest, findAllSwitches){
    RectSwitchFinder finder;
    RectSwitchFinderTestGoldenData golden_data(0);
    finder.setSwitchParamsToRecognize("knifeSwitch", golden_data.width, golden_data.height, golden_data.T_cam_to_obj);
    
    std::shared_ptr<StereoCameraArmModel> model_ptr = std::make_shared<StereoCameraArmModel>();
    model_ptr->loadDefaultParams();
    finder.setCameraParams(model_ptr);
    std::vector<SwitchPtr> all_switches;
    cv::Mat left_roi_img, right_roi_img;
    left_roi_img = golden_data.left_image(golden_data.left_roi);
    right_roi_img = golden_data.right_image(golden_data.right_roi);
    finder.findAllSwitches(left_roi_img, right_roi_img, golden_data.left_roi, golden_data.right_roi, all_switches);
    ASSERT_EQ(all_switches.size(), 1);
    Eigen::Matrix4d T_compare = golden_data.T_cam_to_obj * all_switches[0]->T_cam_to_switch.inverse();
    ASSERT_TRUE(isTransfromSimilaireToIdentity(T_compare, 5, 0.1));
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rect_switch_finder_test_node");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
