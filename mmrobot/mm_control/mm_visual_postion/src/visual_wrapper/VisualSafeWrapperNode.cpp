#include "mm_visual_postion/visual_wrapper/VisualSafeWrapper.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "mm_visual_safe_wrapper");
	ros::NodeHandle nh;
    VisualSafeWrapper wrapper(nh, false);
    wrapper.run();
}