 // Standard includes
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"

#include "cameraZedUVC.h"
// OpenCV include (for display)
#include <opencv2/opencv.hpp>
#include "mm_visual_postion/utils/StereoImageReceiver.h"

void printHelp();

// Using std and sl namespaces
using namespace std;
int main(int argc, char **argv) {
    ros::NodeHandle n;
    //int i=0;
	ZedWrapper zed_wrapper;

    zed_wrapper.init();
    cv::Mat imageRight_cv, imageRight_cv_x, imageLeft_cv, imageLeft_cv_x;
    StereoImageReceiver processor(n);

    zed_wrapper.grab(imageLeft_cv, imageRight_cv);
    processor.rectifyLeftImage(imageLeft_cv, imageLeft_cv_x);
    processor.rectifyRightImage(imageRight_cv, imageRight_cv_x);
    // Display image with OpenCV
    cv::imshow("1",imageLeft_cv_x);
    cv::waitKey(30);
    //cv::imwrite("/home/youibot/mrobot/src/mm_control/mm_visual_postion/src/Lefttest.bmp", imageLeft_cv);
    //cv::imwrite("/home/youibot/mrobot/src/mm_control/mm_visual_postion/src/Righttest.bmp", imageRight_cv);
    cv::imwrite("/home/youibot/mrobot/src/mmrobot/mm_control/mm_pattern_recognition/src/meter_recognition/source/Lefttestx.bmp", imageLeft_cv_x);
    //cv::imwrite("/home/youibot/mrobot/src/mm_control/mm_visual_postion/src/Righttestx.bmp", imageRight_cv_x);

            

        

           

        return 0;
}



void printHelp() {
    cout << endl;
    cout << endl;
    cout << "Camera controls hotkeys: " << endl;
    cout << "  Increase camera settings value:            '+'" << endl;
    cout << "  Decrease camera settings value:            '-'" << endl;
    cout << "  Toggle camera settings:                    's'" << endl;
    cout << "  Reset all parameters:                      'r'" << endl;
    cout << endl;
    cout << "Exit : 'q'" << endl;
    cout << endl;
    cout << endl;
    cout << endl;
}
