#include <iostream>
#include <fstream>
//#include <opencv2/opencv.hpp>
//#include<vector>
#include<sstream>
#include<string>
#include<math.h>
//#include"ZedCalib.h"
//#include"CoordinateCalculation.h"
//#include"eyeInhand.h"
#include"ros/ros.h"
#include "mm_robot_decision/pose4.h"


using namespace std;
//using namespace cv;

int main(int argc, char **argv)
{
	//cout<<"我们"<<endl;
	//CoordinateCalculation Correct1;
	//cv::Mat q1 = imread("Leftx8.bmp");
	//cv::Mat q2 = imread("开关模板2变x.bmp");
	//int nxusd = Correct1.Switch2StateRecognize(q1, q2);
	//cout << nxusd << endl;
       
        ros::init(argc, argv, "camera_coordinate_talker"); 
        ros::NodeHandle camera_n;
        ros::Publisher coordinate_pub = camera_n.advertise<mm_robot_decision::pose4>("visual_pose", 1000);

	/*Vec3f Angle1(-1.622, -0.007, -3.125);
	double b1[3][1] = { -287.5, -477.12, 777.73 };

	cv::Mat nxl = imread("/home/mrobot/桌面/ROS/vision2/Left5.bmp");
	cv::Mat nxr = imread("/home/mrobot/桌面/ROS/vision2/Right5.bmp");
	cv::Mat nx2 = imread("/home/mrobot/桌面/ROS/vision2/开关模板11x小.bmp");
	double *xyzabc = Correct1.Switch1Calculate(nxl, nxr, nx2, Angle1, b1);
	double *xyzabc_move=Correct1.Tmove(0,0,-100,xyzabc);
	/*for (int i = 0; i < 3; i++)
		cout << xyzabc[i] <<"mm"<< endl;
	for (int i = 3; i < 6; i++)
		cout << xyzabc[i] / 3.1415926 * 180 << "°" << endl;
	for (int i = 0; i < 3; i++)
		cout << xyzabc_move[i] <<"mm"<< endl;
	for (int i = 3; i < 6; i++)
		cout << xyzabc_move[i] / 3.1415926 * 180 << "°" << endl;*/

	//cv::Mat nx21 = imread("/home/mrobot/桌面/ROS/vision2/开关模板2x.bmp");
	//double *xyzabc1 = Correct1.Switch2Calculate(nxl, nxr, nx21, Angle1, b1);
	/*for (int i = 0; i < 3; i++)
		cout << xyzabc1[i] << "mm" << endl;
	for (int i = 3; i < 6; i++)
		cout << xyzabc1[i] / 3.1415926 * 180 << "°" << endl;*/
	//bool isT = Correct1.isSwitchRecognitionCorrect(xyzabc, xyzabc1);

        mm_robot_decision::pose4 move1_order;
        move1_order.state = "armGo";
        move1_order.wct = 1;
        //move1_order.wct = isT;
        //move_order.x = xyzabc_move[0]/1000; //单位：m
        //move_order.y = xyzabc_move[1]/1000;
        //move_order.z = xyzabc_move[2]/1000;
        //move_order.a = xyzabc_move[3];
        //move_order.b = xyzabc_move[4]; 
        //move_order.c = xyzabc_move[5];

        //测试直接输入

        move1_order.x = -3.83864/1000; //单位：m
        move1_order.y = -58.8739/1000;
        move1_order.z = 376.44/1000;
        move1_order.a = -0.0243569;
        move1_order.b = 0.0101524;
        move1_order.c = 0.11462;
        ROS_INFO("I start to publish!");
        sleep(1); //发布之前必须延时！！！否则消息丢失，无数据发布
        coordinate_pub.publish(move1_order);
        ROS_INFO("I published something!");



	//cout << isT << endl;
        
	//cout<<"我们"<<endl;
	//ZedCalib xxxxxx(10);
	//xxxxxx.MonCalib();
        //xxxxxx.LeftCalibrationEvaluate();
        //xxxxxx.RightCalibrationEvaluate();
	return 0;
}


