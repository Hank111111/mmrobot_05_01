#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include<sstream>
#include<math.h>
#include"ros/ros.h"
#include "mm_robot_decision/pose4.h"
#include "mm_robot_decision/status.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "mm_endmotor_control/motorCommand.h"


using namespace std;

//定义主程序类
class mmRobotDecsion
{
public:
	int lineNum;            //当前执行命令的行数
	int numOfTotalLine;     //txt机器人命令总行数
	int transArriveSymbol;  //底盘到达标志
   	int armArriveSymbol;    //机械臂到达标志
    	int cameraPhotoSymbol;  //相机拍照标志
	int transNumOfTry;      //底盘失败尝试的次数
	int motorArriveSymbol;  //电机转动标志
	int takeToolSymbol;     //取工具动作的标志
	string toolInHandID;    //机械臂末端装配工具的编号

	//**第一次初始化**//
	void firstInitialization()
	{

		lineNum = 1;
		transNumOfTry = 0;
		toolInHandID = "none";
		
		numOfTotalLine = getNumberOfEdges();
	}

	//**初始化:每执行一行命令自动运行一遍**//
	void initialization()
	{
		
		transArriveSymbol = 0;
		armArriveSymbol = 0;
		cameraPhotoSymbol = 0;
		takeToolSymbol = 0;
		
		motorArriveSymbol = 0; 
		readMission();
	}


	//**发布和订阅函数**//
	void orderSubAndPub()
	{
		
		//底盘的位姿发布和订阅
		pubTransPose = decisionNode.advertise<geometry_msgs::PoseStamped>("/mm_trans/goal", 10);
		transArriveSub = decisionNode.subscribe("/mm_trans/isArrived", 10, &mmRobotDecsion::transCallback, this);
		//机械臂姿态的发布和订阅
		pubArmPose = decisionNode.advertise<mm_robot_decision::pose4>("/mm_arm/goal", 10);
		armArriveSub = decisionNode.subscribe("/mm_arm/isArrived", 20, &mmRobotDecsion::armCallback, this);
		//相机识别的发布和订阅
		pubRecogOrder = decisionNode.advertise<std_msgs::String>("/mm_recognition/visualRecogOrder", 10);                  //有修改，与相机拍照的识别程序统一话题名称
		equipIDResultSub = decisionNode.subscribe("/mm_recognition/numberResult", 20, &mmRobotDecsion::recogCallback, this);  //新增，与设备编号识别程序统一话题名称
		meterResultSub = decisionNode.subscribe("/mm_recognition/meterResult", 20, &mmRobotDecsion::meterCallback, this);  //新增，与仪表识别程序统一话题名称
		lightResultSub = decisionNode.subscribe("/mm_recognition/lightResult", 20, &mmRobotDecsion::lightCallback, this);//新增，与原来的指示灯识别程序统一话题名称
		remoteResultSub = decisionNode.subscribe("/mm_recognition/remoteResult", 20, &mmRobotDecsion::remoteCallback, this);//新增，与原来的指示灯识别程序统一话题名称
		//电机角度的发布和订阅
		pubMotorTheta = decisionNode.advertise<mm_endmotor_control::motorCommand>("/mm_motor/motor_command", 20);
		motorInfoSub = decisionNode.subscribe("/mm_motor/isArrived", 20, &mmRobotDecsion::endMotorCallback, this);
		//相机定位的发布和订阅
		pubCalcuOrder = decisionNode.advertise<std_msgs::String>("/mm_visiual/calcuPose", 20);
		switchPoseSub = decisionNode.subscribe("/mm_visual/switchPose", 20, &mmRobotDecsion::visualPoseCallback, this);
	
		//底盘充电指令的发布
		pubChargeMsg = decisionNode.advertise<std_msgs::Int8>("/is_charge", 10);
		initialization();
	
	}
	//**统计机器人指令的总行数**//
	int getNumberOfEdges()
	{
		char c;
		int edgeNum, count;
		count = 0;
		ifstream infile;
		//string file;
		char file[] = "C:\\Users\\Tina\\Desktop\\mission.txt";
		infile.open(file);
		while (infile.get(c))
		{
			if (c == '\n')
				count++;
		}

		edgeNum = count + 1;//因为最后一行没有换行符\n，所以需要在count补加1
		infile.close();
		cout << " the edge number is " << edgeNum << endl;
		return edgeNum;

	}


	//**按行读取读mission文件**//
	void readMission()
	{
		ifstream infile;
		//string file;
		char file[] = "C:\\Users\\Tina\\Desktop\\mission.txt";
		infile.open(file);
		char linetxt[1024];
		int numOfLine = 1;
		while (infile.getline(linetxt, 1024))
		{
			if (lineNum == numOfLine)
			{
				break;
			}
			numOfLine++;
		}
		cout << linetxt << endl;
		//lineNum = lineNum + 1;
		infile.close();
		
		//分割字符串
		char seg[] = ",";
		char charOrderList[50][50] = { "" };
		int i = 0;
		char *substr = strtok(linetxt, seg);
		while (substr != NULL)
		{
			strcpy(charOrderList[i], substr);
			i++;
			printf("%s\n", substr);
			substr = strtok(NULL, seg);
		}
		if (lineNum <= numOfTotalLine)
		{
			//判断第一个标识符，进入不同的子函数执行命令
			string robotOrder = charOrderList[0];
			if (robotOrder == "moveToEquipment")
			{
				moveToEquipment(charOrderList);
			}
			else if (robotOrder == "checkMeter")
			{
				checkMeter(charOrderList);
			}
			else if (robotOrder == "checkSwitchStatus")
			{
				checkSwitchStatus(charOrderList);
			}
			else if (robotOrder == "checkRemoteStatus")
			{
				checkRemoteStatus(charOrderList);
			}
			else if (robotOrder == "operateRemoteSwitch")
			{
				operateRemoteSwitch(charOrderList);
			}
			else if (robotOrder == "operateHandcartSwitch")
			{
				operateHandcartSwitch(charOrderList);
			}
			/*else if (robotOrder == "checkHandcartStatus")
			{
				checkHandcartStatus(charOrderList);
			}*/
			else if (robotOrder == "operateKnifeSwitch")
			{
				operateKnifeSwitch(charOrderList);
			}
			/*else if (robotOrder == "checkKnifeSwitch")
			{
				checkKnifeSwitch(charOrderList);
			}*/
			else if (robotOrder == "finishSingleEquipmentOperate")
			{
				finishSingleEquipmentOperate();
			}
		}
		else
		{
			ROS_INFO("Mission is over, go back to charge");
			std_msgs::Int8 chargeData;
			chargeData.data = 1;
			sleep(0.2);
			pubChargeMsg.publish(chargeData);
		}
		
		
		
	}


	//**将机器人命令中的字符转换为float型**//
	float *charToFloat(char a[50][50], int n)
	{


		int i = 0;
		char b[50];
		for (i = 0; i <= strlen(a[n]) - 3; i++)
		{
			b[i] = a[n][i + 1];
		}

		char seg[] = " ";
		char charToChar[50][50] = { "" };
		int j = 0;
		char *substr = strtok(b, seg);
		while (substr != NULL)
		{
			strcpy(charToChar[j], substr);
			j++;
			substr = strtok(NULL, seg);
		}
		static float returnFloat[10];
		for (int k = 0; k <= 9; k++)
		{
			returnFloat[k] = atof(charToChar[k]); //转换
			printf("%f\n", returnFloat[k]);
		}
		return  returnFloat;
	}



	//**读取每行中的指定字符串**//
	char* readMissionMsgs(int n)
	{
		ifstream infile;
		//string file;
		char file[] = "C:\\Users\\Tina\\Desktop\\mission.txt";
		infile.open(file);
		char linetxt[1024];
		int numOfLine = 1;
		while (infile.getline(linetxt, 1024))
		{
			if (lineNum == numOfLine)
			{
				break;
			}
			numOfLine++;
		}
		cout << linetxt << endl;
		//lineNum = lineNum + 1;
		infile.close();
		//分割字符串
		char seg[] = ",";
		static char charOrderList[50][50] = { "" };
		int i = 0;
		char *substr = strtok(linetxt, seg);
		while (substr != NULL)
		{
			strcpy(charOrderList[i], substr);
			i++;
			printf("%s\n", substr);
			substr = strtok(NULL, seg);
		}
		
		return charOrderList[n];

	}

	//**成功后往机器人指令表中写入success**//

	//**将char类型转换为string类型**//
	string CharToStr(char * contentChar)
	{
		string tempStr;
		for (int i = 0; contentChar[i] != '\0'; i++)
		{
			tempStr += contentChar[i];
		}
		return tempStr;
	}
	
	//**一行命令成功后往机器人指令表中写入success**//

	void WriteLineData()
	{
		//int lineNum = 2;
		ifstream infile;
		//string file;
		char file[] = "C:\\Users\\Tina\\Desktop\\mission.txt";
		infile.open(file);

		char linetxt[1024];
		int numOfLine = 1;
		while (infile.getline(linetxt, 1024))
		{
			if (lineNum == numOfLine)
			{
				break;
			}
			numOfLine++;
		}

		strcat(linetxt, "success");


		string strFileData = "";
		int line = 1;
		char tmpLineData[1024] = { 0 };
		while (infile.getline(tmpLineData, sizeof(tmpLineData)))
		{
			if (line == lineNum)
			{
				strFileData += CharToStr(linetxt);
				strFileData += "\n";
			}
			else
			{
				strFileData += CharToStr(tmpLineData);
				strFileData += "\n";
			}
			line++;
		}
		infile.close();

		//写入文件
		ofstream out;
		out.open(file);
		out.flush();
		out << strFileData;
		out.close();
	}

	//**执行命令失败，统计失败次数**//
	void tryAgain(int numOfTry)
	{
		if (numOfTry <= 3)
		{
			ROS_INFO("Accumulated failures are less than 3 times, please try again!");
			initialization();

		}
		else
		{
			ROS_INFO("Accumulated failures have more than 3 times, mission is failed!");
			exit(0);

		}
	}



	//******************与作业指令直接相关的函数**********************//

	//**移动到目标位置**//

	void moveToEquipment(char robotDecision[50][50])
	{
		//transarriveSymbol默认为0，底盘到达后收到确认信息后置为1
		if (transArriveSymbol == 0)
		{
			float *agvLocation;
			agvLocation = charToFloat(robotDecision, 1);
			float agvLocation1[3] = { *agvLocation, *(agvLocation + 1), *(agvLocation + 2) };
			float *agvPose;
			agvPose = charToFloat(robotDecision, 2);
			float agvPose1[4] = { *agvPose, *(agvPose + 1), *(agvPose + 2), *(agvPose + 3) };
			mm_trans(agvLocation1, agvPose1);
		}
		//armarriveSymbol默认为0，机械臂到达后收到确认信息后置为1
		else if (armArriveSymbol == 0)
		{
			float *robotJointTheta;
			robotJointTheta = charToFloat(robotDecision, 3);
			float robotJointTheta1[6] = { *robotJointTheta, *(robotJointTheta + 1), *(robotJointTheta + 2), *(robotJointTheta + 3), *(robotJointTheta + 4), *(robotJointTheta + 5) };
			char armState[] ="jointSpacePlanning";
			mm_arm(robotJointTheta1,armState);
		}
		//cameraPhotoSymbol默认为0，识别成功并对比成功后为1
		else if (cameraPhotoSymbol == 0)
		{
			string visualSymbol = "checkNumber";
			mm_recognition(visualSymbol);
		}
		//如果以上任务均已完成，则在mission.txt的对应行写上success!并进入下一行继续执行任务
		else if (transArriveSymbol == 1 && armArriveSymbol == 1 && cameraPhotoSymbol == 1)
		{
			WriteLineData();
			lineNum++;
			transNumOfTry = 0;
			initialization();
		}
		

	}

	//**检查仪表信息**//
	void checkMeter(char robotDecision[50][50])
	{
		if (armArriveSymbol == 0)
		{
			float *robotJointTheta;
			robotJointTheta = charToFloat(robotDecision, 3);
			float robotJointTheta1[6] = { *robotJointTheta, *(robotJointTheta + 1), *(robotJointTheta + 2), *(robotJointTheta + 3), *(robotJointTheta + 4), *(robotJointTheta + 5) };
			char armState[] = "jointSpacePlanning";
			mm_arm(robotJointTheta1, armState);
		}

		else if (cameraPhotoSymbol == 0)
		{
			string visualSymbol = "checkMeterNumber";
			mm_recognition(visualSymbol);
		}
		else if (armArriveSymbol == 1 && cameraPhotoSymbol == 1)
		{
			WriteLineData();
			lineNum++;
			initialization();
		}
	}

	//**检查指示灯状态**//
	void checkSwitchStatus(char robotDecision[50][50])
	{
		if (armArriveSymbol == 0)
		{
			float *robotJointTheta;
			robotJointTheta = charToFloat(robotDecision, 3);
			float robotJointTheta1[6] = { *robotJointTheta, *(robotJointTheta + 1), *(robotJointTheta + 2), *(robotJointTheta + 3), *(robotJointTheta + 4), *(robotJointTheta + 5) };
			char armState[] = "jointSpacePlanning";
			mm_arm(robotJointTheta1, armState);
		}
		else if (cameraPhotoSymbol == 0)
		{
			string visualSymbol = "recognizeLight";
			mm_recognition(visualSymbol);
		}
		else if (armArriveSymbol == 1 && cameraPhotoSymbol == 1)
		{
			WriteLineData();
			lineNum++;
			initialization();
		}
	}

	//**检查远方就地开关状态**//
	void checkRemoteStatus(char robotDecision[50][50])
	{
		if (armArriveSymbol == 0)
		{
			float *robotJointTheta;
			robotJointTheta = charToFloat(robotDecision, 3);
			float robotJointTheta1[6] = { *robotJointTheta, *(robotJointTheta + 1), *(robotJointTheta + 2), *(robotJointTheta + 3), *(robotJointTheta + 4), *(robotJointTheta + 5) };
			char armState[] = "jointSpacePlanning";
			mm_arm(robotJointTheta1, armState);
		}
		else if (cameraPhotoSymbol == 0)
		{
			string visualSymbol = "recognizeRemote";
			mm_recognition(visualSymbol);
		}
		else if (armArriveSymbol == 1 && cameraPhotoSymbol == 1)
		{
			WriteLineData();
			lineNum++;
			initialization();
		}
	}

	//**操作远方就地开关**//
	
	void operateRemoteSwitch(char robotDecision[50][50])
	{
		string tool = "t12";
		if (toolInHandID != "t12")
		{
			if (toolInHandID != "none")
			{
				
				putBackTool(toolInHandID);
			}
			else
			{
				takeTool(tool);
			}
		}
		else if (armArriveSymbol == 0)
		{
			float *robotJointTheta1;
			robotJointTheta1 = charToFloat(robotDecision, 3);
			float robotJointTheta11[6] = { *robotJointTheta1, *(robotJointTheta1 + 1), *(robotJointTheta1 + 2), *(robotJointTheta1 + 3), *(robotJointTheta1 + 4), *(robotJointTheta1 + 5) };
			char armState1[] = "jointSpacePlanning";
			mm_arm(robotJointTheta11, armState1);
		}
		else if (motorArriveSymbol == 0)
		{
			float motorTheta1 = 90;
			mm_end_motor(motorTheta1);
		}
		else if (cameraPhotoSymbol == 0)
		{
			string visualSymbol = "calcuSwitch1Pose1";  //远方就地开关目前将两个档位分为不同的开关进行处理mm_end_motor
			mm_visual_position(visualSymbol);
		}
		else if (motorArriveSymbol == 1)
		{
			char* s1Symbol0 = readMissionMsgs(2);
			string s1Symbol = s1Symbol0;
			float motorTheta2;
			if (s1Symbol=="open")
				motorTheta2 = -90;
			else if (s1Symbol == "close")
				motorTheta2 = 90;
			mm_end_motor(motorTheta2);
		}
		else if (armArriveSymbol == 1)
		{
			char armState2[] = "MoveInTool";
			float robotTheta2[6] = { 0, 0, -0.1, 0, 0, 0 };
			mm_arm(robotTheta2, armState2);
		}

		else
		{
			WriteLineData();
			lineNum++;
			initialization();
		}
	}
	//**操作手车开关**//
	void operateHandcartSwitch(char robotDecision[50][50])
	{

		/*
		取工具时，先判断是否末端有对应工具;
		若不是对应工具，则再判断有无工具，无工具则取工具库对应位置取
		若有其他工具，则先取再换
		*/
		string tool = "t3";
		if (toolInHandID != "t3")
		{

			if (toolInHandID != "none")
			{
				putBackTool(toolInHandID);
			}
			else
			{
				takeTool(tool);
			}
		
		}
		else if (armArriveSymbol == 0)
		{	
			float *robotJointTheta1;
			robotJointTheta1 = charToFloat(robotDecision, 3);
			float robotJointTheta11[6] = { *robotJointTheta1, *(robotJointTheta1 + 1), *(robotJointTheta1 + 2), *(robotJointTheta1 + 3), *(robotJointTheta1 + 4), *(robotJointTheta1 + 5) };
			char armState1[] = "jointSpacePlanning";
			mm_arm(robotJointTheta11, armState1);
		}
		else if (motorArriveSymbol == 0)
		{
			float motorTheta1 = 45;
			mm_end_motor(motorTheta1);
		}
		else if (cameraPhotoSymbol == 0)
		{
			string visualSymbol = "calcuSwitch3Pose"; //此处需要对应把visual里面的cpp文件的表示改掉 
			mm_visual_position(visualSymbol);
		}
		else if (armArriveSymbol == 1)
		{
			char armState2[] = "MoveInTool";
			float robotTheta2[6] = { 0, 0, -0.1, 0, 0, 0 };
			mm_arm(robotTheta2, armState2);
		}
		
		else
		{
			WriteLineData();
			lineNum++;
			initialization();
		}
	}

	//**操作分合闸开关**//

	void operateKnifeSwitch(char robotDecision[50][50])
	{
		string tool = "t12";
		if (toolInHandID != "t12")
		{
			if (toolInHandID != "none")
			{
				putBackTool(toolInHandID);
			}
			else
			{
				takeTool(tool);
			}
		}
		else if (armArriveSymbol == 0)
		{
			float *robotJointTheta1;
			robotJointTheta1 = charToFloat(robotDecision, 3);
			float robotJointTheta11[6] = { *robotJointTheta1, *(robotJointTheta1 + 1), *(robotJointTheta1 + 2), *(robotJointTheta1 + 3), *(robotJointTheta1 + 4), *(robotJointTheta1 + 5) };
			char armState1[] = "jointSpacePlanning";
			mm_arm(robotJointTheta11, armState1);
		}
		else if (motorArriveSymbol == 0)
		{
			float motorTheta1 = 90;
			mm_end_motor(motorTheta1);
		}
		else if (cameraPhotoSymbol == 0)
		{
			string visualSymbol = "calcuSwitch2Pose";  //远方就地开关目前将两个档位分为不同的开关进行处理
			mm_visual_position(visualSymbol);
		}
		else if (motorArriveSymbol == 1)
		{
			char* s1Symbol0 = readMissionMsgs(2);
			string s1Symbol = s1Symbol0;
			float motorTheta2;
			if (s1Symbol == "open")
				motorTheta2 = -45;
			else if (s1Symbol == "close")
				motorTheta2 = 45;
			mm_end_motor(motorTheta2);
		}
		else if (motorArriveSymbol == 2)
		{
			char* s1Symbol0 = readMissionMsgs(2);
			string s1Symbol = s1Symbol0;
			float motorTheta2;
			if (s1Symbol == "open")
				motorTheta2 = 45;
			else if (s1Symbol == "close")
				motorTheta2 = -45;
			mm_end_motor(motorTheta2);
		}

		else if (armArriveSymbol == 1)
		{
			char armState2[] = "MoveInTool";
			float robotTheta2[6] = { 0, 0, -0.1, 0, 0, 0 };
			mm_arm(robotTheta2, armState2);
		}

		else
		{
			WriteLineData();
			lineNum++;
			initialization();
		}
	}
	//**单个配电柜任务操作完毕**//
	void finishSingleEquipmentOperate()
	{
		if (toolInHandID != "none")
		{
			putBackTool(toolInHandID);
		}
		else if (armArriveSymbol == 0)
		{
			char armState1[] = "BackHome";
			float robotTheta1[6] = { 0, 0, 0, 0, 0, 0 };
			//robotReadySymbol = 1;
			mm_arm(robotTheta1, armState1);
		}
		else
		{
			WriteLineData();
			lineNum++;
			initialization();

		}
	}




	//******************中间层级的函数：更换工具**********************//

	//***取工具***//

	void takeTool(const std::string str)
	{
		if (str == "t12")
		{
			if (armArriveSymbol == 0)
			{

				{
					char armState1[] = "StandBy2";
					float robotTheta1[6] = { 0, 0, 0, 0, 0, 0 };
					//robotReadySymbol = 1;
					mm_arm(robotTheta1, armState1);
				}
			
			}
			//此处还欠缺一个读取电机的角度并将电机转动到参考位置的函数

			else if (armArriveSymbol == 1)
			{
				char armState2[] = "MoveInTool";
				float robotTheta2[6] = { 0, 0, 0.1, 0, 0, 0 };
				mm_arm(robotTheta2, armState2);
			}

			else if (motorArriveSymbol == 0)
			{
				float motorTheta = -45.0;

				mm_end_motor(motorTheta);
			}
			else if (armArriveSymbol == 2)
			{
				char armState3[] = "MoveInTool";
				float robotTheta3[6] = { 0, 0, -0.1, 0, 0, 0 };
				mm_arm(robotTheta3, armState3);

			}
			else
			{
				armArriveSymbol = 0;
				motorArriveSymbol = 0;
				takeToolSymbol = 1;
				toolInHandID = "t12";
				readMission();
			}
				
		}
		else if (str == "t3")
		{
			if (armArriveSymbol == 0)
			{
				char armState1[] = "StandBy3";
				float robotTheta1[6] = { 0, 0, 0, 0, 0, 0 };
				//robotReadySymbol = 1;
				mm_arm(robotTheta1, armState1);
			}

			//此处还欠缺一个读取电机的角度并将电机转动到参考位置的函数

			else if (armArriveSymbol == 1)
			{
				char armState2[] = "MoveInTool";
				float robotTheta2[6] = { 0, 0, 0.1, 0, 0, 0 };
				mm_arm(robotTheta2, armState2);
			}

			else if (motorArriveSymbol == 0)
			{
				float motorTheta = -45.0;

				mm_end_motor(motorTheta);
			}
			else if (armArriveSymbol == 2)
			{
				char armState3[] = "MoveInTool";
				float robotTheta3[6] = { 0, 0, -0.1, 0, 0, 0 };
				mm_arm(robotTheta3, armState3);

			}
			else
			{
				armArriveSymbol = 0;
				motorArriveSymbol = 0;
				takeToolSymbol = 1;
				toolInHandID = "t3";
				readMission();
				
			}
			
		}
		
	}

	//***放回工具***//
	void putBackTool(const std::string str)
	{
		if (str == "t12")
		{
			if (armArriveSymbol == 0)
			{

				{
					char armState1[] = "StandBy2";
					float robotTheta1[6] = { 0, 0, 0, 0, 0, 0 };
					//robotReadySymbol = 1;
					mm_arm(robotTheta1, armState1);
				}

			}
			//此处还欠缺一个读取电机的角度并将电机转动到参考位置的函数

			else if (armArriveSymbol == 1)
			{
				char armState2[] = "MoveInTool";
				float robotTheta2[6] = { 0, 0, 0.1, 0, 0, 0 };
				mm_arm(robotTheta2, armState2);
			}

			else if (motorArriveSymbol == 0)
			{
				float motorTheta = 45.0;

				mm_end_motor(motorTheta);
			}
			else if (armArriveSymbol == 2)
			{
				char armState3[] = "MoveInTool";
				float robotTheta3[6] = { 0, 0, -0.1, 0, 0, 0 };
				mm_arm(robotTheta3, armState3);

			}
			else
			{
				armArriveSymbol = 0;
				motorArriveSymbol = 0;
				toolInHandID = "none";
				readMission();
			}
		}
		else if (str == "t3")
		{
			if (armArriveSymbol == 0)
			{

				{
					char armState1[] = "StandBy3";
					float robotTheta1[6] = { 0, 0, 0, 0, 0, 0 };
					//robotReadySymbol = 1;
					mm_arm(robotTheta1, armState1);
				}

			}
			//此处还欠缺一个读取电机的角度并将电机转动到参考位置的函数

			else if (armArriveSymbol == 1)
			{
				char armState2[] = "MoveInTool";
				float robotTheta2[6] = { 0, 0, 0.1, 0, 0, 0 };
				mm_arm(robotTheta2, armState2);
			}

			else if (motorArriveSymbol == 0)
			{
				float motorTheta = 45.0;

				mm_end_motor(motorTheta);
			}
			else if (armArriveSymbol == 2)
			{
				char armState3[] = "MoveInTool";
				float robotTheta3[6] = { 0, 0, -0.1, 0, 0, 0 };
				mm_arm(robotTheta3, armState3);

			}
			else
			{
				armArriveSymbol = 0;
				motorArriveSymbol = 0;
				toolInHandID = "none";
				readMission();
			}
		}
	}


	



	//******************与机器人底层相关的函数，包括底盘、机械臂、视觉系统、电机、力控等。**********************//
	
	//**底盘位姿发布函数**//
	void mm_trans(float location[3], float pose[4])
	{
		geometry_msgs::PoseStamped transGoal;
		transGoal.header.frame_id = "trans";
		transGoal.pose.position.x = location[0];
		transGoal.pose.position.y = location[1];
		transGoal.pose.position.z = location[2];
		transGoal.pose.orientation.x = pose[0];
		transGoal.pose.orientation.y = pose[1];
		transGoal.pose.orientation.z = pose[2];
		transGoal.pose.orientation.w = pose[3];
		sleep(0.2);
		pubTransPose.publish(transGoal);
		cout<<transGoal<<endl;
	}
	//**底盘回调函数**//
	void transCallback(const std_msgs::Int8::ConstPtr& msg)
	{
		if (msg->data == 1)
		{
			ROS_INFO("Trans is arrived,please check the location");
			transArriveSymbol = 1;
			readMission();
		}
		else
		{
			ROS_INFO("ERROR01:Trans is not arrived!");
			exit(0);
		}
	}

	//**机械臂位姿发布函数**//
	void mm_arm(float robotTheta[6], char robotState[])
	{
		mm_robot_decision::pose4 armGoal;
		armGoal.state = robotState;
		armGoal.x = robotTheta[0];
		armGoal.y = robotTheta[1];
		armGoal.z = robotTheta[2];
		armGoal.a = robotTheta[3];
		armGoal.b = robotTheta[4];
		armGoal.c = robotTheta[5];
		cout << armGoal << endl;
		sleep(0.2);
		pubArmPose.publish(armGoal);
	}

	//**机械臂到位回调函数**//
	void armCallback(const std_msgs::Int8::ConstPtr& msg)
	{
		if (msg->data == 1)
		{
			ROS_INFO("Arm is arrived,please check the id of equipment");
			armArriveSymbol++;
			readMission();
		}
		else
		{
			ROS_INFO("ERROR02:Arm is not arrived!");
			exit(0);
		}
	}


	//**相机识别编号发布函数**//
	void mm_recognition(const std::string str)
	{
		std_msgs::String recogOrder;
		recogOrder.data = str;
		cout << recogOrder << endl;
		sleep(0.2);
		pubRecogOrder.publish(recogOrder);

	}

	//**相机识别配电设备编号回调函数**//
	void recogCallback(const std_msgs::Int8::ConstPtr& msg)
	{
		char* equipmentID0 = readMissionMsgs(4);
		int equipmentID = atoi(equipmentID0);
		if (msg->data == equipmentID)
		{
			ROS_INFO("The robot is in correct place,Please go on!");
			cameraPhotoSymbol = 1;
			readMission();
		}
		else
		{
			ROS_INFO("Warning:The robot has gone wrong place, please try again");
			transNumOfTry++;
			tryAgain(transNumOfTry);

		}
	}
	/**相机识别仪表读数回调函数**/
	//此处代码待完善：定义一个新的消息类型，传递出来
	void meterCallback(const mm_robot_decision::status::ConstPtr& msg)
	{
		char* VMeterNumber00 = readMissionMsgs(3);
		int VMeterNumber0 = atoi(VMeterNumber00);
		bool VMeterNumber;
		bool AMeterNumber;
		if (VMeterNumber0 != 0)
		{
			VMeterNumber = 1;
		}
		else
		{
			VMeterNumber = 0;
		}

		char* AMeterNumber00 = readMissionMsgs(5);
		int AMeterNumber0 = atoi(AMeterNumber00);
		if (VMeterNumber0 != 0)
		{
			AMeterNumber = 1;
		}
		else
		{
			AMeterNumber = 0;
		}

		if (msg->v == VMeterNumber && msg->a == AMeterNumber)
		{
			ROS_INFO("The meter reading is  correct,Please go on!");
			cameraPhotoSymbol = 1;
			readMission();
		}
		else
		{
			ROS_INFO("ERROR04:meter reading is wrong");
			exit(0);

		}
	}

	//**相机识别指示灯回调函数**//
	void lightCallback(const mm_robot_decision::status msg)
	{   
		char* s2Status00 = readMissionMsgs(3);
		int s2Status0 = atoi(s2Status00);
		char* s3Status00 = readMissionMsgs(5);
		int s3Status0 = atoi(s3Status00);
		bool s2Status;
		bool s3Status;

		if (s2Status0 != 0)
		{
			s2Status = 1;
		}
		else
		{
			s2Status = 0;
		}

		if (s3Status0 != 0)
		{
			s3Status = 1;
		}
		else
		{
			s3Status = 0;
		}

		if (msg.s2 == s2Status&&msg.s3 == s3Status)
		{
			ROS_INFO("The  switch status is  correct,Please go on!");
			cameraPhotoSymbol = 1;
			readMission();
		}
		else
		{
			ROS_INFO("ERROR05: The switch status is wrong");
			exit(0);
		}

	}


	//**相机识别远方就地开关回调函数**//
	void remoteCallback(const mm_robot_decision::status msg)
	{
		char* s1Status00 = readMissionMsgs(3);
		int s1Status0 = atoi(s1Status00);
		bool s1Status;
		if (s1Status0 != 0)
		{
			s1Status = 1;
		}
		else
		{
			s1Status = 0;
		}
		if (msg.s1 == s1Status)
		{
			ROS_INFO("The  remote switch status is  correct,Please go on!");
			cameraPhotoSymbol = 1;
			readMission();
		}
		else
		{
			ROS_INFO("ERROR05: The remote switch status is wrong");
			exit(0);
		}

	}
	//**电机角度发布函数**//
	void mm_end_motor(float theta)
	{
		mm_endmotor_control::motorCommand motorTheta;
		motorTheta.degree = theta;
		sleep(0.2);
		pubMotorTheta.publish(motorTheta);
		cout << motorTheta << endl;
		
	}
	//**电机到位回调函数**//
	void endMotorCallback(const std_msgs::Int8 msg)
	{
		if (msg.data == 1)
		{  	motorArriveSymbol++;
			//motorRotateS1SymbolS1Before = 1;
			//motorRotateS1SymbolS1After = 1;
			readMission();
		}
		else
		{
			ROS_INFO("ERROR06: Motor rotation is abnormal,please check");
			exit(0);
		}
	}

	//**视觉定位发布函数**//
	void mm_visual_position(const std::string str)
	{
		
		std_msgs::String calcuOrder;
		calcuOrder.data = str;
		sleep(0.2);
		pubRecogOrder.publish(calcuOrder);
		cout << calcuOrder << endl;
	}

	//**视觉定位回调函数**//
	void visualPoseCallback(const mm_robot_decision::pose4 msg)
	{
		cameraPhotoSymbol = 1;
		sleep(0.2);
		pubArmPose.publish(msg);
		cout << msg << endl;
	}


private:
	ros::NodeHandle decisionNode;
    
	ros::Publisher pubTransPose;
	ros::Subscriber transArriveSub;

	ros::Publisher pubArmPose;
	ros::Subscriber armArriveSub;
	
	ros::Publisher pubRecogOrder;
	ros::Subscriber equipIDResultSub;
	ros::Subscriber meterResultSub;
	ros::Subscriber lightResultSub;
	ros::Subscriber remoteResultSub;

	ros::Publisher pubMotorTheta;
	ros::Subscriber motorInfoSub;
	
	ros::Publisher pubCalcuOrder;
	ros::Subscriber switchPoseSub;
	
	ros::Publisher pubChargeMsg;  
	


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mm_decision");
	mmRobotDecsion robotDecision;
	sleep(5);
	robotDecision.firstInitialization();
	robotDecision.orderSubAndPub();
	return 0;
}
