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

//������������
class mmRobotDecsion
{
public:
	int lineNum;            //��ǰִ�����������
	int numOfTotalLine;     //txt����������������
	int transArriveSymbol;  //���̵����־
   	int armArriveSymbol;    //��е�۵����־
    	int cameraPhotoSymbol;  //������ձ�־
	int transNumOfTry;      //����ʧ�ܳ��ԵĴ���
	int motorArriveSymbol;  //���ת����־
	int takeToolSymbol;     //ȡ���߶����ı�־
	string toolInHandID;    //��е��ĩ��װ�乤�ߵı��

	//**��һ�γ�ʼ��**//
	void firstInitialization()
	{

		lineNum = 1;
		transNumOfTry = 0;
		toolInHandID = "none";
		
		numOfTotalLine = getNumberOfEdges();
	}

	//**��ʼ��:ÿִ��һ�������Զ�����һ��**//
	void initialization()
	{
		
		transArriveSymbol = 0;
		armArriveSymbol = 0;
		cameraPhotoSymbol = 0;
		takeToolSymbol = 0;
		
		motorArriveSymbol = 0; 
		readMission();
	}


	//**�����Ͷ��ĺ���**//
	void orderSubAndPub()
	{
		
		//���̵�λ�˷����Ͷ���
		pubTransPose = decisionNode.advertise<geometry_msgs::PoseStamped>("/mm_trans/goal", 10);
		transArriveSub = decisionNode.subscribe("/mm_trans/isArrived", 8, &mmRobotDecsion::transCallback, this);
		//��е����̬�ķ����Ͷ���
		pubArmPose = decisionNode.advertise<mm_robot_decision::pose4>("/mm_arm/goal", 10);
		armArriveSub = decisionNode.subscribe("/mm_arm/isArrived", 20, &mmRobotDecsion::armCallback, this);
		//���ʶ��ķ����Ͷ���
		pubRecogOrder = decisionNode.advertise<std_msgs::String>("/mm_recognition/visualRecogOrder", 10);                  //���޸ģ���������յ�ʶ�����ͳһ��������
		equipIDResultSub = decisionNode.subscribe("/mm_recognition/numberResult", 20, &mmRobotDecsion::recogCallback, this);  //���������豸���ʶ�����ͳһ��������
		meterResultSub = decisionNode.subscribe("/mm_recognition/meterResult", 20, &mmRobotDecsion::meterCallback, this);  //���������Ǳ�ʶ�����ͳһ��������
		lightResultSub = decisionNode.subscribe("/mm_recognition/lightResult", 20, &mmRobotDecsion::lightCallback, this);//��������ԭ����ָʾ��ʶ�����ͳһ��������
		remoteResultSub = decisionNode.subscribe("/mm_recognition/remoteResult", 20, &mmRobotDecsion::remoteCallback, this);//��������ԭ����ָʾ��ʶ�����ͳһ��������
		//����Ƕȵķ����Ͷ���
		pubMotorTheta = decisionNode.advertise<mm_endmotor_control::motorCommand>("/mm_motor/motor_command", 20);
		motorInfoSub = decisionNode.subscribe("/mm_motor/isArrived", 20, &mmRobotDecsion::endMotorCallback, this);
		//�����λ�ķ����Ͷ���
		pubCalcuOrder = decisionNode.advertise<std_msgs::String>("/mm_visiual/calcuPose", 20);
		switchPoseSub = decisionNode.subscribe("/mm_visual/switchPose", 20, &mmRobotDecsion::visualPoseCallback, this);
	
		//���̳��ָ��ķ���
		pubChargeMsg = decisionNode.advertise<std_msgs::Int8>("/is_charge", 10);
		initialization();
	
	}
	//**ͳ�ƻ�����ָ���������**//
	int getNumberOfEdges()
	{
		char c;
		int edgeNum, count;
		count = 0;
		ifstream infile;
		//string file;
		char file[] = "/home/youibot/mrobot/src/mmrobot/mm_control/mm_robot_decision/src/mission.txt";
		infile.open(file);
		while (infile.get(c))
		{
			if (c == '\n')
				count++;
		}

		edgeNum = count;//��Ϊ���һ��û�л��з�\n��������Ҫ��count����1
		infile.close();
		cout << " the totoal line is " << edgeNum << endl;
		return edgeNum;

	}


	//**���ж�ȡ��mission�ļ�**//
	void readMission()
	{
		ifstream infile;
		//string file;
		char file[] =  "/home/youibot/mrobot/src/mmrobot/mm_control/mm_robot_decision/src/mission.txt";
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
		
		//�ָ��ַ���
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
			//�жϵ�һ����ʶ�������벻ͬ���Ӻ���ִ������
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


	//**�������������е��ַ�ת��Ϊfloat��**//
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
			returnFloat[k] = atof(charToChar[k]); //ת��
			//printf("%f\n", returnFloat[k]);
		}
		return  returnFloat;
	}



	//**��ȡÿ���е�ָ���ַ���**//
	char* readMissionMsgs(int n)
	{
		ifstream infile;
		//string file;
		char file[] =  "/home/youibot/mrobot/src/mmrobot/mm_control/mm_robot_decision/src/mission.txt";
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
		//�ָ��ַ���
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

	//**�ɹ�����������ָ�����д��success**//

	//**��char����ת��Ϊstring����**//
	string CharToStr(char * contentChar)
	{
		string tempStr;
		for (int i = 0; contentChar[i] != '\0'; i++)
		{
			tempStr += contentChar[i];
		}
		return tempStr;
	}
	
	//**һ������ɹ�����������ָ�����д��success**//

	void WriteLineData()
	{
		//int lineNum = 2;
		ifstream infile;
		//string file;
		char file[] =  "/home/youibot/mrobot/src/mmrobot/mm_control/mm_robot_decision/src/mission.txt";
		//char file1[] =  "/home/youibot/mrobot/src/mmrobot/mm_control/mm_robot_decision/src/mission1.txt";
		infile.open(file);

		char linetxt[1024]={0};
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
		//cout<<linetxt<<endl;
		infile.close();

		ifstream infile1;
                infile1.open(file);

		string strFileData = "";
		int line = 1;
		char tmpLineData[1024] = { 0 };
		while (infile1.getline(tmpLineData, sizeof(tmpLineData)))
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
			cout<<strFileData<<endl;
		}
		infile1.close();
		//д���ļ�
		ofstream out;
		out.open(file);
		out.flush();
		out<<strFileData;
                
		
		out.close();
		
	}

	//**ִ������ʧ�ܣ�ͳ��ʧ�ܴ���**//
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



	//******************����ҵָ��ֱ����صĺ���**********************//

	//**�ƶ���Ŀ��λ��**//

	void moveToEquipment(char robotDecision[50][50])
	{
		//transarriveSymbolĬ��Ϊ0�����̵�����յ�ȷ����Ϣ����Ϊ1
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
		//armarriveSymbolĬ��Ϊ0����е�۵�����յ�ȷ����Ϣ����Ϊ1
		/*else if (armArriveSymbol == 0)
		{
			float *robotJointTheta;
			robotJointTheta = charToFloat(robotDecision, 3);
			float robotJointTheta1[6] = { *robotJointTheta, *(robotJointTheta + 1), *(robotJointTheta + 2), *(robotJointTheta + 3), *(robotJointTheta + 4), *(robotJointTheta + 5) };
			char armState[] ="jointSpacePlanning";
			mm_arm(robotJointTheta1,armState);
		}
		//cameraPhotoSymbolĬ��Ϊ0��ʶ��ɹ����Աȳɹ���Ϊ1
		else if (cameraPhotoSymbol == 0)
		{
			string visualSymbol = "checkNumber";
			mm_recognition(visualSymbol);
		}*/
		//����������������ɣ�����mission.txt�Ķ�Ӧ��д��success!��������һ�м���ִ������
		else 
		{
			WriteLineData();
			lineNum++;
			transNumOfTry = 0;
			initialization();
			
		}
		

	}

	//**����Ǳ���Ϣ**//
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

	//**���ָʾ��״̬**//
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

	//**���Զ���͵ؿ���״̬**//
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

	//**����Զ���͵ؿ���**//
	
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
			string visualSymbol = "calcuSwitch1Pose1";  //Զ���͵ؿ���Ŀǰ��������λ��Ϊ��ͬ�Ŀ��ؽ��д���mm_end_motor
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
	//**�����ֳ�����**//
	void operateHandcartSwitch(char robotDecision[50][50])
	{

		/*
		ȡ����ʱ�����ж��Ƿ�ĩ���ж�Ӧ����;
		���Ƕ�Ӧ���ߣ������ж����޹��ߣ��޹�����ȡ���߿��Ӧλ��ȡ
		�����������ߣ�����ȡ�ٻ�
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
			string visualSymbol = "calcuSwitch3Pose"; //�˴���Ҫ��Ӧ��visual�����cpp�ļ��ı�ʾ�ĵ� 
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

	//**�����ֺ�բ����**//

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
			string visualSymbol = "calcuSwitch2Pose";  //Զ���͵ؿ���Ŀǰ��������λ��Ϊ��ͬ�Ŀ��ؽ��д���
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
	//**������������������**//
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




	//******************�м�㼶�ĺ�������������**********************//

	//***ȡ����***//

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
			//�˴���Ƿȱһ����ȡ����ĽǶȲ������ת�����ο�λ�õĺ���

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

			//�˴���Ƿȱһ����ȡ����ĽǶȲ������ת�����ο�λ�õĺ���

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

	//***�Żع���***//
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
			//�˴���Ƿȱһ����ȡ����ĽǶȲ������ת�����ο�λ�õĺ���

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
			//�˴���Ƿȱһ����ȡ����ĽǶȲ������ת�����ο�λ�õĺ���

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


	



	//******************������˵ײ���صĺ������������̡���е�ۡ��Ӿ�ϵͳ����������صȡ�**********************//
	
	//**����λ�˷�������**//
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
		sleep(1);
		pubTransPose.publish(transGoal);
		cout<<transGoal<<endl;
	}
	//**���̻ص�����**//
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

	//**��е��λ�˷�������**//
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

	//**��е�۵�λ�ص�����**//
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


	//**���ʶ���ŷ�������**//
	void mm_recognition(const std::string str)
	{
		std_msgs::String recogOrder;
		recogOrder.data = str;
		cout << recogOrder << endl;
		sleep(0.2);
		pubRecogOrder.publish(recogOrder);

	}

	//**���ʶ������豸��Żص�����**//
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
	/**���ʶ���Ǳ�����ص�����**/
	//�˴���������ƣ�����һ���µ���Ϣ���ͣ����ݳ���
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

	//**���ʶ��ָʾ�ƻص�����**//
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


	//**���ʶ��Զ���͵ؿ��ػص�����**//
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
	//**����Ƕȷ�������**//
	void mm_end_motor(float theta)
	{
		mm_endmotor_control::motorCommand motorTheta;
		motorTheta.degree = theta;
		sleep(0.2);
		pubMotorTheta.publish(motorTheta);
		cout << motorTheta << endl;
		
	}
	//**�����λ�ص�����**//
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

	//**�Ӿ���λ��������**//
	void mm_visual_position(const std::string str)
	{
		
		std_msgs::String calcuOrder;
		calcuOrder.data = str;
		sleep(0.2);
		pubRecogOrder.publish(calcuOrder);
		cout << calcuOrder << endl;
	}

	//**�Ӿ���λ�ص�����**//
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
	sleep(10);
	robotDecision.firstInitialization();
	robotDecision.orderSubAndPub();
	ros::spin();
	return 0;
}
