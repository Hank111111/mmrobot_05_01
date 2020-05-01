#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include "mm_endmotor_control/motorCommand.h"
#include "std_msgs/Int8.h"

using namespace std;

//0112修改说明：增加了单次上电的回零，设置返回'*'时的中断（上一条指令未执行完，存储待执行）
//0115修改说明：增加了读取绝对位置的函数，每次运动完记录当前位置;通过两次速度判断是否到位
//0415修改说明：通过位置判断到位，增加寻零方案，增加超位报警

serial::Serial ros_ser;
void sendorder(char order[], size_t length);
void getback();
long long int hex2ten(char *str, int k);
float getAbsposition();
void write2txt();  //调用函数记录当前绝对位置和角度，存入txt
void checkstate(); //速度为零，则判断为到位，废弃
int getVelocity();

class MotorCommandSubAndPub
{
public:
	void SubAndPub()
	{
		commandSub = motorNode.subscribe("/mm_motor/motor_command", 1000, &MotorCommandSubAndPub::commandcallback, this);
		stopSub = motorNode.subscribe("/mm_motor/motor_stop",1000,&MotorCommandSubAndPub::stopcallback, this);
		overPub = motorNode.advertise<std_msgs::Int8>("/mm_motor/isArrived", 10);
	}

	//回调函数
	void commandcallback(const mm_endmotor_control::motorCommand::ConstPtr &command)
	{

		//初始化返回信息
		std_msgs::Int8 IsArrivedmsg;
		IsArrivedmsg.data = 0;
		double init_secs;
		double timeout;
		if (command->workstate == "home")
		{
			char AR_order[] = {'A', 'R'};
			sendorder(AR_order, sizeof(AR_order));
			char ME_order[] = {'M', 'E'};
			sendorder(ME_order, sizeof(ME_order));
			int inihpoi = getAbsposition();
			char DL1_order[] = {'D', 'L', '1', '9'};
			sendorder(DL1_order, sizeof(DL1_order));
			char FH1_order[] = {'F', 'H', '1'};
			sendorder(FH1_order, sizeof(FH1_order));
			sleep(0.3);

			float homepoi;
			init_secs = ros::Time::now().toSec();
			do
			{
				homepoi = getAbsposition();
				timeout = ros::Time::now().toSec() - init_secs;
				if ((fabs(homepoi - inihpoi) > 360 && homepoi != 0) || timeout > 25.0)
				{
					IsArrivedmsg.data = 2; //之后改为报警信息
					char SK_order[] = {'S', 'K'};
					sendorder(SK_order, sizeof(SK_order));
					break;
				}
				cout << "本次寻零已转过角度： " << (homepoi - inihpoi) << endl;
				cout << "本次寻零已花费时间： " << timeout << endl;
			} while (homepoi != 0);

			char DL2_order[] = {'D', 'L', '3'};
			sendorder(DL2_order, sizeof(DL2_order));

			if (homepoi == 0)
			{
				//相对物理零点，便于计算，需加一步FL，待测试
				char RP_order[] = {'F', 'L', '-', '1', '9', '5', '0', '0'};
				sendorder(RP_order, sizeof(RP_order));
				char EP0_order[] = {'E', 'P', '0'};
				sendorder(EP0_order, sizeof(EP0_order));
				char SP0_order[] = {'S', 'P', '0'};
				sendorder(SP0_order, sizeof(SP0_order));
				IsArrivedmsg.data = 1;
			}
		}

		if (command->workstate == "move")
		{
			//设置速度
			float startdegree = getAbsposition();
			ROS_INFO_STREAM(startdegree);
			float degree = command->degree - startdegree;
			float order_deg = degree;
			float inipoi, curpoi;
			inipoi = startdegree;

			if (fabs(degree) < 0.1)
			{
				IsArrivedmsg.data = 1;
				overPub.publish(IsArrivedmsg);
			}

			if (degree > 0.0)
			{
				int counts = 200000 * degree / 360;
				int times = 0;
				while (counts != 0)
				{
					counts = counts / 10;
					times++;
				}
				char FL_order[times + 2];		//计算长度
				counts = 200000 * degree / 360; //填入数据
				FL_order[0] = 'F';
				FL_order[1] = 'L';
				for (int i = 0; i < times; i++)
				{
					int mo = counts % 10;
					FL_order[times + 1 - i] = (char)('0' + mo);
					counts = counts / 10;
				}
				sendorder(FL_order, sizeof(FL_order));
			}
			else if (degree < 0.0)
			{
				degree = -degree;
				int counts = 200000 * degree / 360;
				int times = 0;
				while (counts != 0)
				{
					counts = counts / 10;
					times++;
				}
				char FL_order[times + 3];		//计算长度
				counts = 200000 * degree / 360; //填入数据
				FL_order[0] = 'F';
				FL_order[1] = 'L';
				FL_order[2] = '-';
				for (int i = 0; i < times; i++)
				{
					int mo = counts % 10;
					FL_order[times + 2 - i] = (char)('0' + mo);
					counts = counts / 10;
				}
				sendorder(FL_order, sizeof(FL_order));
			}

			//checkstate();
			float checkerror, twiceerror;
			float lastpoi = -100000;
			init_secs = ros::Time::now().toSec();
			do
			{
				curpoi = getAbsposition();
				timeout = ros::Time::now().toSec() - init_secs;
				checkerror = curpoi - inipoi;
				twiceerror = curpoi - lastpoi;
				if (fabs(twiceerror) < 0.5 || timeout > fabs(order_deg) / 12.5)
				{
					IsArrivedmsg.data = 2; //之后改为报警信息
					char SK_order[] = {'S', 'K'};
					sendorder(SK_order, sizeof(SK_order));
					break;
				}
				cout << "本次运动已转过角度： " << checkerror << endl;
				cout << "本次运动已花费时间： " << timeout << endl;
				lastpoi = curpoi;
			} while (fabs(checkerror - order_deg) > 0.1);

			if (fabs(checkerror - order_deg) < 0.1)
			{
				IsArrivedmsg.data = 1;
			}
		}

		if (command->workstate == "jog")
		{
			float dir = command->degree;
			if (dir >= 0.0)
			{
				//设置方向，可外传参
				char DI_order[] = {'D', 'I', '-', '1', '0', '0'};
				sendorder(DI_order, sizeof(DI_order));
			}
			if (dir < 0.0)
			{
				//设置方向，可外传参
				char DI_order[] = {'D', 'I', '1', '0', '0'};
				sendorder(DI_order, sizeof(DI_order));
			}

			//设置速度
			char JS_order[] = {'J', 'S', '5'};
			sendorder(JS_order, sizeof(JS_order));
			char CJ_order[] = {'C', 'J'};
			sendorder(CJ_order, sizeof(CJ_order));
			init_secs = ros::Time::now().toSec();
			int vel;
//			do
//			{
//				timeout = ros::Time::now().toSec() - init_secs;
//				vel = getVelocity();
//				cout << "本次运动已花费时间： " << timeout << endl;
//				cout << "当前速度: " << vel << endl;
//				if ( abs(vel)<50 || timeout > 50)
//				{
//					//vel = getVelocity();
//					//if ( abs(vel)<50)
//					{
//						char SK_order[] = {'S', 'K'};
//						char MD_order[] = {'M', 'D'};
//						cout << "到达机械限制： " << timeout << endl;
//						sendorder(SK_order, sizeof(SK_order));
//						sendorder(MD_order, sizeof(MD_order));
//						IsArrivedmsg.data = 1;
//						break;
//					}
//				}
//			} while (1);
			do
			{
				timeout = ros::Time::now().toSec() - init_secs;
				vel = getVelocity();
				cout << "本次运动已花费时间： " << timeout << endl;
				cout << "当前速度: " << vel << endl;
			}while(timeout<3.8 && ros::ok());

			if (dir > 0.0)
			{
				char CS_order01[] = {'C', 'S', '-','1','5'};
				sendorder(CS_order01, sizeof(CS_order01));
				sleep(0.6);
				char CS_order02[] = {'C', 'S', '-','3','0'};
				sendorder(CS_order02, sizeof(CS_order02));
			}
			else if (dir < 0.0)
			{
				char CS_order01[] = {'C', 'S','1','5'};
				sendorder(CS_order01, sizeof(CS_order01));
				sleep(0.6);
				char CS_order02[] = {'C', 'S','3','0'};
				sendorder(CS_order02, sizeof(CS_order02));
			}

			do
			{
				timeout = ros::Time::now().toSec() - init_secs;
				vel = getVelocity();
				cout << "本次运动已花费时间： " << timeout << endl;
				cout << "当前速度: " << vel << endl;
			}while(timeout<15.8  && ros::ok());

			if (dir > 0.0)
			{
				char CS_order2[] = {'C', 'S', '-','1','2'};
				sendorder(CS_order2, sizeof(CS_order2));
			}

			else if (dir < 0.0)
			{
				char CS_order2[] = {'C', 'S', '1','2'};
				sendorder(CS_order2, sizeof(CS_order2));
			}


			do
			{
				timeout = ros::Time::now().toSec() - init_secs;
				vel = getVelocity();
				cout << "本次运动已花费时间： " << timeout << endl;
				cout << "当前速度: " << vel << endl;
				if ( abs(vel)<50 || timeout > 30)
				{
//					vel = getVelocity();
//					if ( abs(vel)<50)
					{
						char SK_order[] = {'S', 'K'};
						char MD_order[] = {'M', 'D'};
						cout << "到达机械限制： " << timeout << endl;
						sendorder(SK_order, sizeof(SK_order));
						sendorder(MD_order, sizeof(MD_order));
						IsArrivedmsg.data = 1;
						break;
					}
				}
			} while (ros::ok());

		}
		overPub.publish(IsArrivedmsg);
	}

	void stopcallback(const std_msgs::String::ConstPtr &command){
		if (command->data == "stop")
		{
			char SK_order[] = {'S', 'K'};
			sendorder(SK_order, sizeof(SK_order));
			char MD_order[] = {'M', 'D'};
			sendorder(MD_order, sizeof(MD_order));
		}

	}

private:
	ros::NodeHandle motorNode;
	ros::Subscriber commandSub;
	ros::Subscriber stopSub;
	ros::Publisher overPub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_serial_node");
	MotorCommandSubAndPub motorMove;
	motorMove.SubAndPub();
	try
	{
		ros_ser.setPort("/dev/endmotor");
		ros_ser.setBaudrate(9600);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ros_ser.setTimeout(to);
		ros_ser.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	if (ros_ser.isOpen())
	{
		ROS_INFO_STREAM("Serial Port opened");
	}
	else
	{
		return -1;
	}
	sleep(0.5);

	//电机使能
	char ME_order[] = {'M','E'};
	sendorder(ME_order, sizeof(ME_order));
	//16进制消息
	char IF_order[] = {'I', 'F', 'H'};
	sendorder(IF_order, sizeof(IF_order));

	//速度设置
	char VE_order[] = {'V', 'E', '3'};
	sendorder(VE_order, sizeof(VE_order));

	ROS_INFO_STREAM("Listening to command and communicating with serial port");

	//测试不断电归零
	/*char test1_order[] = {'F','L','1','5','6','3','4','6'};
     sendorder(test1_order, sizeof(test1_order)); 
     checkstate(); 

     char test2_order[] = {'F','L','-','6','4','6','3','2'};
     sendorder(test2_order, sizeof(test2_order)); 
     checkstate(); 

     getAbsposition();
     write2txt();
     getback();
     checkstate(); */
	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
	return 0;
}

//赋值发送指令,检验是否收到，判断是否为%
void sendorder(char order[], size_t length)
{
	char flag = '0';
	while (flag != '%')
	{
		uint8_t order_buffer[length + 1];
		//赋值并加上终止行,发送
		for (int i = 0; i < length; i++)
			order_buffer[i] = order[i];
		order_buffer[length] = 0x0d;
		ros_ser.write(order_buffer, length + 1);
		//获取缓冲区内的字节数
		uint8_t testSI[10];
		ros_ser.read(testSI, 10);
		flag = testSI[0];
		ROS_INFO_STREAM(order_buffer);
		ROS_INFO_STREAM(flag);
		if (flag == '*')
			break;
	}
}

//已废弃
void getback()
{
	int abspos;
	abspos = getAbsposition();
	if (abspos > 0)
	{
		int counts = abspos;
		int times = 0;
		while (counts != 0)
		{
			counts = counts / 10;
			times++;
		}
		char FL_order[times + 3]; //计算长度
		counts = abspos;		  //重新填入数据
		FL_order[0] = 'F';
		FL_order[1] = 'L';
		FL_order[2] = '-';
		for (int i = 0; i < times; i++)
		{
			int mo = counts % 10;
			FL_order[times + 2 - i] = (char)('0' + mo);
			counts = counts / 10;
		}
		sendorder(FL_order, sizeof(FL_order));
		//sleep(8);
	}
	else if (abspos < 0)
	{
		int counts = -abspos;
		int times = 0;
		while (counts != 0)
		{
			counts = counts / 10;
			times++;
		}
		char FL_order[times + 2]; //计算长度
		counts = -abspos;		  //填入数据
		FL_order[0] = 'F';
		FL_order[1] = 'L';
		for (int i = 0; i < times; i++)
		{
			int mo = counts % 10;
			FL_order[times + 1 - i] = (char)('0' + mo);
			counts = counts / 10;
		}
	}
	/*uint8_t abspoi[20], test[2];
     int icount;
     char IFD_order[] = {'I','F','D'};
     sendorder(IFD_order, sizeof(IFD_order));
     uint8_t IP_order[] = {'I','P',0x0d};
     ros_ser.write(IP_order,3);
     ros_ser.read(abspoi,20);
     ROS_INFO_STREAM(abspoi); 
     while (abspoi[0] != 'I')
     {
         sleep(0.5);
         ros_ser.write(IP_order,3);
         ros_ser.read(abspoi,20); 
	 ROS_INFO_STREAM(abspoi); 
     }
     abspoi[0] = 'F';
     abspoi[1] = 'L';
     if (abspoi[3] == '-')
     {
         for (int i = 2; i<20; i++)
         {
             abspoi[i] = abspoi[i+2];
             if (abspoi[i] == 0x0d)
             {
                  icount = i+1;
                  break;
             }
         }
     }
     else
     {
         abspoi[2] = '-';
         for (int i = 3; i<20; i++)
         {
             if (abspoi[i] == 0x0d)
             {
                  icount = i+1;
                  break;
             }
         }  
     }
     ROS_INFO_STREAM(abspoi); 
     test[0] = ' ';
     while (test[0] != '%')
     {
         ros_ser.write(abspoi,icount);
         ros_ser.read(test,2);
         ROS_INFO_STREAM(test[0]);
         if (test[0] == '*')
           break;
         sleep(2);
     }*/
}

long long int hex2ten(uint8_t *str, int k)
{
	long long int var = 0;
	long long int t;
	/*int len = strlen(str);
	if (var > 8) //最长8位
		return -1;*/
	for (int i = 0; i < k; i++)
	{
		if (*str >= 'A' && *str <= 'F')
			t = *str - 55;
		else
			t = *str - 48;
		var <<= 4;
		var |= t;
		str++;
	}
	return var;
}

float getAbsposition()
{
	uint8_t temp[100], abs[9];
	uint8_t IP_order[] = {'I', 'P', 0x0d};
	ros_ser.write(IP_order, 3);
	ros_ser.read(temp, 20);
	while (temp[0] != 'I')
	{
		ros_ser.write(IP_order, 3);
		ros_ser.read(temp, 20);
	}
	//ROS_INFO_STREAM(temp);
	for (int i = 0; i < 8; i++)
		abs[i] = temp[i + 3];
	abs[8] = '\0';
	//ROS_INFO_STREAM(abs);
	long long int pos = hex2ten(abs, 8);
	if (pos > 2147483648)
		pos = pos - 4294967296;
	//ROS_INFO_STREAM(pos);
	float degree = pos * 360.0 / 200000.0;
	return degree;
}

int getVelocity()
{
	uint8_t temp[100], abs[5];
	uint8_t IV_order[] = {'I', 'V', 0x0d};
	ros_ser.write(IV_order, 3);
	ros_ser.read(temp, 10);
	while (temp[0] != 'I')
	{
		ros_ser.write(IV_order, 3);
		ros_ser.read(temp, 10);
	}
	//ROS_INFO_STREAM(temp);
	for (int i = 0; i < 4; i++)
		abs[i] = temp[i + 3];
	abs[4] = '\0';
	//ROS_INFO_STREAM(abs);
	long long int vel = hex2ten(abs, 4);
	if (vel > 32767)
		vel = vel - 65536;
	//ROS_INFO_STREAM(pos);
	return vel;
}

//已废弃
void write2txt()
{
	int result = getAbsposition();
	FILE *fp = NULL;
	fp = fopen("/home/youibot/mrobot/src/mm_control/mm_endmotor_control/src/currentposition.txt", "w");
	double degree = result * 360.0 / 200000.0;
	fprintf(fp, "counts//%d\n", result);
	fprintf(fp, "degrees//%f\n", degree);
	fclose(fp);
	fp = NULL;
}

//已废弃
void checkstate()
{
	uint8_t back[10];
	uint8_t IV_order[] = {'I', 'V', '0', 0x0d};
	while (1)
	{
		ros_ser.write(IV_order, 4);
		ros_ser.read(back, 8);
		if (back[3] == '0' && back[4] == '0' && back[5] == '0' && back[6] == '0')
		{
			sleep(0.5);
			ros_ser.write(IV_order, 4);
			ros_ser.read(back, 8);
			if (back[3] == '0' && back[4] == '0' && back[5] == '0' && back[6] == '0')
			{
				ROS_INFO_STREAM(back);
				break;
			}
		}
		sleep(0.5);
	}
}
