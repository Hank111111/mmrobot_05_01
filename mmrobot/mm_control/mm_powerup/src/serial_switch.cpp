#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <iostream>

using namespace std;
serial::Serial ros_ser;

//回调函数
void commandcallback(const std_msgs::UInt8::ConstPtr& command){
	int cmd = command->data;
	switch (cmd){

		case 1 :{
			cout << "开关5点动，UR控制柜上电" << endl;
			uint8_t s_buffer[13];
			s_buffer[0] = 0xFE;
			s_buffer[1] = 0x10;
			s_buffer[2] = 0x00;
			s_buffer[3] = 0x17;
			s_buffer[4] = 0x00;
			s_buffer[5] = 0x02;
			s_buffer[6] = 0x04;
			s_buffer[7] = 0x00;
			s_buffer[8] = 0x04;
			s_buffer[9] = 0x00;
			s_buffer[10] = 0x05;
			s_buffer[11] = 0x01;
			s_buffer[12] = 0x90;
        		ros_ser.write(s_buffer,13);
			sleep(0.5);
       			uint8_t callbackmsg[10];
			memset(callbackmsg,0,10);
        		ros_ser.read(callbackmsg,10);
			for (int i = 0; i < 10; i++)
           			ROS_INFO("[0x%02x]",callbackmsg[i]);
			break;	
			}

		case 2 :{
			cout << "开关6点动，初始化，释放制动器" << endl;
			uint8_t s_buffer[13];
			s_buffer[0] = 0xFE;
			s_buffer[1] = 0x10;
			s_buffer[2] = 0x00;
			s_buffer[3] = 0x1C;
			s_buffer[4] = 0x00;
			s_buffer[5] = 0x02;
			s_buffer[6] = 0x04;
			s_buffer[7] = 0x00;
			s_buffer[8] = 0x04;
			s_buffer[9] = 0x00;
			s_buffer[10] = 0x0A;
			s_buffer[11] = 0x00;
			s_buffer[12] = 0x27;
        		ros_ser.write(s_buffer,13);
			sleep(0.5);
       			uint8_t callbackmsg[10];
			memset(callbackmsg,0,10);
        		ros_ser.read(callbackmsg,10);
			for (int i = 0; i < 10; i++)
           			ROS_INFO("[0x%02x]",callbackmsg[i]);
			break;	
		}

		case 3 :{
			cout << "开关7点动，关闭控制柜" << endl;
			uint8_t s_buffer[13];
			s_buffer[0] = 0xFE;
			s_buffer[1] = 0x10;
			s_buffer[2] = 0x00;
			s_buffer[3] = 0x21;
			s_buffer[4] = 0x00;
			s_buffer[5] = 0x02;
			s_buffer[6] = 0x04;
			s_buffer[7] = 0x00;
			s_buffer[8] = 0x04;
			s_buffer[9] = 0x00;
			s_buffer[10] = 0x0A;
			s_buffer[11] = 0xC2;
			s_buffer[12] = 0xAA;
        		ros_ser.write(s_buffer,13);
			sleep(0.5);
       			uint8_t callbackmsg[10];
			memset(callbackmsg,0,10);
        		ros_ser.read(callbackmsg,10);
			for (int i = 0; i < 10; i++)
           			ROS_INFO("[0x%02x]",callbackmsg[i]);
			break;	
		}

		case 4 :{
			cout << "开关1断开,急停" << endl;
			uint8_t s_buffer[8];
			s_buffer[0] = 0xFE;
			s_buffer[1] = 0x05;
			s_buffer[2] = 0x00;
			s_buffer[3] = 0x00;
			s_buffer[4] = 0xFF;
			s_buffer[5] = 0x00;
			s_buffer[6] = 0x98;
			s_buffer[7] = 0x35;
        		ros_ser.write(s_buffer,8);
			sleep(0.5);
       			uint8_t callbackmsg[10];
			memset(callbackmsg,0,10);
        		ros_ser.read(callbackmsg,10);
			for (int i = 0; i < 10; i++)
           			ROS_INFO("[0x%02x]",callbackmsg[i]);
			break;	
		}

		case 5 :{
			cout << "开关1闭合,取消急停" << endl;
			uint8_t s_buffer[8];
			s_buffer[0] = 0xFE;
			s_buffer[1] = 0x05;
			s_buffer[2] = 0x00;
			s_buffer[3] = 0x00;
			s_buffer[4] = 0x00;
			s_buffer[5] = 0x00;
			s_buffer[6] = 0xD9;
			s_buffer[7] = 0xC5;
        		ros_ser.write(s_buffer,8);
			sleep(0.5);
       			uint8_t callbackmsg[10];
			memset(callbackmsg,0,10);
        		ros_ser.read(callbackmsg,10);
			for (int i = 0; i < 10; i++)
           			ROS_INFO("[0x%02x]",callbackmsg[i]);
			break;	
		}

		case 9 :{
			cout << "全关，重整" << endl;
			uint8_t s_buffer[10];
			s_buffer[0] = 0xFE;
			s_buffer[1] = 0x0F;
			s_buffer[2] = 0x00;
			s_buffer[3] = 0x00;
			s_buffer[4] = 0x00;
			s_buffer[5] = 0x08;
			s_buffer[6] = 0x01;
			s_buffer[7] = 0x00;
			s_buffer[8] = 0xB1;
			s_buffer[9] = 0x91;
        		ros_ser.write(s_buffer,10);
			sleep(0.5);
       			uint8_t callbackmsg[10];
			memset(callbackmsg,0,10);
        		ros_ser.read(callbackmsg,10);
			for (int i = 0; i < 10; i++)
           			ROS_INFO("[0x%02x]",callbackmsg[i]);
			break;	
		}

		case 0 :{
			cout << "查询开关状态" << endl;
			uint8_t s_buffer[8];
			s_buffer[0] = 0xFE;
			s_buffer[1] = 0x01;
			s_buffer[2] = 0x00;
			s_buffer[3] = 0x00;
			s_buffer[4] = 0x00;
			s_buffer[5] = 0x08;
			s_buffer[6] = 0x29;
			s_buffer[7] = 0xC3;
        		ros_ser.write(s_buffer,8);
			sleep(0.5);
       			uint8_t switch_state[10];
			memset(switch_state,0,10);
        		ros_ser.read(switch_state,10);
			for (int i = 0; i < 10; i++)
           			ROS_INFO("[0x%02x]",switch_state[i]);
			break;
		}
	
     		default:{
      			cout << "未查询到相关指令" << endl;
      			break;	
		}
	}
     		sleep(0.5);
}


int main (int argc, char** argv){
     ros::init(argc, argv, "switch_serial_node");
     ros::NodeHandle n;
     //订阅主题motor_command
     ros::Subscriber command_sub = n.subscribe("/mm_powerup/switch_command", 1000, commandcallback);
     //发布主题sensor
     //ros::Publisher sensor_pub = n.advertise<std_msgs::String>("sensor", 1000);
     try
     {
         ros_ser.setPort("/dev/switch");
         ros_ser.setBaudrate(9600);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser.setTimeout(to);
         ros_ser.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }
     if(ros_ser.isOpen()){
         ROS_INFO_STREAM("Serial Port opened");
     }else{
         return -1;
     }
     sleep(1);


     ROS_INFO_STREAM("Listening to command and communicating with serial port");

     ros::spin();
     return 0;    
 }





