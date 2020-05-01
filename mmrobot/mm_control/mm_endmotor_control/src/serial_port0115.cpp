#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include "mm_endmotor_control/motorCommand.h"
#include "std_msgs/Int8.h"

//0112修改说明：增加了单次上电的回零，设置返回'*'时的中断（上一条指令未执行完，存储待执行）



serial::Serial ros_ser;
void sendorder(char order[], size_t length);
void getback();

class MotorCommandSubAndPub
{
public:
   void SubAndPub()
     {
	commandSub=motorNode.subscribe("/mm_motor/motor_command", 1000, &MotorCommandSubAndPub::commandcallback,this);
	overPub=motorNode.advertise<std_msgs::Int8>("/mm_motor/isArrived",10);
     }


//回调函数
void commandcallback(const mm_endmotor_control::motorCommand::ConstPtr& command){
     //设置速度
     float degree = command->degree;
     char VE_order[] = {'V','E','1'};
     sendorder(VE_order, sizeof(VE_order)); 
     std_msgs::Int8 IsArrivedmsg;
     if (degree>0.0)
     {
         int counts = 200000*degree/360;
         int times = 0;
         while (counts !=0)
         {
            counts = counts / 10;
            times++;
         }
         char FL_order[times+2];       //计算长度
         counts = 200000*degree/360;   //填入数据
         FL_order[0] = 'F';
         FL_order[1] = 'L';
         for (int i = 0; i < times; i++)
         {
           int mo = counts % 10;
           FL_order[times+1-i] = (char)('0' + mo);
           counts = counts / 10;
         }
         sendorder(FL_order, sizeof(FL_order)); 
         sleep(8);
	 IsArrivedmsg.data=1;
	 sleep(0.1);
         overPub.publish(IsArrivedmsg);

     }
     else if (degree<0.0)
     {
         degree = -degree;
         int counts = 200000*degree/360;
         int times = 0;
         while (counts !=0)
         {
            counts = counts / 10;
            times++;
         }
         char FL_order[times+3];       //计算长度
         counts = 200000*degree/360;   //填入数据
         FL_order[0] = 'F';
         FL_order[1] = 'L';
         FL_order[2] = '-';
         for (int i = 0; i < times; i++)
         {
           int mo = counts % 10;
           FL_order[times+2-i] = (char)('0' + mo);
           counts = counts / 10;
         }
         sendorder(FL_order, sizeof(FL_order)); 
         sleep(4);
	 IsArrivedmsg.data=1;
 	 sleep(0.1);
         overPub.publish(IsArrivedmsg);
     }
     sleep(1);
 }

private:
	ros::NodeHandle motorNode;
        ros::Subscriber commandSub;
	ros::Publisher overPub;
};
     

int main (int argc, char** argv){
     ros::init(argc, argv, "motor_serial_node");
     MotorCommandSubAndPub motorMove;
     motorMove.SubAndPub();
     try
     {
         ros_ser.setPort("/dev/ttyACM0");
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

     //电机使能
     char SI_order[] = {'S','I','1'};
     sendorder(SI_order, sizeof(SI_order)); 


     sleep(1);
     ROS_INFO_STREAM("Listening to command and communicating with serial port");
     
     //测试不断电归零
     /*char test1_order[] = {'F','L','1','5','6','3','4','6'};
     sendorder(test1_order, sizeof(test1_order)); 

     sleep(10);
     char test2_order[] = {'F','L','-','6','4','6','3','2'};
     sendorder(test2_order, sizeof(test2_order)); 
     sleep(5);

     getback();
     */

     ros::spin();
     return 0;    
 }

//赋值发送指令,检验是否收到，只判断是否为%
void sendorder(char order[], size_t length)
{
     char flag = '0';
     while(flag != '%')
     {
        uint8_t order_buffer[length+1];
        //赋值并加上终止行,发送
        for(int i = 0; i<length; i++)
            order_buffer[i] = order[i];
        order_buffer[length] = 0x0d;
        ros_ser.write(order_buffer,length+1);
        //获取缓冲区内的字节数
        uint8_t testSI[2];
        ros_ser.read(testSI,2);
        flag = testSI[0];
        ROS_INFO_STREAM(order_buffer);
        ROS_INFO_STREAM(flag);
        sleep(2);
        if (flag == '*')
           break;
     } 
}


//记录单次上电不断电运动距离，并归零
void getback()
{
     uint8_t abspoi[20], test[2];
     int icount;
     char IFD_order[] = {'I','F','D'};
     sendorder(IFD_order, sizeof(IFD_order));
     uint8_t IP_order[] = {'I','P',0x0d};
     ros_ser.write(IP_order,3);
     ros_ser.read(abspoi,20);
     ROS_INFO_STREAM(abspoi); 
     while (abspoi[0] != 'I')
     {
         ros_ser.write(IP_order,3);
         ros_ser.read(abspoi,20); 
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
     }
}
     


