#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include "mm_endmotor_control/motorCommand.h"
serial::Serial ros_ser;
void sendorder(char order[], size_t length);


//回调函数
void commandcallback(const mm_endmotor_control::motorCommand::ConstPtr& command){
     //设置速度
     float degree = command->degree;
     char VE_order[] = {'V','E','1'};
     sendorder(VE_order, sizeof(VE_order)); 
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
     }
     sleep(1);
 }


int main (int argc, char** argv){
     ros::init(argc, argv, "motor_serial_node");
     ros::NodeHandle n;
     //订阅主题motor_command
     ros::Subscriber command_sub = n.subscribe("/mm_motor/motor_command", 1000, commandcallback);
     //发布主题sensor
     //ros::Publisher sensor_pub = n.advertise<std_msgs::String>("sensor", 1000);
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
     //上电
     char SI_order[] = {'S','I','1'};
     sendorder(SI_order, sizeof(SI_order)); 


     sleep(1);
     ROS_INFO_STREAM("Listening to command and communicating with serial port");

     ros::spin();
     return 0;    
 }

//赋值发送指令,检验是否收到
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
     } 
}



