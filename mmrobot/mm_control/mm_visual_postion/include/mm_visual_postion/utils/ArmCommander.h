#ifndef __ARM_COMMANDER_H__
#define __ARM_COMMANDER_H__
#include "ros/ros.h"

#include "mm_robot_decision/pose4.h"
#include <std_msgs/Int8.h>
class ArmCommander{
private:

    ros::Publisher cmd_pub;
    ros::Subscriber arrived_sub;
    ros::NodeHandle n;
    bool arrived;
    ros::Time pub_time;
public:
    ArmCommander(ros::NodeHandle& nh):n(nh){
        arrived = true;
        cmd_pub = n.advertise<mm_robot_decision::pose4>("/mm_arm/goal", 1);
        arrived_sub = n.subscribe("/mm_arm/isArrived", 1, &ArmCommander::arriveCallback, this); // this signal is not accurate currently, try to check manually the position of end effector in base
    }
    void arriveCallback(const std_msgs::Int8::ConstPtr msg){
        if(msg->data == 1) arrived = true;
    }
    void moveByBaseSpacePlanning(const double& x, const double& y, const double& z, const double& a, const double& b, const double& c){
        // in meter and radian
        std::cout<<"move in base :"<<" x y z a b c: "<<x<<" "<<y<<" "<<z<<" "<<a<<" "<<b<<" "<<c<<std::endl;
        std::cout<<"Make sure the a b c is the aix-angle represention (not euler, not quaternion"<<std::endl;
        mm_robot_decision::pose4 pose_msg;
        pose_msg.state = "BaseSpacePlanning";
        pose_msg.x = x;
        pose_msg.y = y;
        pose_msg.z = z;
        pose_msg.a = a;
        pose_msg.b = b;
        pose_msg.c = c;
        arrived = false;
        cmd_pub.publish(pose_msg);
        pub_time = ros::Time::now();
    }

    void moveByToolSpacePlanning(const double& x, const double& y, const double& z, const double& q_x, const double& q_y, const double& q_z, const double& q_w){
        // in meter and quaternion
        assert(fabs(q_x*q_x + q_y*q_y + q_z*q_z + q_w*q_w - 1.0 ) < 1e-5);
        std::cout<<"move in tool :"<<" x y z q_x q_y q_z q_w: "<<x<<" "<<y<<" "<<z<<" "<<q_x<<" "<<q_y<<" "<<q_z<<" "<<q_w<<std::endl;
        mm_robot_decision::pose4 pose_msg;
        pose_msg.state = "MoveInTool";
        pose_msg.x = x;
        pose_msg.y = y;
        pose_msg.z = z;
        pose_msg.a = q_x;
        pose_msg.b = q_y;
        pose_msg.c = q_z;
        pose_msg.w = q_w;
        arrived = false;
        cmd_pub.publish(pose_msg);
        pub_time = ros::Time::now();
    }
    void moveByWorkSpacePlanning(const double& x, const double& y, const double& z, const double& q_x, const double& q_y, const double& q_z, const double& q_w){
        // in meter and quaternion
        assert(fabs(q_x*q_x + q_y*q_y + q_z*q_z + q_w*q_w - 1.0 ) < 1e-5);
        std::cout<<"move in WorkSpacePlanning :"<<" x y z q_x q_y q_z q_w: "<<x<<" "<<y<<" "<<z<<" "<<q_x<<" "<<q_y<<" "<<q_z<<" "<<q_w<<std::endl;
        mm_robot_decision::pose4 pose_msg;
        pose_msg.state = "WorkSpacePlanning";
        pose_msg.x = x;
        pose_msg.y = y;
        pose_msg.z = z;
        pose_msg.a = q_x;
        pose_msg.b = q_y;
        pose_msg.c = q_z;
        pose_msg.w = q_w;
        arrived = false;
        cmd_pub.publish(pose_msg);
        pub_time = ros::Time::now();
    }
    void standBy(){
        mm_robot_decision::pose4 pose_msg;
        pose_msg.state = "StandBy";
        arrived = false;
        cmd_pub.publish(pose_msg);
    }
    void moveByJointSpacePlanning(const double& x, const double& y, const double& z, 
                            const double& a, const double& b, const double& c){
        // in meter and rad.
        mm_robot_decision::pose4 pose_msg;
        pose_msg.state = "jointSpacePlanning";
        pose_msg.x = x;
        pose_msg.y = y;
        pose_msg.z = z;
        pose_msg.a = a;
        pose_msg.b = b;
        pose_msg.c = c;
        arrived = false;
        cmd_pub.publish(pose_msg);                          
    }
    void backHome(){
        mm_robot_decision::pose4 pose_msg;
        pose_msg.state = "BackHome";
        arrived = false;
        cmd_pub.publish(pose_msg);
    }
    bool isArrived(){
        return arrived;
    }
    bool isTimeOut(){
        if((ros::Time::now() - pub_time).toSec() > 10) return true;
        else return false;
    }

    bool waitToArrive(double timeout_seconds, volatile bool& stop_loop_condition){
        // attention: the thread will be blocked
        const double loop_sleep_time = 0.02;
        int wait_loops = timeout_seconds / loop_sleep_time;
        for(int i=0; i<wait_loops; i++){
            if(arrived){
                return true;
            }
            if(stop_loop_condition){
                return false;
            }
            ros::spinOnce();
            ros::Duration(loop_sleep_time).sleep();
        }
        return false; //time out;
    }
};

#endif