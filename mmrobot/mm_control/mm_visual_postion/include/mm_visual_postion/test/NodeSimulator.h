#ifndef __NODE_SIMULATOR_H__
#define __NODE_SIMULATOR_H__
#include "ros/ros.h"
#include <ros/callback_queue.h>

template <typename T_SUB, typename T_PUB>
class NodeSimulator{
protected:
    ros::Publisher msg_pub_;
    ros::Subscriber msg_sub_;
    bool msg_received_;
    T_SUB registered_msg_received_;
    bool pattern_activated_;
    T_SUB pattern_sub_msg_;
    T_PUB pattern_pub_msg_;

public:
    std::string object_name_;
    VisualAppNodeSimulator(ros::NodeHandle& nh, std::string sub_topic_name, std::string pub_topic_name){
        msg_pub_ = nh.advertise<T_PUB>(pub_topic_name, 3);    
        msg_sub_ = nh.subscribe(sub_topic_name, 1000, msgCallback);
        msg_received_ = false;
        pattern_activate_ = false;
    }
    void pubMsg(const T_PUB& pub_msg){
        response_pub_.publish(pub_msg);
        msg_received_=false;
    }
    void setPattern(T_SUB& sub_msg, T_PUB& pub_msg){
        // once receive the sub_msg, publish the pub_msg
        pattern_sub_msg_ = sub_msg;
        pattern_pub_msg_ = pub_msg;
        pattern_activated_ = true;
    }
    void msgCallback(const T_SUB::ConstPtr& msg_ptr){
        msg_received = true;
        registered_msg_received_ = *msg_ptr;
        if(pattern_activated_ && if(*msg_ptr == pattern_sub_msg_)){
            pubMsg(pattern_pub_msg_);
        }
    }
    bool lastMsgReceived(T_SUB& last_msg){
        if(msg_received_){
            last_msg = registered_msg_received_;
            msg_received_ = false;
            return true;
        }
        else{
            return false;
        }
    }
};

class VisualNodeSimulator: public NodeSimulator <mm_robot_decision::AppInnerRequest, mm_robot_decision::VisualAppResponse>{

public:
    std::string object_name_;
    VisualAppNodeSimulator(ros::NodeHandle& nh, std::string object_name, std::string sub_topic_name, std::string pub_topic_name)
    :NodeSimulator(nh, sub_topic_name, pub_topic_name){
        object_name_ = object_name;
    }
    
    void cmdCallback(const T_SUB::ConstPtr& msg_ptr) override{
        if(msg_ptr->object_name == object_name_){
            msg_received_ = true;
            registered_msg_received_ = *msg_ptr;
        }
    }
};

class VisualNodeSimulatorVector{
public:
    std::vector<VisualNodeSimulator> vec;
    VisualNodeSimulator& operator [](std::string name){
        for(unsigned int i=0; i<vec.size(); i++){
            if(name == vec[i].object_name_)
                return vec[i].object_name_;
        }
        std::string error_info = "cannot find " + name + " in the node simulator vector";
        assert(error_info.c_str());
    }
    void push_back_new(ros::NodeHandle& nh, std::string object_name, std::string sub_topic_name, std::string pub_topic_name){
        vec.push_back(VisualNodeSimulator(nh, object_name, sub_topic_name, pub_topic_name));
    }
}

#endif