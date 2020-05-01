#include <opencv2/core/core.hpp> 
#include <opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>  
#include "lightDeal.h"
#include "mm_visual_postion/utils/StereoImageReceiver.h"
#include "mm_visual_postion/AppInnerRequest.h"
#include "mm_robot_decision/VisualAppResponse.h"
#include "mm_visual_postion/utils/utils.h"
#include"std_msgs/Int8MultiArray.h"
#include"std_msgs/String.h"
#include"ros/ros.h"
#include<string.h>


using namespace std;
using namespace cv;



class RecognizeOrderSubscribeAndPublish
{
public:
  RecognizeOrderSubscribeAndPublish(ros::NodeHandle &n):cameraNode(n),image_receiver(n){}
  void SubscribeAndPublish()
  {
    lightPub=cameraNode.advertise<mm_robot_decision::VisualAppResponse>("/mm_visual/apps/light/result",100);
    orderSub=cameraNode.subscribe("/mm_visual/apps/light/goal",20,&RecognizeOrderSubscribeAndPublish::lightCallback,this);
  }


  void lightCallback(const mm_visual_postion::AppInnerRequest::ConstPtr& msg)
  {
    mm_robot_decision::VisualAppResponse result;
    ROS_INFO("Copy that");
    cout<<msg->object_name.c_str()<<endl;
    Mat left, right;
	  ros::Time latest_update_time;
    image_receiver.getLatestImages(left, right, latest_update_time);
    cv::Rect2f cv_left_roi_f;
    
    ROIMsgToCv(msg->left_roi, cv_left_roi_f);
    cv::Rect cv_left_roi = cv_left_roi_f;
    expandROI(cv_left_roi, 50, left.size());
    result.object_name = "light";
    result.frame_id = ENDEFFECTOR_FRAME_NAME;
    result.pose.state = "null";
    if(msg->object_name == "light")
    {
      //imshow("src", left);
      //waitKey();
      lightProcess Light;
      float *state1 = Light.findlights(left(cv_left_roi));
      if (state1[0]>0)
      {
        cout<<"Light Recognize Success"<<endl;
        if (state1[1]>0)
          result.object_status.push_back(1);
        else 
          result.object_status.push_back(0);
        if (state1[3]>0)
          result.object_status.push_back(1);
        else
          result.object_status.push_back(0);
        sleep(0.5);
        result.success = true;
        lightPub.publish(result);
      }
    else
    {
      cout<<"ERROR"<<endl;
      result.success = false;
      lightPub.publish(result);
    }

    }
  }

  private:
    ros::NodeHandle cameraNode;
    ros::Publisher lightPub;
    ros::Subscriber orderSub;
    StereoImageReceiver image_receiver;
};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"light11");
	  ros::NodeHandle nh;
    RecognizeOrderSubscribeAndPublish recogSubAndPub(nh);
    recogSubAndPub.SubscribeAndPublish();
    
    ros::spin();
    return 0;
}




