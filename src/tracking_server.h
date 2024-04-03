#ifndef TRACKING_SERVER_H
#define TRACKING_SERVER_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <actionlib/server/simple_action_server.h>
#include <ai_robot_control/trackingAction.h>
#include <ai_robot_waving/SendLocalTarget.h>
// #include <ai_robot_tracking/ToggleModule.h>

typedef actionlib::SimpleActionServer<ai_robot_control::trackingAction> trackingServer; 

class Tracking_server
{
public:
    Tracking_server();
    ~Tracking_server();
private:
    void executeCb(const ai_robot_control::trackingActionGoalConstPtr& tracking_goal, trackingServer* as);
    void target_poseCb(const ai_robot_control::trackingActionFeedbackConstPtr& msg);
    ros::NodeHandle nh_;
    ros::ServiceClient target_send_srv_;
    ros::Subscriber target_pose_sub_;

    double control_freq_;
    ai_robot_control::trackingActionFeedback target_pose;

};


#endif //!TRACKING_SERVER_H