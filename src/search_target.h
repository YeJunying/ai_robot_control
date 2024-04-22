#ifndef SEARCH_TARGET_H
#define SEARCH_TARGET_H

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <ai_robot_control/trackingAction.h>
#include <ai_robot_waving/SendLocalTarget.h>
#include <ai_robot_tracking/ToggleModule.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

typedef actionlib::SimpleActionClient<ai_robot_control::trackingAction> trackingClient; 

class Search_target : public BT::StatefulActionNode
{
public:
    Search_target(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh);
    ~Search_target();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    //处理接口参数
    static BT::PortsList providedPorts() 
    {
        BT::PortsList ports_list;
        return ports_list;
    }


private:
    ros::Publisher target_pub_;
    ros::Publisher vel_pub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber odom_sub_;
    ai_robot_control::trackingGoal goal;
    void feedbackCb(const ai_robot_control::trackingFeedbackConstPtr& msg);
    void doneCb();
    void activeCb();
    trackingClient* client_;
    bool is_target_found;
    geometry_msgs::Twist wander_angle_vel_;
    geometry_msgs::Twist wander_linear_vel_;
    geometry_msgs::PoseStamped target_pose_;
    double angle_vel;
    double linear_vel;
    void wandering();
    void vel_subCb(const geometry_msgs::TwistConstPtr& msg);
    void odom_subCb(const nav_msgs::OdometryConstPtr& msg);
    bool is_stop;
    bool is_reached;
    double bh_loop_freq;
    bool forward_flag;
    bool is_spinning;
    double forward_distance_;
    bool is_tracking;

    static int linear_vel_pub_num_;
    // geometry_msgs::PoseStamped tf_fuc(const geometry_msgs::PoseStampedConstPtr& msg);
};

#endif //!SEARCH_TARGET_H