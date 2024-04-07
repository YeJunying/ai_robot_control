#include "waving_detected.h"

Waving_detected::Waving_detected(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::StatefulActionNode(name, config) 
{
    ros::NodeHandle nh_(root_nh);

    waving_detected_flag = false;
    target_sub_ = nh_.subscribe("/ai_robot/control/target_pose", 1, &Waving_detected::detectedCb, this);
}

Waving_detected::~Waving_detected() {}

BT::NodeStatus Waving_detected::onStart()
{
    if(waving_detected_flag)
    {
        waving_detected_flag = false;
        ROS_INFO("Detected waving.");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Waving_detected::onRunning()
{
    if(waving_detected_flag)
    {
        waving_detected_flag = false;
        ROS_INFO("Detected waving.");
        return BT::NodeStatus::SUCCESS;
    }
    ROS_INFO("Detecting waving.");
    return BT::NodeStatus::RUNNING;
}

void Waving_detected::onHalted()
{
    ROS_INFO("Node is aborted");
}

void Waving_detected::detectedCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg)
{
    waving_detected_flag = true;
}