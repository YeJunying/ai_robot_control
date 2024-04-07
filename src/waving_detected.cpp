#include "waving_detected.h"

Waving_detected::Waving_detected(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::SyncActionNode(name, config) 
{
    ros::NodeHandle nh_(root_nh);

    waving_detected_flag = false;
    target_sub_ = nh_.subscribe("/ai_robot/control/target_pose", 1, &Waving_detected::detectedCb, this);
}

Waving_detected::~Waving_detected() {}

BT::NodeStatus Waving_detected::tick()
{
    if(waving_detected_flag)
    {
        waving_detected_flag = false;
        ROS_INFO("Detected waving.");
    }
    return BT::NodeStatus::SUCCESS;
}

void Waving_detected::detectedCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg)
{
    waving_detected_flag = true;
}