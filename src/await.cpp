#include "await.h"

Await::Await(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::StatefulActionNode(name, config)
{
    ros::NodeHandle nh_(root_nh);
    active_flag_ = false;

    activate_toggle_srv_ = nh_.advertiseService("/ai_robot_control/await_toggle", &Await::activateCb, this);
}

Await::~Await() { }

BT::NodeStatus Await::onStart()
{
    if(active_flag_)
    {
        active_flag_ = false;
        ROS_INFO("Start."); 
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Await::onRunning()
{
    if(active_flag_)
    {
        active_flag_ = false;
        ROS_INFO("Start."); 
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void Await::onHalted()
{
    ROS_INFO("Node is aborted");
}

bool Await::activateCb(ai_robot_waving::ToggleModule::Request& req, ai_robot_waving::ToggleModule::Response& resp)
{
    active_flag_ = req.active;
    return true;
}