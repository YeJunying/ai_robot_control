#include "waving.h"

bool Waving::waving_start_flag_ = false;

Waving::Waving(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::SyncActionNode(name, config) 
{
    ros::NodeHandle nh_(root_nh);
    // waving_start_flag_ = false;
    active_.request.active = false;
    waving_toggle_srv_ = nh_.serviceClient<ai_robot_waving::ToggleModule>("/ai_robot/waving/toggle");
    target_pub_ = nh_.advertise<ai_robot_waving::SendLocalTargetRequest>("/ai_robot/control/target_pose", 1);
    server_ = nh_.advertiseService("/ai_robot/waving/target_pose", &Waving::goal_pubCb, this);
}

Waving::~Waving() {}

BT::NodeStatus Waving::tick()
{
    if(!Waving::waving_start_flag_)
    {
        //激活招手识别
        active_.request.active = true;
        waving_toggle_srv_.waitForExistence();
        bool flag_ = waving_toggle_srv_.call(active_);

        if(flag_)
        {
            Waving::waving_start_flag_ = true;
            ROS_INFO("Start waving successfully.");
        }
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        //挂起招收识别
        active_.request.active = false;
        bool flag = waving_toggle_srv_.call(active_);

        Waving::waving_start_flag_ = false;

        if(flag)
        {
            ROS_INFO("Close waving successfully.");
        }
        return BT::NodeStatus::SUCCESS;
    }
}

// bool Waving::goal_pubCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg)
// {
//     target_pose_.target = msg->target;
//     target_pose_.target_image = msg->target_image;
//     target_pub_.publish(target_pose_);
//     waving_start_flag_ = true;
//     return true;
// }
bool Waving::goal_pubCb(ai_robot_waving::SendLocalTarget::Request& req, ai_robot_waving::SendLocalTarget::Response& res)
{
    target_pose_.target = req.target;
    target_pose_.target_image = req.target_image;
    target_pub_.publish(target_pose_);
    // waving_start_flag_ = true;
    return true;
}