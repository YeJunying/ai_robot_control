#include "waving.h"

Waving::Waving(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::SyncActionNode(name, config) 
{
    ros::NodeHandle nh_(root_nh);
    waving_start_flag_ = false;
    active_.request.active = false;
    waving_toggle_srv_ = nh_.serviceClient<ai_robot_waving::ToggleModule>("/ai_robot/waving/toggle");
    target_pub_ = nh_.advertise<ai_robot_waving::SendLocalTargetRequest>("/ai_robot/control/target_pose", 1);
    ros::ServiceServer server_ = nh_.advertiseService<ai_robot_waving::SendLocalTargetRequest>("/ai_robot/waving/target_pose", &Waving::goal_pubCb);
}

Waving::~Waving() {}

BT::NodeStatus Waving::tick()
{
    if(!waving_start_flag_)
    {
        //激活招手识别
        active_.request.active = true;
        waving_toggle_srv_.call(active_);

        if(waving_start_flag_)
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
    else
    {
        //挂起招收识别
        active_.request.active = false;
        bool flag = waving_toggle_srv_.call(active_);

        waving_start_flag_ = false;

        if(flag)
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
    
}

void Waving::goal_pubCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg)
{
    target_pose_.target = msg->target;
    target_pose_.target_image = msg->target_image;
    target_pub_.publish(target_pose_);
    waving_start_flag_ = true;
}