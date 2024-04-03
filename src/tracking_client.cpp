#include "tracking_client.h"

Tracking_client::Tracking_client(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::SyncActionNode(name, config) 
{
    ros::NodeHandle nh_(root_nh);

    active_.request.active = false;
    tracking_toggle_srv_ = nh_.serviceClient<ai_robot_tracking::ToggleModule>("/ai_robot/tracking/toggle");
    pose_sub_ = nh_.subscribe("/ai_robot/control/target_pose", 1, &Tracking_client::goal_sendCb, this);
    point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

}

Tracking_client::~Tracking_client() {}

BT::NodeStatus Tracking_client::tick()
{
    static trackingClient client_("/ai_robot_control/tracking_", true);
    client_.waitForServer();

    if(client_.getState() != actionlib::SimpleClientGoalState::ACTIVE)
    {
        //激活跟踪模块中转节点
        active_.request.active = true;
        bool flag = tracking_toggle_srv_.call(active_);

        //向action服务端发送目标
        client_.sendGoal(goal.goal, 
                        boost::bind(&Tracking_client::doneCb, this), 
                        boost::bind(&Tracking_client::activeCb, this), 
                        boost::bind(&Tracking_client::feedbackCb, this, _1));
    
        if(client_.getState() == actionlib::SimpleClientGoalState::ACTIVE && flag)
        {
            return BT::NodeStatus::SUCCESS;
        }

    }
    else
    {
        //取消目标
        client_.cancelAllGoals();

        //挂起跟踪模块中转节点
        active_.request.active = false;
        bool flag = tracking_toggle_srv_.call(active_);
        if(client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED && flag)
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
       
}

void Tracking_client::feedbackCb(const ai_robot_control::trackingActionFeedbackConstPtr& msg)
{
    geometry_msgs::PoseStamped pose_;

    pose_ = msg->feedback.pose;

    point_pub_.publish(pose_);
}
    
void Tracking_client::goal_sendCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg)
{
    goal.goal.target = msg->target;
    goal.goal.target_image = msg->target_image;
}

void Tracking_client::doneCb() {}
void Tracking_client::activeCb() {}