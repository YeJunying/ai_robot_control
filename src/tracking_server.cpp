#include "tracking_server.h"

Tracking_server::Tracking_server()
{
    nh_ = ros::NodeHandle();
    ros::NodeHandle private_nh("~");
    private_nh.param("control_freq", control_freq_, 10.0);

    target_send_srv_ = nh_.serviceClient<ai_robot_waving::SendLocalTarget>("/ai_robot/tracking/target");

    target_pose_sub_= nh_.subscribe("/ai_robot/tracking/target_pose", 1000, &Tracking_server::target_poseCb, this);

    // trackingServer as_(nh_, "/ai_robot_control/tracking_", boost::bind(&Tracking_server::executeCb, _1, &as_), false);
    as_ = new trackingServer(nh_, "/ai_robot_control/tracking_", [this](auto& goal){ executeCb(goal); }, false);
    as_->start();
}

Tracking_server::~Tracking_server()
{
    if (as_ != nullptr) 
        delete as_;
}

void Tracking_server::executeCb(const ai_robot_control::trackingGoalConstPtr& tracking_goal)
{
    ros::Rate r(control_freq_);

    //向中转节点发送目标截图
    ai_robot_waving::SendLocalTarget goal_;
    goal_.request.target = tracking_goal->target;
    goal_.request.target_image = tracking_goal->target_image;
    target_send_srv_.call(goal_);

    while(true)
    {
        as_->publishFeedback(target_pose);
        r.sleep();
    }
}

void Tracking_server::target_poseCb(const ai_robot_control::trackingFeedbackConstPtr& msg)
{
    target_pose.pose = msg->pose;
}