#include "tracking_server.h"

Tracking_server::Tracking_server()
{
    nh_ = ros::NodeHandle();
    ros::NodeHandle private_nh("~");
    private_nh.param("control_freq", control_freq_, 10.0);

    target_send_srv_ = nh_.serviceClient<ai_robot_waving::SendLocalTarget>("/ai_robot/tracking/target");

    target_pose_sub_= nh_.subscribe("/ai_robot/tracking/target_pose", 1000, &Tracking_server::target_poseCb, this);

    trackingServer as_(nh_, "/ai_robot_control/tracking_", boost::bind(&Tracking_server::executeCb, _1, &as_), false);
    // trackingServer as_(nh_, "/ai_robot_control/tracking_", [this](auto& goal){executeCb(goal); }, false);
    as_.start();
}

Tracking_server::~Tracking_server(){}

void Tracking_server::executeCb(const ai_robot_control::trackingActionGoalConstPtr& tracking_goal, trackingServer* as)
{
    ros::Rate r(control_freq_);

    //向中转节点发送目标截图
    target_send_srv_.call(*tracking_goal);

    while(true)
    {
        as->publishFeedback(target_pose.feedback);
        r.sleep();
    }
}

void Tracking_server::target_poseCb(const ai_robot_control::trackingActionFeedbackConstPtr& msg)
{
    target_pose = *msg;
}