#include "tracking_server.h"

Tracking_server::Tracking_server()
{
    nh_ = ros::NodeHandle();
    ros::NodeHandle private_nh("~");
    private_nh.param("control_freq", control_freq_, 10.0);

    active_.request.active = false;
    tracking_toggle_srv_ = nh_.serviceClient<ai_robot_tracking::ToggleModule>("/ai_robot/tracking/toggle");

    target_send_srv_ = nh_.serviceClient<ai_robot_tracking::ReceiveTarget>("/ai_robot/tracking/target");

    target_pose_sub_= nh_.subscribe("/ai_robot/tracking/target_pose", 1000, &Tracking_server::target_poseCb, this);
    // target_image_pub_ = nh_.advertise<sensor_msgs::Image>("/ai_robot/control/target_image", 1);

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
    ai_robot_tracking::ReceiveTarget goal_;
    goal_.request.target_image = tracking_goal->target_image;
    target_send_srv_.call(goal_);
    // target_image_pub_.publish(goal_.request.target_image);

    //激活跟踪模块中转节点
    active_.request.active = true;
    bool flag = tracking_toggle_srv_.call(active_);

    while(ros::ok)
    {
        if(as_->isPreemptRequested())
        {
            if(as_->isNewGoalAvailable())
            {
                ai_robot_tracking::ReceiveTarget new_goal_;
                new_goal_.request.target_image = as_->acceptNewGoal()->target_image;
                target_send_srv_.call(new_goal_);
                ROS_INFO("Change to a new goal.");
            }
            else
            {
                //挂起跟踪模块中转节点
                active_.request.active = false;
                bool flag = tracking_toggle_srv_.call(active_);
                
                as_->setPreempted();
                ROS_INFO("Cancle the goal.");
                return;
            }
        }
        ROS_INFO("Tracking is running.");
        // std::cout<<as_->isActive()<<std::endl;
        r.sleep();
    }
}

void Tracking_server::target_poseCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    target_pose.pose.pose = msg->pose;
    as_->publishFeedback(target_pose);
}