#include "tracking_client.h"

Tracking_client::Tracking_client(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::SyncActionNode(name, config) 
{
    ros::NodeHandle nh_(root_nh);
    ros::NodeHandle private_nh("~");
    is_feedback = false;
    is_tracking = false;
    private_nh.param("angle_vel", angle_vel, 20.0);

    spin_vel_.linear.x = 0.0;
    spin_vel_.linear.y = 0.0;
    spin_vel_.angular.z = 3*M_PI*angle_vel/180;

    client_ = new trackingClient("/ai_robot_control/tracking_", true);
    client_->waitForServer();

    // active_.request.active = false;
    // tracking_toggle_srv_ = nh_.serviceClient<ai_robot_tracking::ToggleModule>("/ai_robot/tracking/toggle");
    pose_sub_ = nh_.subscribe("/ai_robot/control/target_pose", 1, &Tracking_client::goal_sendCb, this);
    point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    vel_sub_ = nh_.subscribe("cmd_vel", 1, &Tracking_client::vel_subCb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    tracking_pose_sub_ = nh_.subscribe("/ai_robot/tracking/target_pose", 100, &Tracking_client::tracking_poseCb, this);
}

Tracking_client::~Tracking_client() 
{
    if(client_ != nullptr)
        delete client_;
}

BT::NodeStatus Tracking_client::tick()
{
    // static trackingClient client_("/ai_robot_control/tracking_", true);
    // client_.waitForServer();

    if(client_->getState() != actionlib::SimpleClientGoalState::ACTIVE)
    {
        // 向action服务端发送目标
        client_->sendGoal(goal, 
                        boost::bind(&Tracking_client::doneCb, this), 
                        boost::bind(&Tracking_client::activeCb, this), 
                        boost::bind(&Tracking_client::feedbackCb, this, _1));
        
        is_tracking = true;

        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        //取消目标
        client_->cancelGoal();

        is_tracking = false;
       
        return BT::NodeStatus::SUCCESS;
    }
}

void Tracking_client::feedbackCb(const ai_robot_control::trackingFeedbackConstPtr& msg)
{
    is_feedback = true;
    pose_ = msg->pose;

    point_pub_.publish(pose_);
    // ROS_INFO("Get the feedback.");
}
    
void Tracking_client::goal_sendCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg)
{
    goal.target = msg->target;
    goal.target_image = msg->target_image;
}

void Tracking_client::doneCb() 
{
    // ROS_INFO("The goal has been canceled.");
}
void Tracking_client::activeCb() 
{
    // is_goal_sent_ = true;
    // ROS_INFO("Tracking is active and has gotten the goal.");
}

void Tracking_client::vel_subCb(const geometry_msgs::TwistConstPtr& msg)
{
    bool is_stop = (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0);

    if(is_tracking && is_stop && (!is_feedback))
    {
        vel_pub_.publish(spin_vel_);
    }

    if(!is_stop)
    {
        is_feedback = false;
    }
}

void Tracking_client::tracking_poseCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if(msg->pose.position.y != 0)
    {
        if(msg->pose.position.y < 0)
        {
            spin_vel_.angular.z = -3*M_PI*angle_vel/180;
        }
        else
        {
            spin_vel_.angular.z = 3*M_PI*angle_vel/180;
        }
    }
    
}