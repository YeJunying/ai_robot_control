#include "search_target.h"

int Search_target::linear_vel_pub_num_ = 0;

Search_target::Search_target(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::StatefulActionNode(name, config)
{
    ros::NodeHandle nh_(root_nh);
    ros::NodeHandle private_nh("~");

    srand(time(0));

    private_nh.param("angle_vel", angle_vel, 20.0); 
    private_nh.param("linear_vel", linear_vel, 0.5);
    private_nh.param("bh_loop_freq", bh_loop_freq, 5.0);

    client_ = new trackingClient("/ai_robot_control/tracking_", true);
    client_->waitForServer();

    is_target_found = false;
    is_stop = false;
    is_reached = true;
    forward_flag = false;
    is_spinning = false;
    is_tracking = false;
    wander_angle_vel_.linear.x = 0.0;
    wander_angle_vel_.linear.y = 0.0;
    wander_angle_vel_.angular.z = M_PI*angle_vel/180;

    wander_linear_vel_.linear.x = linear_vel;
    wander_linear_vel_.linear.y = 0.0;
    wander_linear_vel_.angular.z = 0.0;

    vel_sub_ = nh_.subscribe("cmd_vel", 1, &Search_target::vel_subCb, this);
    odom_sub_ = nh_.subscribe("odom", 1, &Search_target::odom_subCb, this);

    target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

Search_target::~Search_target()
{
    if(client_ != nullptr)
        delete client_;
}

BT::NodeStatus Search_target::onStart()
{
    cv::Mat image = cv::imread("/home/ye/ai_robot_ws/src/ai_robot_control/config/target.jpg");
    if(image.empty())
    {
        ROS_ERROR("Read the picture failed!");
    }
    // std_msgs::Header header_;
    // header_.stamp = ros::Time::now();
    // sensor_msgs::ImagePtr target_ = cv_bridge::CvImage(header_, "bgr8", image).toImageMsg();
    // goal.target_image = *target_;
    cv_bridge::CvImage cvi;
    cvi.header.stamp = ros::Time::now();
    cvi.encoding = "bgr8";
    cvi.image = image;
    cvi.toImageMsg(goal.target_image);

    if(client_->getState() != actionlib::SimpleClientGoalState::ACTIVE)
    {
        client_->sendGoal(goal, 
                        boost::bind(&Search_target::doneCb, this), 
                        boost::bind(&Search_target::activeCb, this), 
                        boost::bind(&Search_target::feedbackCb, this, _1));
    }

    if(!is_tracking)
    {
        if(is_target_found)
        {
            ROS_INFO("Find the target.");
            target_pub_.publish(target_pose_);
            client_->cancelGoal();
            is_target_found = false;
            is_tracking = true;
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            wandering();
            return BT::NodeStatus::RUNNING;
        }
    }
    else
    {
        if(is_stop)
        {
            is_tracking = false;
            return BT::NodeStatus::SUCCESS;
        }
        else 
        {
            return BT::NodeStatus::RUNNING;
        }
    }
    
}

BT::NodeStatus Search_target::onRunning()
{
    if(!is_tracking)
    {
        if(is_target_found)
        {
            ROS_INFO("Find the target.");
            target_pub_.publish(target_pose_);
            client_->cancelGoal();
            is_target_found = false;
            is_tracking = true;
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            wandering();
            return BT::NodeStatus::RUNNING;
        }
    }
    else
    {
        if(is_stop)
        {
            is_tracking = false;
            ROS_INFO("Reach the destination.");
            return BT::NodeStatus::SUCCESS;
        }
        else 
        {
            ROS_INFO("Going to the destination.");
            return BT::NodeStatus::RUNNING;
        }
    }
}

void Search_target::onHalted()
{
    ROS_INFO("Node is aborted");
}

void Search_target::feedbackCb(const ai_robot_control::trackingFeedbackConstPtr& msg)
{
    target_pose_ = msg->pose;
    is_target_found = true;
}

void Search_target::doneCb() {}
void Search_target::activeCb() {}

void Search_target::vel_subCb(const geometry_msgs::TwistConstPtr& msg)
{
    bool flag = msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0;
    if(flag)
        is_stop = true;
    else
        is_stop = false;

//     if(msg->linear.x == 0 && msg->linear.y == 0)
//         is_reached = true;
}

void Search_target::odom_subCb(const nav_msgs::OdometryConstPtr& msg)
{
    double tmp = msg->pose.pose.orientation.w * msg->pose.pose.orientation.z 
            + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y;
    // ROS_INFO("tmp = %f", tmp);
    if((abs(tmp)<0.04) && is_spinning)
    {
        forward_flag = true;
        is_spinning = false;
    } 
}

void Search_target::wandering()
{
    if(is_reached)
    {
        if(!forward_flag)
        {
            vel_pub_.publish(wander_angle_vel_);
            // angle_vel_pub_num_++;
            ROS_INFO("Spinning.");
            is_spinning = true;
        }
        else
        {
            forward_distance_ = rand() / double(RAND_MAX);
            vel_pub_.publish(wander_linear_vel_);
            ROS_INFO("Going forward");
            is_reached = false;
        }
    }
    else
    {
        ROS_INFO("forward_distance_ = %f, %f", linear_vel_pub_num_*linear_vel/bh_loop_freq, forward_distance_);
        if(linear_vel_pub_num_*linear_vel/bh_loop_freq < forward_distance_)
        {
            linear_vel_pub_num_++; 
        }
        else
        {
            geometry_msgs::Twist zero_vel_;
            zero_vel_.linear.x = 0.0;
            zero_vel_.linear.y = 0.0;
            zero_vel_.angular.z = 0.0;
            vel_pub_.publish(zero_vel_);
            forward_flag = false;
            linear_vel_pub_num_ = 0;
            is_reached = true;
        }
    }
}
