#include "tracking_server.h"

Tracking_server::Tracking_server(tf2_ros::Buffer& tf)
    : tf_(tf)
{
    nh_ = ros::NodeHandle();
    ros::NodeHandle private_nh("~");
    private_nh.param("control_freq", control_freq_, 10.0);
    private_nh.param("tracking_distance", tracking_distance_, 0.5);

    active_.request.active = false;
    feedback_flag = false;
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

        if(feedback_flag)
        {
            as_->publishFeedback(target_pose);
            feedback_flag = false;
        }
        ROS_INFO("Tracking is running.");
        // std::cout<<as_->isActive()<<std::endl;
        r.sleep();
    }
}

void Tracking_server::target_poseCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    geometry_msgs::PoseStamped point_camera;

    point_camera.pose = msg->pose;
    if(point_camera.pose.position.x <= tracking_distance_)
        point_camera.pose.position.x = 0;
    else
        point_camera.pose.position.x -= tracking_distance_;

    if(std::abs(point_camera.pose.position.y) < 0.07)
        point_camera.pose.position.y = 0;

    point_camera.header.frame_id = "camera_link";
    point_camera.header.stamp = ros::Time::now();
    geometry_msgs::TransformStamped tfs;
    try
    {
        tfs = tf_.lookupTransform("map", point_camera.header.frame_id, ros::Time(point_camera.header.stamp), ros::Duration(0.1));

        target_pose.pose = tf_.transform(point_camera, "map");
        target_pose.pose.pose.position.z = 0;
        target_pose.pose.pose.orientation = point_camera.pose.orientation;

        if(point_camera.pose.position.x != 0)
            feedback_flag = true;
        
    }
    catch(tf2::LookupException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    
}