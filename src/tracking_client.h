#ifndef TRACKING_CLIENT_H
#define TRACKING_CLIENT_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
// #include <sensor_msgs/Image.h>
#include <actionlib/client/simple_action_client.h>
#include <ai_robot_control/trackingAction.h>
#include <ai_robot_waving/SendLocalTarget.h>
#include <ai_robot_tracking/ToggleModule.h>
#include <behaviortree_cpp_v3/action_node.h>

typedef actionlib::SimpleActionClient<ai_robot_control::trackingAction> trackingClient; 

class Tracking_client : public BT::SyncActionNode
{
public:
    Tracking_client(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh);
    ~Tracking_client();
    BT::NodeStatus tick() override;

    //处理接口参数
    static BT::PortsList providedPorts() 
    {
        BT::PortsList ports_list;
        return ports_list;
    }


private:
    ros::Subscriber pose_sub_;
    ros::Publisher point_pub_;
    ros::Subscriber vel_sub_;
    ros::Publisher vel_pub_;
    ros::Subscriber tracking_pose_sub_;
    ai_robot_control::trackingGoal goal;
    // ros::ServiceClient tracking_toggle_srv_;
    // ai_robot_tracking::ToggleModule active_;
    void feedbackCb(const ai_robot_control::trackingFeedbackConstPtr& msg);
    void goal_sendCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg);
    void doneCb();
    void activeCb();
    void vel_subCb(const geometry_msgs::TwistConstPtr& msg);
    void tracking_poseCb(const geometry_msgs::PoseStampedConstPtr& msg);
    trackingClient* client_;
    geometry_msgs::PoseStamped pose_;
    bool is_feedback;
    bool is_tracking;
    double angle_vel;
    geometry_msgs::Twist spin_vel_;
};

#endif //!TRACKING_CLIENT_H