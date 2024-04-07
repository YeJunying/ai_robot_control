#ifndef WAVING_H
#define WAVING_H

#include <iostream>
#include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/Image.h>
// #include <actionlib/client/simple_action_client.h>
// #include <ai_robot_control/trackingAction.h>
#include <ai_robot_waving/SendLocalTarget.h>
#include <ai_robot_waving/SendLocalTargetRequest.h>
#include <ai_robot_waving/ToggleModule.h>
#include <behaviortree_cpp_v3/action_node.h>

class Waving : public BT::SyncActionNode
{
public:
    Waving(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh);
    ~Waving();
    BT::NodeStatus tick() override;

    //处理接口参数
    static BT::PortsList providedPorts() 
    {
        BT::PortsList ports_list;
        return ports_list;
    }


private:
    ros::Publisher target_pub_;
    ros::ServiceClient waving_toggle_srv_;
    ros::ServiceServer target_get_;
    ai_robot_waving::ToggleModule active_;
    ai_robot_waving::SendLocalTargetRequest target_pose_;
    // bool goal_pubCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg);
    bool goal_pubCb(ai_robot_waving::SendLocalTarget::Request& req, ai_robot_waving::SendLocalTarget::Response& res);
    bool waving_start_flag_;

    ros::ServiceServer server_;
};

#endif //!WAVING_H