#ifndef WAVING_DETECTED_H
#define WAVING_DETECTED_H

#include <iostream>
#include <string>
#include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/Image.h>
// #include <actionlib/client/simple_action_client.h>
// #include <ai_robot_control/trackingAction.h>
#include <ai_robot_waving/SendLocalTarget.h>
// #include <ai_robot_waving/ToggleModule.h>
#include <behaviortree_cpp_v3/action_node.h>

class Waving_detected : public BT::SyncActionNode
{
public:
    Waving_detected(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh);
    ~Waving_detected();
    BT::NodeStatus tick() override;

    //处理接口参数
    static BT::PortsList providedPorts() 
    {
        BT::PortsList ports_list;
        return ports_list;
    }


private:
    ros::Subscriber target_sub_;
    bool waving_detected_flag;
    void detectedCb(const ai_robot_waving::SendLocalTargetRequestConstPtr& msg);

};

#endif //!WAVING_DETECTED_H