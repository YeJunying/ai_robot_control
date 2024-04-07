#ifndef AWAIT_H
#define AWAIT_H

#include <iostream>
#include <string>
#include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/Image.h>
// #include <actionlib/client/simple_action_client.h>
// #include <ai_robot_control/trackingAction.h>
// #include <ai_robot_waving/SendLocalTarget.h>
#include <ai_robot_waving/ToggleModule.h>
#include <behaviortree_cpp_v3/action_node.h>

class Await : public BT::StatefulActionNode
{
public:
    Await(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh);
    ~Await();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    //处理接口参数
    static BT::PortsList providedPorts() 
    {
        BT::PortsList ports_list;
        return ports_list;
    }


private:
    bool active_flag_;
    ros::ServiceServer activate_toggle_srv_;
    
    bool activateCb(ai_robot_waving::ToggleModule::Request& req, ai_robot_waving::ToggleModule::Response& resp);
};




#endif //!AWAIT_H