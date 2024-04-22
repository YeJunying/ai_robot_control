#ifndef SHOW_H
#define SHOW_H

#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <geometry_msgs/Twist.h>
#include <behaviortree_cpp_v3/action_node.h>

struct Instruction
{
    uint32_t code;
    uint32_t paramters_size;
    uint32_t type;
};

class ControlRobot
{
public:
    ControlRobot();
    ~ControlRobot();
    void send_data(uint32_t code, uint32_t paramters_size, uint32_t type);

private:
    int udp_socket;
    struct sockaddr_in send_addr;
};



class Show : public BT::StatefulActionNode
{
public:
    Show(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh);
    ~Show();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
   
    //处理接口参数
    static BT::PortsList providedPorts() 
    {
        BT::PortsList ports_list;
        // ports_list.insert(BT::InputPort<int>("show_flag"));
        return ports_list;
    }


private:
    ControlRobot* robot_;
    // bool success_flag;
    // ros::Subscriber vel_sub_;
    // bool is_stop;
    double bh_loop_freq;
    int clock_num_;
    // bool start_show_flag;
    // void vel_subCb(const geometry_msgs::TwistConstPtr& msg);
};

#endif //!SHOW_H
