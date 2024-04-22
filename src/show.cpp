#include "show.h"

ControlRobot::ControlRobot()
{
    udp_socket = socket(PF_INET, SOCK_DGRAM, 0);
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr("192.168.1.120");
    send_addr.sin_port = htons(43893);
}

ControlRobot::~ControlRobot() { }

void ControlRobot::send_data(uint32_t code, uint32_t paramters_size, uint32_t type)
{
    Instruction data;
    data.code = code;
    data.paramters_size = paramters_size;
    data.type = type;
    sendto(udp_socket, &data, sizeof(data), 0, (sockaddr*)&send_addr, sizeof(send_addr));

}


Show::Show(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh)
    : BT::StatefulActionNode(name, config)
{
    ros::NodeHandle nh_(root_nh);
    ros::NodeHandle private_nh("~");

    private_nh.param("bh_loop_freq", bh_loop_freq, 5.0);
    
    robot_ = new ControlRobot();
    // success_flag = false;
    // is_stop = false;
    // start_show_flag = false;
    clock_num_ = 0;

    // vel_sub_ = nh_.subscribe("cmd_vel", 1, &Show::vel_subCb, this);
}

Show::~Show() 
{ 
    if(robot_ != nullptr)
        delete robot_;
}

BT::NodeStatus Show::onStart()
{
    ROS_INFO("Start showing.");
    robot_->send_data(0x21010C03, 0, 0); //自主模式
    robot_->send_data(0x21010D05, 0, 0); //原地模式
    robot_->send_data(0x21010202, 0, 0); //趴下
    clock_num_++;
    return BT::NodeStatus::RUNNING;
        
}

BT::NodeStatus Show::onRunning()
{
    if(clock_num_ >= 3*bh_loop_freq && clock_num_ < 3*bh_loop_freq + 1)
    {
        ROS_INFO("Greeting.");
        robot_->send_data(0x21010507, 0, 0); //打招呼
    }
    if(clock_num_ >= 18*bh_loop_freq && clock_num_ <18*bh_loop_freq + 1)
    {
        ROS_INFO("Stand up.");
        robot_->send_data(0x21010202, 0, 0); //起立
        robot_->send_data(0x21010D06, 0, 0); //切换为移动模式
    }
    if(clock_num_ >= 23*bh_loop_freq && clock_num_ < 23*bh_loop_freq + 1)
    {
        ROS_INFO("Ready to move.");
        robot_->send_data(0x21010C03, 0, 0); //自主模式
        clock_num_ = 0;
        return BT::NodeStatus::SUCCESS;
    }

    clock_num_++;
    return BT::NodeStatus::RUNNING;
}

void Show::onHalted()
{
    ROS_INFO("Node is aborted");
}

// void Show::vel_subCb(const geometry_msgs::TwistConstPtr& msg)
// {
//     bool flag = msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0;
//     if(flag)
//         is_stop = true;
//     else
//         is_stop = false;
// }