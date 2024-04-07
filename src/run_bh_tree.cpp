#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include "tracking_client.h"
#include "waving.h"
#include "waving_detected.h"
#include "await.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_bh_tree");

    ros::NodeHandle nh;
    ros::NodeHandle bh_tree_nh("~");

    double bh_loop_freq;
    bh_tree_nh.param("bh_loop_freq", bh_loop_freq, 5.0);
    
    BT::BehaviorTreeFactory factory;

    //注册待机行为节点
    BT::NodeBuilder builder_await = [&nh](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<Await>(name, config, nh);
    };
    factory.registerBuilder<Await>("await", builder_await);

    //注册招手识别行为节点
    BT::NodeBuilder builder_waving = [&nh](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<Waving>(name, config, nh);
    };
    factory.registerBuilder<Waving>("waving", builder_waving);

    //注册招手识别结果检测行为节点
    BT::NodeBuilder builder_waving_detected = [&nh](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<Waving_detected>(name, config, nh);
    };
    factory.registerBuilder<Waving_detected>("waving_detected", builder_waving_detected);

    //注册行人跟踪行为节点
    BT::NodeBuilder builder_tracking = [&nh](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<Tracking_client>(name, config, nh);
    };
    factory.registerBuilder<Tracking_client>("tracking", builder_tracking);
    
    //读取n行为树xml文件，生成行为树
    std::string file_path = bh_tree_nh.param("file_path", std::string(" "));
    auto tree = factory.createTreeFromFile(file_path);
    // auto tree = factory.createTreeFromFile("./main_tree.xml");
    //运行行为树
    ros::Rate loop_rate(bh_loop_freq); //设置行为树运行频率
    while(ros::ok())
    {
        BT::NodeStatus status = tree.tickRoot();
        std::cout<< status << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}