#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include "tracking_client.h"
#include "waving.h"
#include "waving_detected.h"
#include "await.h"
#include "show.h"
#include "search_target.h"

const double X = 0.24376;
const double Y = -0.012728;
const double Z = 0.475118;
const double ROLL = 0.0;
const double PITCH = 0.0;
const double YAW =0.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_bh_tree");

    ros::NodeHandle nh;
    ros::NodeHandle bh_tree_nh("~");

    tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped ts_;

    //相机到底盘的静态坐标变换
    ts_.header.seq = 100;
    ts_.header.stamp = ros::Time::now();
    ts_.header.frame_id = "base_link";
    ts_.child_frame_id = "camera_base_link";

    ts_.transform.translation.x = X;
    ts_.transform.translation.y = Y;
    ts_.transform.translation.z = Z;

    tf2::Quaternion qtn;
    qtn.setRPY(ROLL, PITCH, YAW);
    ts_.transform.rotation.x = qtn.getX();
    ts_.transform.rotation.y = qtn.getY();
    ts_.transform.rotation.z = qtn.getZ();
    ts_.transform.rotation.w = qtn.getW();

    broadcaster.sendTransform(ts_);

    //获取行为树运行频率参数
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

    //注册表演行为节点
    BT::NodeBuilder builder_show = [&nh](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<Show>(name, config, nh);
    };
    factory.registerBuilder<Show>("show", builder_show);

    //注册目标搜寻节点
    BT::NodeBuilder builder_search_target = [&nh](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<Search_target>(name, config, nh);
    };
    factory.registerBuilder<Search_target>("search_target", builder_search_target);
    
    //读取行为树xml文件，生成行为树
    std::string file_path = bh_tree_nh.param("file_path", std::string(" "));
    auto tree = factory.createTreeFromFile(file_path);
    // auto tree = factory.createTreeFromFile("./main_tree.xml");
    //运行行为树
    ros::Rate loop_rate(bh_loop_freq); //设置行为树运行频率
    while(ros::ok())
    {
        BT::NodeStatus status = tree.tickRoot();
        // std::cout<< status << std::endl;
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}