#include "tracking_server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_server_node");
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    Tracking_server server_(buffer);

    ros::spin();

    return 0;
}