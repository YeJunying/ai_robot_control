#include "tracking_server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_server_node");

    Tracking_server server_;

    ros::spin();

    return 0;
}