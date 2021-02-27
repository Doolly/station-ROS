#include "ros/ros.h"
#include "w_station/MyMessage.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;
    ros::Publisher w_station_pub = n.advertise<w_station::MyMessage>("my_message", 1000);
    ros::Rate loop_rate(10);

    int count = 0;

    while (ros::ok())
    {
        w_station::MyMessage msg;

        msg.message = "Hello! World";
				msg.count = count;

        ROS_INFO("%s %ld", msg.message.c_str(), msg.count);
        w_station_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}