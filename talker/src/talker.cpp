#include "ros/ros.h"
#include "std_msgs/Time.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Time>("/time", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Time msg;
    msg.data = ros::Time::now();
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
