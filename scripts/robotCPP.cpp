#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("/temp_sensor/activation", 1 );
    std_msgs::Bool msg;
    msg = true;
    chatter_pub.publish(msg);

  ros::Subscriber sub = n.subscribe("/temp_sensor/data", 10, callback);
  ros.spin();
  return 0;
}
void callback(const std_msgs::String::ConstPtr& msg)
{
	cout << msg->data.c_str() << endl;
}
