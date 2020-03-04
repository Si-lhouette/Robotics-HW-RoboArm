
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
 
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    float pos[6];
    for(int i = 0; i < 6;i++){
        pos[i]=msg->position[i];
    }
    

    ROS_INFO("Recive: [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]);
 
}
 
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "subs");
 
 
  ros::NodeHandle n;
 
 
  ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 10, jointstatesCallback);
 
 
  ros::spin();
 
 
  return 0;
}