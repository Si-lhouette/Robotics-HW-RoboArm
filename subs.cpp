
#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

using namespace std;
using namespace Eigen;


 
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    float pos[6];
    for(int i = 0; i < 6;i++){
        pos[i]=msg->position[i];
    }
    

    //ROS_INFO("Recive: [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]);
 
}

void LinkStatesCallBack(const gazebo_msgs::LinkStatesConstPtr& msg){
	geometry_msgs::Pose Link6msg = msg->pose[7];
	Vector3d Link6Pose;
	Link6Pose << Link6msg.position.x, Link6msg.position.y, Link6msg.position.z;
    cout<<endl<<"-------------------------"<<endl;
    cout<<msg->name[7]<<endl;
    cout<<"Pose: "<<endl<<Link6Pose.transpose()<<endl;

    // Quaterniond Link6q(Link6msg.orientation.w, Link6msg.orientation.x, Link6msg.orientation.y, Link6msg.orientation.z);
	// Vector3d eulerAngle1=Link6q.matrix().eulerAngles(0,1,2);
    // cout << "EulerAngles:"<<endl<<eulerAngle1<<endl<<endl;

    geometry_msgs::Twist Tw6 = msg->twist[7];

    //全局线速度
    cout<<"linear speed: "<<Tw6.linear.x<<", "<<Tw6.linear.y<<", "<<Tw6.linear.z<<endl;
    //全局角速度
    cout<<"angle speed: "<<Tw6.angular.x<<", "<<Tw6.angular.y<<", "<<Tw6.angular.z<<endl;
    

}
 
int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "subs");


    ros::NodeHandle n;
    ros::Rate loop_rate(3);

    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 10, jointstatesCallback);

    ros::Subscriber sublink6 = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, LinkStatesCallBack);

    cout<<fixed<<setprecision(3);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    

    return 0;
}