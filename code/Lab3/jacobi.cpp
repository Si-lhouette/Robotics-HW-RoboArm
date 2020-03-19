/* Jaccobi矩阵速度传递 */

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "jacobi.h"
#include<ros/time.h>

ros::Publisher vel_pub;

VectorXd nowRobotAngle(6);  //当前关节角

/* 回调函数：接收实时关节角 */ 
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for(int i = 0; i < 6;i++){
        nowRobotAngle(i)=msg->position[i];
    }

    cout<<"nowRobotAngle: "<<nowRobotAngle.transpose()<<endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "jacobi");
    ros::NodeHandle nh;
    ros::Rate loopHZ(100);
    
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/probot_anno/joint_states", 10, jointstatesCallback);

    vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);

    /* 初始化 */
    InitialDH2();

    nowRobotAngle << 0,0,0,0,0,0;
    
    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);


    VectorXd endv(6);

    ros::Time begin=ros::Time::now();
    double t;
    double nowv;
    double a1 = 0.005;
    double a2 = 0.004;

    /* 开始计时速度控制 */
    while (ros::ok()){
        ros::Time nowtime=ros::Time::now();
        t = nowtime.toSec() - begin.toSec();    //获取当前时刻

        cout<<endl<<"-------------------"<<"t: "<<t<<endl;
        /* 计算不同时刻期望末端速度 */ 
        if(t < 4.0){
            cout<<"step 01"<<endl;
            nowv = a1 * t;
            endv << nowv,0,-nowv,0,0,0;
        }else if(t >=4 && t < 7.0){
            cout<<"step 02"<<endl;            
            endv << 0.02,0,-0.02,0,0,0;           
        }else if(t >=7 && t < 12.0){
            cout<<"step 03"<<endl;            
            nowv = 0.02 - a2 * (t-7.0);
            endv << nowv,0,-nowv,0,0,0;
        }else{
            cout<<"step 04"<<endl;            
            endv << 0,0,0,0,0,0;
        }

        ros::spinOnce();    //运行回调函数
        velcontrol(endv, nowRobotAngle, init_pos);  //读入实时关节角，由期望的末端速度计算关节速度
        vel_pub.publish(init_pos);
        
        loopHZ.sleep();
    }
    


    return 0;
}