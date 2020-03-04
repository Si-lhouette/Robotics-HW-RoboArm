//
// Created by kjwang on 2020/2/19.
//
//本示例程序供大家学习probot_anno的位置及速度控制如何编程，展示了：
//1.如何使用ikfast求解器帮我们去计算机械臂逆解;
//2.如何创建位置控制数据发布者，并发布位置数据（速度控制也一样的，把名字换换而已），控制gazebo的机械臂动起来。
//

#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <tf/tf.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"

using namespace ikfast_kinematics_plugin;
using namespace std;
int main(int argc, char **argv) {

    bool ret;
    //节点初始化
    ros::init(argc, argv, "control_example");
    //创建节点句柄对象
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    //创建发布者对象，用于发布位置信息，
    //位置控制的话题名为："/probot_anno/arm_pos_controller/command"
    //速度控制的话题名为："/probot_anno/arm_vel_controller/command"
    //发送的数据类型均为：std_msgs::Float64MultiArray，
    //它的创建与赋值方法在下面有，一组6个浮点数
    ros::Publisher pos_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);
    
    //以下为ikfast运动学求解器的使用，要想仔细了解，自己看它的类的头文件，注释只解释必要的内容
    //创建运动学类实例ik
    ikfast_kinematics_plugin::IKFastKinematicsPlugin ik;
    //初始化对象，参数是机械臂模型相关的一些内容，link_6即末端连杆，它的坐标系所在位置就是末端中心点，与PPT图片里的一致
    ret = ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link","link_6",0.001);
    // 设置机器人终端的目标位置
    //geometry_msgs::Pose 这个类就是用来装机械臂位置姿态的，
    // 它的position成员里装3个坐标，orientation成员里装x,y,z,w，这是四元数位姿表示法
    //当x=1时，末端处于初始位姿，对应的仿真机器人欧拉角姿态为（1.57, 0, 0）
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.2;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.2007 - 0.022;
    

    tf::Quaternion q;
    q.setRPY(1.57, -1.57, 0);

    cout<<q[0]<<",";
    cout<<q[1]<<",";
    cout<<q[2]<<",";
    cout<<q[3]<<endl;

    target_pose.orientation.x = q[0];
    target_pose.orientation.y = q[1];
    target_pose.orientation.z = q[2];
    target_pose.orientation.w = q[3];


    
    //pose1为vector<geometry_msgs::Pose>类型变量，装入上述位姿信息
    std::vector<geometry_msgs::Pose> pose1;
    pose1.push_back(target_pose);

    //设定关节角参考初值，这是求解函数要求的一个参数，默认为0即可，是vector<double>类型
    std::vector<double> seed1;
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);

    //创建用于保存逆解的变量，它是vector矩阵，vector<std::vector<double>>类型，因为会解出不止一组解装在里面
    //内容六关节角度，每个角度是浮点数，为弧度制
    std::vector<std::vector<double>> sol_rad;
    //这是求解函数要求的一个参数，具体功能不明
    kinematics::KinematicsResult kinematic_result;

    //计算逆解
    //参数表：（末端位姿，参考关节角初值，关节角，......）
    ret = ik.getPositionIK(pose1, seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());

    if(ret)//若求解成功，输出结果
    {
        std::cout << sol_rad.size() << " IK solved successfully." << endl;
        //角度制计算，并输出每一组计算结果
        for (int q = 0; q < sol_rad.size(); q++)
        {
            for (int i = 0; i < 6; i++)
            {
                cout << sol_rad[q][i]<<" ";
            }
            cout << endl;
        }
    }

    //初始化publish的变量并初始化6个值
    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);
    //double targetPose[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};//运动的起点位置
    //double targetPose[6] = {0, -1.5708, 1.5708, 0, 1.5708, 0};//运动的起点位置

    //为要发送的变量装入解出的六关节坐标
    //如果是要发送速度，那也是类似于这样，装入6个关节角速度值（弧度制）即可
    init_pos.data.at(0) = sol_rad[0][0];
    init_pos.data.at(1) = sol_rad[0][1];
    init_pos.data.at(2) = sol_rad[0][2];
    init_pos.data.at(3) = sol_rad[0][3];
    init_pos.data.at(4) = sol_rad[0][4];
    init_pos.data.at(5) = sol_rad[0][5];

    //发送出去，若成功，机械臂状态会改变
    pos_pub.publish(init_pos);
    ROS_INFO_STREAM("published");
    ros::spin();
    return 0;
}
