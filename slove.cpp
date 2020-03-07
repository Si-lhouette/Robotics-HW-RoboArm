//正运动学解算
//Input:DH参数
//Output:T06Matrix
#include <ros/ros.h>
#include <cmath>
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"


using namespace std;
using namespace Eigen;


/* 计算齐次变换矩阵T */
//Input: DH
//Output:Matrix4d& T
void getT(Matrix4d& T, double alpha, double a, double d, double theta){
    T(0,0) = cos(theta);
    T(0,1) = -sin(theta);
    T(0,2) = 0;
    T(0,3) = a;
    T(1,0) = sin(theta)*cos(alpha);
    T(1,1) = cos(theta)*cos(alpha);
    T(1,2) = -sin(alpha);
    T(1,3) = -sin(alpha)*d;
    T(2,0) = sin(theta)*sin(alpha);
    T(2,1) = cos(theta)*sin(alpha);
    T(2,2) = cos(alpha);
    T(2,3) = cos(alpha)*d;
    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;
}

/* 通过旋转矩阵计算固定角XYZ */
//Input: Matrix3d& RMatrix      旋转矩阵
//Output:Vector3d& AngleXYZ     固定角XYZ[roll, pitch, yaw] /rad
void getFixedAngleXYZ(Vector3d& AngleXYZ, Matrix3d& RMatrix){
    double roll, pitch, yaw;
    double pitch_x = sqrt(RMatrix(0,0)*RMatrix(0,0)+RMatrix(1,0)*RMatrix(1,0));
    double pitch_y = -RMatrix(2,0);
    if(pitch_x != 0){
        pitch = atan2(pitch_y, pitch_x);
        yaw = atan2(RMatrix(1,0)/cos(pitch), RMatrix(0,0)/cos(pitch));
        roll = atan2(RMatrix(2,1)/cos(pitch), RMatrix(2,2)/cos(pitch));
    }
    else if(pitch_y > 0){
        pitch = 90*M_PI/180;
        yaw = 0;
        roll = atan2(RMatrix(0,1), RMatrix(1,1));
    }else{
        pitch = -90*M_PI/180;
        yaw = 0;
        roll = -atan2(RMatrix(0,1), RMatrix(1,1));        
    }

    AngleXYZ << roll, pitch, yaw;
}


/* 计算正运动学 */
void Motion(Vector3d& endPose, Vector3d& endAngle , VectorXd& robotAngle){

    //DH参数初始化
    VectorXd alphaDH(6);
    alphaDH << 0, (90.0*M_PI/180.0), 0, (90.0*M_PI/180.0), (-90.0*M_PI/180.0), (90.0*M_PI/180.0);
    VectorXd aDH(6);
    aDH << 0, 0, 0.225, 0, 0, 0;
    VectorXd dDH(6);
    dDH << 0.284, 0, 0, 0.2289, 0, 0;


    //计算末端齐次变换矩阵
    Matrix4d T_all = Matrix4d::Identity();
    Matrix4d T;
    for(int i = 0; i < 6; i++){
        getT(T, alphaDH(i), aDH(i), dDH(i), robotAngle(i));
        T_all *= T;
    }

    getT(T, 0, 0, 0.055, 0);   //第七坐标轴平移
    T_all *= T;
    getT(T, (-90*M_PI/180), 0, 0, 0);   //gazebo末端坐标系旋转
    T_all *= T;

    for(int i = 0; i < 4; i++){ //将本应是0的元素赋值为0
        for(int j = 0; j < 4; j++){
            if(abs(T_all(i,j)) < 1e-5){
                T_all(i,j) = 0;
            }
        }
    }
    endPose << T_all(0, 3), T_all(1, 3), T_all(2, 3);

    //计算末端XYZ固定角
    Matrix3d RMatrix = T_all.block(0,0,3,3);
    getFixedAngleXYZ(endAngle, RMatrix);

    cout<<"Rmatrix:"<<endl<<RMatrix<<endl;
    Vector3d z0;
    z0<<0,0,1;
    cout<<"z6:"<<endl<<z0.transpose()*RMatrix<<endl;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "slove");
    ros::NodeHandle nh;
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);

    Vector3d endPose;
    Vector3d endAngle;

    //预设关节角度
    VectorXd robotAngle(6);
    //robotAngle << -90, 60, 30, 0, 0, 0;
    //robotAngle = robotAngle * M_PI/180;
    robotAngle << 0,60,30,0,0,0;
// resRobotAngle: 0.943592, -0.924595, -0.347169, 0.965984, 2.96725, -0.244775, 

// resRobotAngle: -2.198, -3.41613, -0.347169, -1.17125, 2.6454, -2.29525, 

// resRobotAngle: 0.943592, -2.86705, -2.79442, 1.97034, 2.6454, -2.29525, 

// resRobotAngle: -2.198, 0.924595, -2.79442, -2.17561, 2.96725, -0.244775, 
    robotAngle(1) += 90.0*M_PI/180.0;
    robotAngle(4) += -90.0*M_PI/180.0;


    //正运动学计算
    Motion(endPose ,endAngle, robotAngle);
    
    cout<<endl;
    cout<<"RobotAngle: "<<endl<<robotAngle.transpose()<<endl;
    cout<<"EndPose: "<<endl<<endPose.transpose()<<endl;
    cout<<"EndAngle: "<<endl<<endAngle.transpose()<<endl;


    //初始化publish的变量并初始化6个值
    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back((-90.0*M_PI/180.0));
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back((90.0*M_PI/180.0));
    init_pos.data.push_back(0);
    sleep(1);


    for(int i = 0; i < 6; i++){
        init_pos.data.at(i) += robotAngle(i);
    }


    pos_pub.publish(init_pos);


    ros::spin();
    return 0;
}