//逆运动学解算
//Input:DH参数，Pose
//Output:各关节角度
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"


using namespace std;
using namespace Eigen;


VectorXd alphaDH(6);
VectorXd aDH(6);
VectorXd dDH(6);

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

/* 通过旋转矩阵计算欧拉角ZYZ */
//Input: Matrix3d& RMatrix      旋转矩阵
//Output:Vector3d& AngleZYZ     欧拉角ZYZ[alpha, beta, gamma] /rad
void getEulerAngleZYZ(vector<Vector3d>& AngleZYZ, Matrix3d& RMatrix){
    double alpha, beta, gamma;
    //beta in [-180,0]
    double beta_y = -sqrt(RMatrix(2,0)*RMatrix(2,0)+RMatrix(2,1)*RMatrix(2,1));
    double beta_x = RMatrix(2,2);
    //cout<<endl<<"RM46:"<<endl<<RMatrix<<endl;
    //cout<<"beta_y:"<<beta_y<<endl<<endl;
    if(abs(beta_y) > 1e-3){
        beta = atan2(beta_y, beta_x);
        alpha = atan2(RMatrix(1,2)/sin(beta), RMatrix(0,2)/sin(beta));
        gamma = atan2(RMatrix(2,1)/sin(beta), -RMatrix(2,0)/sin(beta));
    }
    else if(beta_x >= 0){
        beta = 0;
        alpha = 0;
        gamma = atan2(-RMatrix(0,1), RMatrix(1,1));
    }else{
        beta = M_PI;
        alpha = 0;
        gamma = atan2(RMatrix(0,1), -RMatrix(0,0));        
    }

    Vector3d ZYZ(alpha, beta, gamma);
    AngleZYZ.push_back(ZYZ);

    //beta in [0,180]
    beta_y = sqrt(RMatrix(2,0)*RMatrix(2,0)+RMatrix(2,1)*RMatrix(2,1));
    beta_x = RMatrix(2,2);
    //cout<<endl<<"RM46:"<<endl<<RMatrix<<endl;
    //cout<<"beta_y:"<<beta_y<<endl<<endl;
    if(abs(beta_y) > 1e-3){
        beta = atan2(beta_y, beta_x);
        alpha = atan2(RMatrix(1,2)/sin(beta), RMatrix(0,2)/sin(beta));
        gamma = atan2(RMatrix(2,1)/sin(beta), -RMatrix(2,0)/sin(beta));
    }
    else if(beta_x >= 0){
        beta = 0;
        alpha = 0;
        gamma = atan2(-RMatrix(0,1), RMatrix(1,1));
    }else{
        beta = M_PI;
        alpha = 0;
        gamma = atan2(RMatrix(0,1), -RMatrix(0,0));        
    }
    ZYZ << alpha, beta, gamma;
    AngleZYZ.push_back(ZYZ);

}

/* 求解一元二次方程 */
int solve2O(vector<double>& ret, vector<double>& coff){
    double a, b, c;
    a = coff[0];
    b = coff[1];
    c = coff[2];
    double del = b*b - 4*a*c;
    if(del < 0){
        cout<<"Error! delta <= 0"<<endl;
        return -1;
    }
    else if(del == 0){
        cout<<"delta = 0"<<endl;
        ret[0] = (-b+sqrt(del))/(2*a);
        ret[1] = (-b-sqrt(del))/(2*a);
        return 0;
    }
    else{
        ret[0] = (-b+sqrt(del))/(2*a);
        ret[1] = (-b-sqrt(del))/(2*a);
        return 1;
    }
}

/* 将齐次变换矩阵中本应是0的元素赋值为0 */
void setzero(Matrix4d& T){
    for(int i = 0; i < 4; i++){ //将本应是0的元素赋值为0
        for(int j = 0; j < 4; j++){
            if(abs(T(i,j)) < 1e-3){
                T(i,j) = 0;
            }
        }
    }
}

/* 计算Link4正运动学 */
void MotionLink4(Vector3d& endPose, Matrix3d& RM4 , VectorXd& robotAngle){

    //计算末端齐次变换矩阵
    Matrix4d T_all = Matrix4d::Identity();
    Matrix4d T;
    for(int i = 0; i < 4; i++){
        getT(T, alphaDH(i), aDH(i), dDH(i), robotAngle(i));
        T_all *= T;
    }
    //cout<<"lin4 pose:"<<"("<<T_all(0,3)<<", "<<T_all(1,3)<<", "<<T_all(2,3)<<")"<<endl;

    setzero(T_all);
    endPose << T_all(0, 3), T_all(1, 3), T_all(2, 3);

    RM4 = T_all.block(0,0,3,3);
    //cout<<endl<<"RM4:"<<endl<<RM4<<endl;
}



/* 计算正运动学 */
void Motion(Vector3d& endPose, Vector3d& endAngle , VectorXd& robotAngle){
    VectorXd robotAngle1 = robotAngle;
    robotAngle1(1) += 90.0*M_PI/180.0;
    robotAngle1(4) += -90.0*M_PI/180.0;

    //计算末端齐次变换矩阵
    Matrix4d T_all = Matrix4d::Identity();
    Matrix4d T;
    for(int i = 0; i < 6; i++){
        getT(T, alphaDH(i), aDH(i), dDH(i), robotAngle1(i));
        T_all *= T;
    }
    //cout<<"lin4 pose:"<<"("<<T_all(0,3)<<", "<<T_all(1,3)<<", "<<T_all(2,3)<<")"<<endl;
    //cout<<endl<<"expect T_06:"<<endl<<T_all<<endl;

    getT(T, 0, 0, 0.055, 0);   //第七坐标轴平移
    T_all *= T;
    getT(T, (-90*M_PI/180), 0, 0, 0);   //gazebo末端坐标系旋转
    T_all *= T;

    setzero(T_all);
    endPose << T_all(0, 3), T_all(1, 3), T_all(2, 3);

    //计算末端XYZ固定角
    Matrix3d RMatrix = T_all.block(0,0,3,3);
    getFixedAngleXYZ(endAngle, RMatrix);
}



/* 计算逆运动学 */
void InvertMotion(Vector3d& endPose, Vector3d& endAngle , vector<Eigen::VectorXd>& robotAngleSet){

    /* 计算末端齐次变换矩阵 */
    Matrix4d T_all;
    double roll = endAngle(0);
    double pitch = endAngle(1);
    double yaw = endAngle(2);
    T_all(0,0) = cos(yaw)*cos(pitch);
    T_all(0,1) = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
    T_all(0,2) = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);
    T_all(1,0) = sin(yaw)*cos(pitch);
    T_all(1,1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
    T_all(1,2) = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
    T_all(2,0) = -sin(pitch);
    T_all(2,1) = cos(pitch)*sin(roll);
    T_all(2,2) = cos(pitch)*cos(roll);

    T_all(0,3) = endPose(0);
    T_all(1,3) = endPose(1);
    T_all(2,3) = endPose(2);
    T_all(3,3) = 1;
    //cout<<endl<<"Tall: "<<endl<<T_all<<endl;


    /* 计算Link4齐次变换矩阵 */
    Matrix4d T;
    Matrix4d T06;
    getT(T, (-90*M_PI/180), 0, 0, 0);
    T_all = T_all*T.inverse();
    setzero(T_all);
    getT(T, 0, 0, 0.055, 0);
    T06 = T_all*T.inverse();
    setzero(T06);
    //cout<<endl<<"T06: "<<endl<<T06<<endl;
    double ex, ey, ez, r;
    ex = T06(0,3);
    ey = T06(1,3);
    ez = T06(2,3) - dDH(0);
    cout<<"Get link4 pose: ("<<ex<<", "<<ey<<", "<<ez<<")"<<endl;
    r = ex*ex + ey*ey + ez*ez;


    /* 计算theta3 */
    vector<double> u(2);
    vector<double> coff(3);
    int state;

    vector<double> theta3(2);
    double sin3 = (r - dDH(3)*dDH(3) - aDH(2)*aDH(2)) / (2*dDH(3)*aDH(2));
    theta3[0] = asin(sin3);
    if(theta3[0] >= 0){
        theta3[1] = M_PI - theta3[0];
    }else{
        theta3[1] = -M_PI - theta3[0] + 2*M_PI;
    }
    if(abs(sin(theta3[0]) - sin3) + abs(sin(theta3[1]) - sin3) < 1e-3){
        cout<<"check theta3 right"<<endl<<endl;
    }


    /* 计算theta2 */
    vector<double> theta2(4);
    for(int i = 0; i < 2; i++){
        if(abs(theta3[i]) < 1e-3){
            theta3[i] = 0;
        }
        coff[0] = dDH(3)*cos(theta3[i]) - ez;
        coff[1] = 2*(dDH(3)*sin(theta3[i]) + aDH(2));
        coff[2] = -(dDH(3)*cos(theta3[i]) + ez);
        state = solve2O(u, coff);
        theta2[2*i] = 2*atan(u[0]);
        theta2[2*i+1] = 2*atan(u[1]);

        cout<<"Get theta2: "<<theta2[2*i]<<", "<<theta2[2*i+1]<<endl;

        if(abs(ez - sin(theta2[2*i])* ( dDH(3)*sin(theta3[i]) + aDH(2) ) + dDH(3)*cos(theta2[2*i])*cos(theta3[i]) ) < 1e-3){
            cout<<"check theta2 "<<2*i<<endl;
        }
        
        if(abs(ez - sin(theta2[2*i+1])* ( dDH(3)*sin(theta3[i]) + aDH(2) ) + dDH(3)*cos(theta2[2*i+1])*cos(theta3[i]) ) < 1e-3){
            cout<<"check theta2 "<<2*i+1<<endl;
        }
    }


    /* 计算theta1 */
    vector<double> theta1(4);
    for(int i = 0; i < 4; i++){
        if(abs(theta2[i]) < 1e-3){
            theta2[i] = 0;
        }
        double g1 = (cos(theta2[i]) * (dDH(3)*sin(theta3[int(i/2)])+aDH(2)) + dDH(3)*sin(theta2[i])*cos(theta3[int(i/2)]));

        theta1[i] = acos(ex/g1);
        if(abs(ey - sin(theta1[i]) * g1) > 1e-3){
            theta1[i] = -theta1[i];
            //cout<<"adj: "<<abs(ey - sin(theta1[i]) * g1)<<", "<<cos(theta1[i]) - ex/g1<<endl;
        }
        if(abs(ey - sin(theta1[i]) * g1) > 1e-3){
            cout<<"i = "<<i+1<<" false"<<endl;
        }
        if(abs(theta1[i]) < 1e-3){
            theta1[i] = 0;
        }
    }


    /* 验证Link4 Pose，计算theta456 */
    cout<<endl<<"Final res:"<<endl;
    vector<double> theta4(8);
    vector<double> theta5(8);
    vector<double> theta6(8);
    vector<int> checkpose(4,0);
    for(int i = 0; i < 4; i++){

        cout<<"theta3 = "<<theta3[int(i/2)]<<", ";
        cout<<"theta2 = "<<theta2[i]<<", ";
        cout<<"theta1 = "<<theta1[i]<<";  ";

        Vector3d epose;
        Matrix3d RM4;
        VectorXd rangle(4);
        rangle << theta1[i], theta2[i], theta3[int(i/2)], 0;
        MotionLink4(epose, RM4, rangle);
        cout<<"epose: ("<<epose(0)<<", "<<epose(1)<<", "<<epose(2)<<")";

        if(abs(epose(0)-ex) + abs(epose(1)-ey) + abs(epose(2)-(ez+dDH(0))) < 1e-3){
            checkpose[i] = 1;
            //cout<<endl<<"T06:"<<endl<<T06<<endl;
            Matrix3d RM46 =  RM4.inverse() * T06.block(0,0,3,3);
            vector<Vector3d> ZYZangle;
            getEulerAngleZYZ(ZYZangle, RM46);
            theta4[2*i] = ZYZangle[0](0);
            theta5[2*i] = ZYZangle[0](1);
            theta6[2*i] = ZYZangle[0](2); 
            theta4[2*i+1] = ZYZangle[1](0);
            theta5[2*i+1] = ZYZangle[1](1);
            theta6[2*i+1] = ZYZangle[1](2); 
        }
        cout<<"  check: "<<checkpose[i]<<endl<<endl;
   

    }

    for(int i = 0; i < 4; i++){
        if(checkpose[i]){
            cout<<endl<<i<<"  |theta456: "<<theta4[2*i]<<", "<<theta5[2*i]<<", "<<theta6[2*i]<<endl;
            VectorXd robotAngle(6);
            robotAngle << theta1[i], theta2[i], theta3[int(i/2)], theta4[2*i], theta5[2*i], theta6[2*i];
            robotAngleSet.push_back(robotAngle);

            cout<<endl<<i<<"  |theta456: "<<theta4[2*i+1]<<", "<<theta5[2*i+1]<<", "<<theta6[2*i+1]<<endl;
            robotAngle << theta1[i], theta2[i], theta3[int(i/2)], theta4[2*i+1], theta5[2*i+1], theta6[2*i+1];
            robotAngleSet.push_back(robotAngle);
        }
    }
}

bool Inrange(VectorXd& robotAngle){
    bool in = true;

    robotAngle(1) = robotAngle(1) - 90.0*M_PI/180.0;//第2轴零点调整
    if(robotAngle(1)<-M_PI)
        robotAngle(1) += 2*M_PI;
    if(robotAngle(1)<-2.01 || robotAngle(1)>2.01)
        in = false;

    if(robotAngle(2)<-0.69 || robotAngle(2)>3.83)
        in = false;
    
    robotAngle(4) = robotAngle(4) + 90.0*M_PI/180.0;//第5轴零点调整
    if(robotAngle(4)<-0.78 || robotAngle(4)>3.92)
        in = false;

    return in;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);

    /* DH参数初始化 */
    alphaDH << 0, (90.0*M_PI/180.0), 0, (90.0*M_PI/180.0), (-90.0*M_PI/180.0), (90.0*M_PI/180.0);
    aDH << 0, 0, 0.225, 0, 0, 0;
    dDH << 0.284, 0, 0, 0.2289, 0, 0;




    /* 设定目标末端位姿 */
    Vector3d aimendPose(0.2, 0.2, 0.2007);
    Vector3d aimendAngle(1.57, -1.57, 0);
    VectorXd robotAngle(6);



    /* 逆运动学计算 */
    vector<Eigen::VectorXd> robotAngleSet;
    InvertMotion(aimendPose ,aimendAngle, robotAngleSet);

    
    cout<<endl;
    cout<<"AimEndPose: "<<endl<<aimendPose.transpose()<<endl;
    cout<<"AimEndAngle: "<<endl<<aimendAngle.transpose()<<endl;
    cout<<"解的个数: "<<robotAngleSet.size()<<endl;


    vector<VectorXd> robotAngleSetInrange;
    
    for(int i = 0; i < robotAngleSet.size(); i++){
        robotAngle = robotAngleSet[i];

        bool inrange;
        inrange = Inrange(robotAngle);


        /* 正运动学验证 */
        cout<<endl<<"----Check "<<i<<"----";
        Vector3d endPose;
        Vector3d endAngle;
        Motion(endPose ,endAngle, robotAngle);
        if( (endPose-aimendPose).cwiseAbs().sum() + (endAngle-aimendAngle).cwiseAbs().sum()< 1e-2){
            cout<<" v!"<<endl;
        }else{
            cout<<" x!"<<endl;
            cout<<"diffabs: "<<(endPose-aimendPose).cwiseAbs().sum() + (endAngle-aimendAngle).cwiseAbs().sum()<<endl;
            cout<<"resEndPose: "<<endl<<endPose.transpose()<<endl;
            cout<<"resEndAngle: "<<endl<<endAngle.transpose()<<endl;
        }
        

        cout<<"resRobotAngle: ";
        for(int j = 0; j < 6; j++){
            cout<<robotAngle[j]<<", ";
        }
        cout << "inrange: "<<inrange<<endl;
        cout<<endl;
        if(inrange){
            robotAngleSetInrange.push_back(robotAngle);
        }
    }

    



    /* Pub关节角 */
    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);

    int pubnum;
    while(1){
        cout<<"select pubnum:";
        cin>>pubnum;
        if(pubnum < robotAngleSetInrange.size()){
            robotAngle = robotAngleSetInrange[pubnum];
            for(int i = 0; i < 6; i++){
                init_pos.data.at(i) = robotAngle(i);
            }
            pos_pub.publish(init_pos);
            cout<<"pub no."<<pubnum<<endl;
        }
    }







    ros::spin();
    return 0;
}