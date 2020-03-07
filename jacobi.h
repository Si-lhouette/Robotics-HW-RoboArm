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

extern VectorXd alphaDH(6);
extern VectorXd aDH(6);
extern VectorXd dDH(6);



void InitialDH2(){
    /* DH参数初始化 */
    alphaDH << 0, (90.0*M_PI/180.0), 0, (90.0*M_PI/180.0), (-90.0*M_PI/180.0), (90.0*M_PI/180.0);
    aDH << 0, 0, 0.225, 0, 0, 0;
    dDH << 0.284, 0, 0, 0.2289, 0, 0.055;
}

void genJMatrix(Matrix<double, 6, 6>& J, VectorXd& robotAngle){
    Vector3d P1;
    Vector3d P2;
    Vector3d P3;
    Vector3d P4;
    Vector3d P5;
    Vector3d P6;
    P1 << 0,0,dDH(0);
    P2 = P1;
    P3(0) = aDH(2) * cos(robotAngle(0)) * cos(robotAngle(1));
    P3(1) = aDH(2) * sin(robotAngle(0)) * cos(robotAngle(1));
    P3(2) = dDH(0) + aDH(2)*sin(robotAngle(1));
    P4(0) = cos(robotAngle(0))*(dDH(3)*sin(robotAngle(1) + robotAngle(2)) + aDH(2)*cos(robotAngle(1)));
    P4(1) = sin(robotAngle(0))*(dDH(3)*sin(robotAngle(1) + robotAngle(2)) + aDH(2)*cos(robotAngle(1)));
    P4(2) = dDH(0) - 1.0*dDH(3)*cos(robotAngle(1) + robotAngle(2)) + aDH(2)*sin(robotAngle(1));
    P5 = P4;
    P6(0) = dDH(5)*(sin(robotAngle(4))*(sin(robotAngle(0))*sin(robotAngle(3)) + cos(robotAngle(1) + 
        robotAngle(2))*cos(robotAngle(0))*cos(robotAngle(3))) + sin(robotAngle(1) + 
        robotAngle(2))*cos(robotAngle(0))*cos(robotAngle(4))) + cos(robotAngle(0))*(dDH(3)*sin(robotAngle(1) + robotAngle(2)) 
        + aDH(2)*cos(robotAngle(1)));

    P6(1) = sin(robotAngle(0))*(dDH(3)*sin(robotAngle(1) + robotAngle(2)) + aDH(2)*cos(robotAngle(1))) - 
        1.0*dDH(5)*(sin(robotAngle(4))*(cos(robotAngle(0))*sin(robotAngle(3)) - 1.0*cos(robotAngle(1) + 
        robotAngle(2))*cos(robotAngle(3))*sin(robotAngle(0))) - 1.0*sin(robotAngle(1) + 
        robotAngle(2))*cos(robotAngle(4))*sin(robotAngle(0)));

    P6(2) = dDH(0) - 1.0*dDH(5)*(cos(robotAngle(1) + robotAngle(2))*cos(robotAngle(4)) - 1.0*sin(robotAngle(1) + 
        robotAngle(2))*cos(robotAngle(3))*sin(robotAngle(4))) - 1.0*dDH(3)*cos(robotAngle(1) + robotAngle(2)) + aDH(2)*sin(robotAngle(1));
    
    Vector3d Z0(0,0,0);
    Vector3d Z1;
    Vector3d Z2;
    Vector3d Z3;
    Vector3d Z4;
    Vector3d Z5;
    Vector3d Z6;
    Z1 << 0,0,1;
    Z2 << sin(robotAngle(0)), -cos(robotAngle(0)), 0;
    Z3 = Z2;
    Z4(0) = sin(robotAngle(1) + robotAngle(2))*cos(robotAngle(0));
    Z4(0) = sin(robotAngle(1) + robotAngle(2))*sin(robotAngle(0));
    Z4(0) = -cos(robotAngle(1) + robotAngle(2));
    Z5(0) = cos(robotAngle(3))*sin(robotAngle(0)) - cos(robotAngle(1) + robotAngle(2))*cos(robotAngle(0))*sin(robotAngle(3));
    Z5(1) = -cos(robotAngle(0))*cos(robotAngle(3)) - cos(robotAngle(1) + robotAngle(2))*sin(robotAngle(0))*sin(robotAngle(3));
    Z5(2) = -sin(robotAngle(1) + robotAngle(2))*sin(robotAngle(3));
    Z6(0) = sin(robotAngle(4))*(sin(robotAngle(0))*sin(robotAngle(3)) + cos(robotAngle(1) + robotAngle(2))*cos(robotAngle(0))*cos(robotAngle(3))) + 
        sin(robotAngle(1) + robotAngle(2))*cos(robotAngle(0))*cos(robotAngle(4));

    Z6(1) = sin(robotAngle(1) + robotAngle(2))*cos(robotAngle(4))*sin(robotAngle(0)) - sin(robotAngle(4))*(cos(robotAngle(0))*sin(robotAngle(3)) - 
        cos(robotAngle(1) + robotAngle(2))*cos(robotAngle(3))*sin(robotAngle(0)));

    Z6(2) = sin(robotAngle(1) + robotAngle(2))*cos(robotAngle(3))*sin(robotAngle(4)) - cos(robotAngle(1) + robotAngle(2))*cos(robotAngle(4));


    J.block(0,0,3,1) = Z1.cross(P6-P1);
    J.block(3,0,3,1) = Z1;

    J.block(0,1,3,1) = Z2.cross(P6-P2);
    J.block(3,1,3,1) = Z2;  

    J.block(0,2,3,1) = Z3.cross(P6-P3);
    J.block(3,2,3,1) = Z3;    

    J.block(0,3,3,1) = Z4.cross(P6-P4);
    J.block(3,3,3,1) = Z4;  

    J.block(0,4,3,1) = Z5.cross(P6-P5);
    J.block(3,4,3,1) = Z5;   

    J.block(0,5,3,1) = Z0;
    J.block(3,5,3,1) = Z6;  
}

void robov2endv(VectorXd& endv, VectorXd& robov, VectorXd& robotAngle){
    Matrix<double, 6, 6> J;
    genJMatrix(J, robotAngle);
    endv = J*robov;
}

void endv2robov(VectorXd& robov, VectorXd& endv, VectorXd& robotAngle){
    Matrix<double, 6, 6> J;
    genJMatrix(J, robotAngle);
    robov = J.inverse() * endv;
}


void velcontrol(VectorXd& endv, VectorXd& nowRobotAngle, std_msgs::Float64MultiArray& init_pos){
    
    VectorXd robov(6);
    VectorXd mybotAngle(6);
    mybotAngle = nowRobotAngle;
    mybotAngle(1) += 90.0*M_PI/180.0;
    mybotAngle(4) += -90.0*M_PI/180.0;
    endv2robov(robov, endv, mybotAngle);

    for(int i = 0; i < 6; i++){
        init_pos.data.at(i) = robov(i);
    }
    cout<<"pub: "<<robov.transpose()<<endl;
}