#include <ros/ros.h>
#include <cmath>
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    /* 1.定义 */

    /* 矩阵 */
    //动态矩阵
    MatrixXd m1(2,2);
    m1(0,0) = 3;
    m1(0,1) = 2;
    m1(1,0) = 1;
    m1(1,1) = -1;
    cout<<"m1:"<<endl;
    cout<<m1<<endl;

    //随机矩阵，各元素取值范围是[-1，1]
    MatrixXd m2 = MatrixXd::Random(3,3);
    cout<<"m2:"<<endl;
    cout<<m2<<endl;


    //常数矩阵
    MatrixXd m3 = MatrixXd::Constant(3,3,1.2);
    cout<<"m3:"<<endl;
    cout<<m3<<endl;

    /* 向量 */
    //维度为3的列向量v
    VectorXd v1(3);
    v1 << 1, 2, 3;//赋值于v
    cout<<"v1:"<<endl;
    cout<<v1<<endl;


    /* 2.运算 */
    //乘法
    cout << "m3 * v1 =" << endl << m3 * v1 << endl;
    cout << "m3 * 2 =" << endl << m3 * 2 << endl;

    Vector3d v(1,2,3);
    Vector3d w(0,1,2);
    //向量点乘
    cout << "Dot product: " << v.dot(w) << endl;
    //向量叉乘
    cout << "Cross product:\n" << v.cross(w) << endl;

    Matrix2d a;
    a << 1, 2,
        3, 4;
    cout << "Here is the matrix a\n" << a << endl;
    //转置
    cout << "Here is the matrix a^T\n" << a.transpose() << endl;
    //共轭
    cout << "Here is the matrix a^H\n" << a.conjugate() << endl;
    //逆   
    cout << "Here is the matrix a^{-1}\n" << a.inverse() << endl;
    

    /*其他
    m.determinant() // 行列式
    m.sum()         //所有元素之和
    m.prod()        //所有元素之积
    m.mean()        //元素的平均数
    m.minCoeff()    //最小元素
    m.maxCoeff()    //最大元素
    m.trace()       //迹
    */


    ros::spin();
    return 0;
}