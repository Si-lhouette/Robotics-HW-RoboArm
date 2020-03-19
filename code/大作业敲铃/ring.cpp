/* 已知路径点，生成五阶多项式拟合最优轨迹 */
#include <ros/ros.h>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "Invert.h"

// 第一个点:[0.2289, 0, 0.454, 1.57, 0, 0](初始门位置)
// 第二个点:[0.26, 0.15, 0.08, 1.57, 0, 0]
// 第三个点:[0.4, 0, 0.2, 1.57, 0, 0]
// 第四个点:[0.28, ‐0.24, 0.08, 1.57, 0, 0]
// 第五个点:[0.4, 0, 0.2, 1.57, 0, 0]
// 第六个点:[0.26, 0.15, 0.08, 1.57, 0, 0]


VectorXd nowRobotAngle(6);
Vector3d Link6Pose;

/* 根据路径点算对应关节角度解，保证相邻关节角度解最近邻从而排除多解 */
void initialWayPoints(vector<VectorXd>& wp_list){

    vector<Eigen::VectorXd> robotAngleSet;
    
    //末端位置路径点
    vector<VectorXd> pose_list;
    VectorXd onePose(6);
    onePose << 0.2289, 0, 0.454, 1.57, 0, 0;
    pose_list.push_back(onePose);
    onePose << 0.26, 0.15, 0.08, 1.57, 0, 0;
    pose_list.push_back(onePose);
    onePose << 0.4, 0, 0.26, 1.57, 0, 0;
    pose_list.push_back(onePose);
    onePose << 0.28, -0.24, 0.08, 1.57, 0, 0;
    pose_list.push_back(onePose);  
    onePose << 0.4, 0, 0.26, 1.57, 0, 0;
    pose_list.push_back(onePose);
    onePose << 0.26, 0.15, 0.08, 1.57, 0, 0;
    pose_list.push_back(onePose);    


    VectorXd startRobotAngle(6);
    startRobotAngle << 0,0,0,0,0,0;
    wp_list.push_back(startRobotAngle);


    Vector3d aimendPose;
    Vector3d aimendAngle;
    for(int i = 1; i < pose_list.size(); i++){
        //得到对应末端pose的关节角
        robotAngleSet.clear();
        aimendPose = pose_list[i].block(0,0,3,1);
        aimendAngle = pose_list[i].block(3,0,3,1);
        InvertMotion(aimendPose ,aimendAngle, robotAngleSet);

        vector<VectorXd> robotAngleSetInrange;
        robotAngleSetInrange.clear();
        for(int j = 0; j < robotAngleSet.size(); j++){
            VectorXd robotAngle(6);
            robotAngle = robotAngleSet[j];

            bool inrange;
            inrange = AdjAndInrange(robotAngle);
            if(inrange){
                robotAngleSetInrange.push_back(robotAngle);
            }
        }


        //提取上一个关节角
        VectorXd robotAngleLast(6);
        robotAngleLast = wp_list[i-1];

        //在得到的关节角set中找与上一关节角最近的关节角
        VectorXd minrobotAngleNow(6);
        VectorXd robotAngleNow(6);
        minrobotAngleNow = robotAngleSetInrange[0];

        double mindis = (minrobotAngleNow-robotAngleLast).cwiseAbs().sum();
        double dis;
        for(int j = 0; j < robotAngleSetInrange.size(); j++){
            robotAngleNow = robotAngleSetInrange[j];
            dis = (robotAngleNow-robotAngleLast).cwiseAbs().sum();
            if(dis < mindis){
                minrobotAngleNow = robotAngleSetInrange[j];
                mindis = dis;
            }
        }

        wp_list.push_back(minrobotAngleNow);

    }
    //生成关节角序列
    for(int i = 0; i < wp_list.size(); i++){
        cout<<"wp_list_"<<i<<": "<<wp_list[i].transpose()<<endl;
    }
}


/* 存储MATLAB中优化得到的五阶多项式参数 */
//P.S. C++实在没找到能用的QP二次规划求解器
void getPolyCoeff(Matrix<double, 54, 6>& polyCoeff){
    polyCoeff << 00000, 00000, 00000, 00000, 00000, 00000, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        2.37496e-02, -6.55451e-02, -1.35903e-02, 00000, 5.88455e-02, 2.37496e-02, 
        -5.48740e-03, 1.65556e-02, 4.67327e-03, 00000, -1.46047e-02, -5.48740e-03, 
        3.14945e-04, -1.10000e-03, -3.62171e-04, 00000, 9.48948e-04, 3.14945e-04, 
        5.23278e-01, -1.28342e+00, 9.02228e-02, 00000, 1.19319e+00, 5.23278e-01, 
        2.17227e-02, -7.56075e-02, 1.85578e-01, 00000, 7.65056e-02, 2.17227e-02, 
        -7.31850e-02, 1.25156e-01, 4.44221e-02, 00000, -1.21844e-01, -7.31850e-02, 
        -7.26220e-03, -9.43398e-03, -1.06677e-02, 00000, 3.98759e-03, -7.26220e-03, 
        2.38622e-03, -1.09444e-02, -4.38101e-03, 00000, 9.11895e-03, 2.38622e-03, 
        -6.35024e-06, 1.70336e-03, 8.32716e-04, 00000, -1.27391e-03, -6.35024e-06, 
        5.82077e-14, -1.10021e+00, 5.97412e-01, 00000, 8.81978e-01, 5.82077e-14, 
        -3.50870e-01, -3.22385e-02, 5.60378e-02, 00000, -1.04040e-01, -3.50870e-01, 
        -2.39013e-02, -9.50328e-02, -6.86336e-02, 00000, 6.07665e-02, -2.39013e-02, 
        1.88737e-02, 1.01706e-03, 5.25970e-03, 00000, 6.59278e-03, 1.88737e-02, 
        2.29763e-03, 1.28174e-02, 7.23538e-03, 00000, -8.65207e-03, 2.29763e-03, 
        -5.48697e-04, -1.87014e-03, -1.22326e-03, 00000, 1.17317e-03, -5.48697e-04, 
        -7.08626e-01, -1.44733e+00, 5.65348e-01, 00000, 8.81978e-01, -7.08626e-01, 
        -1.01325e-02, 8.10238e-03, 5.38305e-02, 00000, -7.19242e-03, -1.01325e-02, 
        1.22218e-01, 1.05960e-01, 4.76531e-02, 00000, -3.33594e-02, 1.22218e-01, 
        1.80415e-03, -1.51461e-03, -9.21301e-03, 00000, 1.35610e-03, 1.80415e-03, 
        -5.35669e-03, -1.32711e-02, -9.82906e-03, 00000, 7.71360e-03, -5.35669e-03, 
        4.19600e-04, 1.99186e-03, 1.81358e-03, 00000, -1.28419e-03, 4.19600e-04, 
        -1.74825e-12, -1.10021e+00, 5.97412e-01, 00000, 8.81978e-01, -1.74825e-12, 
        3.75756e-01, 1.45789e-02, -1.99822e-01, 00000, 1.19355e-01, 3.75756e-01, 
        -2.17367e-02, -9.39538e-02, -9.46555e-02, 00000, 5.93556e-02, -2.17367e-02, 
        -2.53145e-02, 5.42812e-03, 2.22655e-02, 00000, -1.25229e-02, -2.53145e-02, 
        4.96729e-04, 1.45153e-02, 1.54704e-02, 00000, -1.02009e-02, 4.96729e-04, 
        9.68886e-04, -2.89816e-03, -3.74923e-03, 00000, 2.40306e-03, 9.68886e-04,
        5.23278e-01, -1.28342e+00, 9.02228e-02, 00000, 1.19319e+00, 5.23278e-01, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        -5.66497e-02, 5.89876e-02, 1.02197e-01, 00000, -6.14139e-02, -5.66497e-02, 
        1.45815e-02, -2.63748e-02, -3.98221e-02, 00000, 2.37319e-02, 1.45815e-02, 
        -1.04408e-03, 2.95911e-03, 4.14443e-03, 00000, -2.45730e-03, -1.04408e-03, 
        -2.90900e-14, -1.10021e+00, 5.97412e-01, 00000, 8.81978e-01, -2.90900e-14, 
        -3.72516e-01, -1.71999e-02, 1.82763e-01, 00000, -1.17023e-01, -3.72516e-01, 
        -1.98856e-02, -9.54483e-02, -1.04416e-01, 00000, 6.06846e-02, -1.98856e-02, 
        2.48074e-02, -5.01491e-03, -1.96113e-02, 00000, 1.21548e-02, 2.48074e-02, 
        1.65745e-05, 1.49048e-02, 1.79927e-02, 00000, -1.05475e-02, 1.65745e-05, 
        -3.21069e-04, -2.07246e-03, -2.32766e-03, 00000, 1.35604e-03, -3.21069e-04, 
        -7.08626e-01, -1.44733e+00, 5.65348e-01, 00000, 8.81978e-01, -7.08626e-01, 
        3.49313e-12, -9.30435e-13, -3.72862e-12, 00000, 9.31323e-13, 3.49313e-12, 
        1.18797e-01, 1.08609e-01, 6.62672e-02, 00000, -3.56969e-02, 1.18797e-01, 
        -1.30288e-13, 9.52795e-13, 1.22258e-12, 00000, -7.19673e-13, -1.30288e-13, 
        -4.46234e-03, -1.40060e-02, -1.44781e-02, 00000, 8.36921e-03, -4.46234e-03, 
        3.21069e-04, 2.07246e-03, 2.32766e-03, 00000, -1.35604e-03, 3.21069e-04, 
        -2.34993e-12, -1.10021e+00, 5.97412e-01, 00000, 8.81978e-01, -2.34993e-12, 
        3.72516e-01, 1.71999e-02, -1.82763e-01, 00000, 1.17023e-01, 3.72516e-01, 
        -1.98856e-02, -9.54483e-02, -1.04416e-01, 00000, 6.06846e-02, -1.98856e-02, 
        -2.48074e-02, 5.01491e-03, 1.96113e-02, 00000, -1.21548e-02, -2.48074e-02, 
        1.65745e-05, 1.49048e-02, 1.79927e-02, 00000, -1.05475e-02, 1.65745e-05, 
        1.04408e-03, -2.95911e-03, -4.14443e-03, 00000, 2.45730e-03, 1.04408e-03;


    cout<<"getpoly OK"<<endl;
}

/* 根据现时刻代入五阶多项式，获得关节速度 */
void getrobotv(VectorXd& Robotv, const double t, Matrix<double, 54, 6>& polyCoeff, VectorXd& ts){
    //Get Segment number
    int TheSeg = 0;
    int n_para_per_seg = 6;
    double t_inseg;
    double ts_acc = 0;
    for(int i = 0; i < ts.size(); i++){
        if(t < ts_acc + ts(i)){
            break;
        }
        else{
            ts_acc += ts(i);
            TheSeg++;
        }
    }
    t_inseg = t - ts_acc;

    //Get Robotv
    for(int i = 0; i < 6; i++){
        VectorXd ployc(6);
        ployc = polyCoeff.block(TheSeg*n_para_per_seg, i, n_para_per_seg, 1);
        double v = 0;
        for(int j = 0; j < 5; j++){
            v += (j+1) * ployc(j+1) * pow(t_inseg, j);
        }
        Robotv(i) = v;
    }
}

/* 回调函数：接收实时关节角 */ 
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for(int i = 0; i < 6;i++){
        nowRobotAngle(i)=msg->position[i];
    }

    cout<<"nowRobotAngle: "<<nowRobotAngle.transpose()<<endl;
}

/* 回调函数：接收实时末端关节位置/线速度/角速度 */ 
void LinkStatesCallBack(const gazebo_msgs::LinkStatesConstPtr& msg){
    geometry_msgs::Pose Link6msg = msg->pose[10];

    Link6Pose << Link6msg.position.x, Link6msg.position.y, Link6msg.position.z;

    geometry_msgs::Twist Tw6 = msg->twist[10];

    // //全局线速度
    // cout<<"linear speed: "<<Tw6.linear.x<<", "<<Tw6.linear.y<<", "<<Tw6.linear.z<<endl;
    // //全局角速度
    // cout<<"angle speed: "<<Tw6.angular.x<<", "<<Tw6.angular.y<<", "<<Tw6.angular.z<<endl;
    

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ring");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/probot_anno/joint_states", 1, jointstatesCallback);
    ros::Subscriber sublink6 = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, LinkStatesCallBack);
    
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);
    ros::Rate loopHZ(100);

    /* 初始化 */
    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);

    InitialDH();

    vector<VectorXd> wp_list;
    initialWayPoints(wp_list);

    //获得MATLAB优化得到的五阶多项式参数
    Matrix<double, 54, 6> polyCoeff;
    getPolyCoeff(polyCoeff);
    double fseg_t = 5;  //第一段轨迹时长
    double seg_t = 2.79;    //之后每段轨迹时长

    VectorXd Robotv(6);
    VectorXd RobotvLast(6);
    RobotvLast << 0,0,0,0,0,0;
    VectorXd ts(9);

    ts(0) = fseg_t;
    for(int i = 1; i < 9; i++){
        ts(i) = seg_t;
    }


    VectorXd All_roboAngle(6);
    All_roboAngle << 0,0,0,0,0,0;


    ros::Time begin=ros::Time::now();
    double t;
    double tLast = 0;
    int cnt = 0;
    bool startPoint = false;
    bool ring = false;

    /* 开始计时速度控制 */
    while (ros::ok()){
        ros::Time nowtime=ros::Time::now();
        t = nowtime.toSec() - begin.toSec();
        t -= cnt*seg_t*4;   //时间循环

        //判断是否在蓝球处[startPoint]or红球处[ring]
        if(abs(t-(fseg_t+seg_t*6))<0.01){
            ring = true;
        }
        if(t > fseg_t+seg_t*8){
            startPoint = true;
            cnt++;
            t -= cnt*seg_t*4;
            if(cnt >= 9-1){ //敲铃达到9次，退出循环
                break;
            }
        }
        
        //根据现时刻代入五阶多项式，获得关节速度
        getrobotv(Robotv, t, polyCoeff, ts);
        
        for(int i = 0; i < 6; i++){
            init_pos.data.at(i) = Robotv(i);
        }

        All_roboAngle += Robotv * (t-tLast);

        RobotvLast = Robotv;
        tLast = t;

        vel_pub.publish(init_pos);

        //到达红球或蓝球时，显示误差
        if(startPoint){
            startPoint = false;
            ros::spinOnce();

            cout<<endl<<"-------------------"<<"t: "<<nowtime.toSec() - begin.toSec()<<endl;
            cout<<"No."<<cnt+1<<" Back To Start!!!!!"<<endl;

            cout<<"Link6Pose: "<< Link6Pose.transpose()<<endl;

            double resuial = (nowRobotAngle - wp_list[1]).block(0,0,3,1).cwiseAbs().sum();
            cout<<"resuial to startPoint: "<<resuial<<endl;
        }
        if(ring){
            ring = false;
            ros::spinOnce();

            cout<<endl<<"-------------------"<<"t: "<<nowtime.toSec() - begin.toSec()<<endl;
            cout<<"No."<<cnt+2<<" RingRingRing!!!!!"<<endl;

            cout<<"Link6Pose: "<< Link6Pose.transpose()<<endl;

            double resuial = (nowRobotAngle - wp_list[3]).block(0,0,3,1).cwiseAbs().sum();
            cout<<"resuial to ringPoint: "<<resuial<<endl;
        }

        loopHZ.sleep();
    }

    //到达蓝球时，显示误差
    if(startPoint){
        startPoint = false;
        ros::spinOnce();
        cout<<"Link6Pose: "<< Link6Pose.transpose()<<endl;

        double resuial = (nowRobotAngle - wp_list[1]).block(0,0,3,1).cwiseAbs().sum();
        cout<<"resuial to startPoint: "<<resuial<<endl;
    }

    //理论计算最终时刻机械臂关节角度
    cout<<"All_roboAngle"<<All_roboAngle.transpose()<<endl;

    //pub 0速度
    while(1){
        for(int i = 0; i < 6; i++){
            init_pos.data.at(i) = 0.0;
        }
        //cout<<"pub: "<<Robotv.transpose()<<endl;

        vel_pub.publish(init_pos);
    }


    cout<< "Done" <<endl;
    ros::spin();
    return 0;
}