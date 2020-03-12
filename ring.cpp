/* 已知路径点，生成五阶多项式拟合最优轨迹 */
#include <ros/ros.h>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>


#include "trajectory_gen.h"
#include "Invert.h"
#include "tic_toc.h"

// 第一个点:[0.2289, 0, 0.454, 1.57, 0, 0](初始门位置)
// 第二个点:[0.26, 0.15, 0.08, 1.57, 0, 0]
// 第三个点:[0.4, 0, 0.2, 1.57, 0, 0]
// 第四个点:[0.28, ‐0.24, 0.08, 1.57, 0, 0]
//[0.4, 0, 0.2, 1.57, 0, 0]
//[0.26, 0.15, 0.08, 1.57, 0, 0]


VectorXd nowRobotAngle(6);


Vector3d Link6Pose;





void initialWayPoints(vector<VectorXd>& wp_list){

    vector<Eigen::VectorXd> robotAngleSet;
    

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

    for(int i = 0; i < wp_list.size(); i++){
        cout<<"wp_list_"<<i<<": "<<wp_list[i].transpose()<<endl;
    }
}


void genTrajectory(vector<VectorXd>& wp_list){

    MatrixXd waypoints(wp_list.size(), 6);
    int d_order = 3;
    MatrixXd polyCoeff;
    
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k) = wp_list[k];

    MatrixXd vel = MatrixXd::Zero(2, 6); 
    MatrixXd acc = MatrixXd::Zero(2, 6);

    // generete start and end derivative constraints 
    MatrixXd derivative_cons = MatrixXd::Zero(2*d_order, 6);
    derivative_cons << waypoints.row(0),
                       vel.row(0),
                       acc.row(0),
                       waypoints.row(waypoints.rows()-1),
                       vel.row(1),
                       acc.row(1);

    
    VectorXd ts(waypoints.rows() - 1);
    for (int k = 0; k < waypoints.rows()-1; k++) {
        ts(k) = 5;
    }

    //cout<<"waypoint:"<<endl<<waypoints.col(0)<<endl;
    cout<<"derivative_cons:"<<endl<<derivative_cons<<endl;
    MinimumSnapSolver minimum_snap_solver(d_order, waypoints.col(0), derivative_cons.col(0), ts);

    polyCoeff = minimum_snap_solver.QPSolver();

    //cout<<"polyCoeff:"<<endl<<polyCoeff<<endl;
}


void getPolyCoeff(Matrix<double, 54, 6>& polyCoeff){
    polyCoeff << 00000, 00000, 00000, 00000, 00000, 00000, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        4.19767e-02, -1.07734e-01, -1.39990e-02, 00000, 1.27795e-01, 4.19767e-02, 
        -1.18347e-02, 3.19047e-02, 6.19107e-03, 00000, -4.05058e-02, -1.18347e-02, 
        8.46151e-04, -2.49616e-03, -5.84721e-04, 00000, 3.30451e-03, 8.46151e-04, 
        5.23278e-01, -1.28342e+00, 9.02228e-02, 00000, 1.19319e+00, 5.23278e-01, 
        6.82661e-02, -1.98699e-01, 1.64519e-01, 00000, -5.56941e-03, 6.82661e-02, 
        -9.08760e-02, 1.72504e-01, 5.21334e-02, 00000, -2.40135e-01, -9.08760e-02, 
        -1.19946e-02, 3.35581e-03, -8.49717e-03, 00000, 8.42323e-03, -1.19946e-02, 
        5.08830e-03, -1.80185e-02, -5.50334e-03, 00000, 2.55843e-02, 5.08830e-03, 
        -2.98682e-04, 2.44611e-03, 9.44818e-04, 00000, -3.76285e-03, -2.98682e-04, 
        -8.71728e-14, -1.10021e+00, 5.97412e-01, 00000, 4.04800e-01, -8.71728e-14, 
        -3.67753e-01, 1.58428e-02, 6.37441e-02, 00000, -6.21315e-02, -3.67753e-01, 
        -1.78439e-02, -1.09927e-01, -7.07135e-02, 00000, 2.08086e-01, -1.78439e-02, 
        2.15777e-02, -6.67639e-03, 3.93914e-03, 00000, -3.99394e-05, 2.15777e-02, 
        9.06751e-04, 1.62270e-02, 7.72411e-03, 00000, -2.70956e-02, 9.06751e-04, 
        -3.97596e-04, -2.21085e-03, -1.26315e-03, 00000, 3.98643e-03, -3.97596e-04, 
        -7.08626e-01, -1.44733e+00, 5.65348e-01, 00000, 8.81978e-01, -7.08626e-01, 
        -2.74477e-03, -1.13690e-02, 5.04346e-02, 00000, -5.18615e-02, -2.74477e-03, 
        1.18782e-01, 1.11987e-01, 4.84309e-02, 00000, -1.91727e-01, 1.18782e-01, 
        5.61744e-04, 1.73629e-03, -8.58172e-03, 00000, 9.02518e-03, 5.61744e-04, 
        -4.65959e-03, -1.47248e-02, -9.95998e-03, 00000, 2.87144e-02, -4.65959e-03, 
        3.43605e-04, 2.13785e-03, 1.81130e-03, 00000, -4.60127e-03, 3.43605e-04, 
        -1.97864e-12, -1.10021e+00, 5.97412e-01, 00000, 4.04800e-01, -1.97864e-12, 
        3.72095e-01, 2.06592e-02, -1.98098e-01, 00000, 1.93994e-01, 3.72095e-01, 
        -2.02588e-02, -9.67827e-02, -9.45571e-02, 00000, 2.24738e-01, -2.02588e-02, 
        -2.46871e-02, 4.42576e-03, 2.18722e-02, 00000, -3.01134e-02, -2.46871e-02, 
        1.50873e-04, 1.52050e-02, 1.53982e-02, 00000, -3.57034e-02, 1.50873e-04, 
        1.00462e-03, -2.98669e-03, -3.70581e-03, 00000, 7.77926e-03, 1.00462e-03,
        5.23278e-01, -1.28342e+00, 9.02228e-02, 00000, 1.19319e+00, 5.23278e-01, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        -5.60449e-02, 5.83578e-02, 1.01106e-01, 00000, -1.84972e-01, -5.60449e-02, 
        1.43743e-02, -2.60000e-02, -3.92563e-02, 00000, 7.60744e-02, 1.43743e-02, 
        -1.02557e-03, 2.90664e-03, 4.07095e-03, 00000, -8.15702e-03, -1.02557e-03, 
        2.92287e-14, -1.10021e+00, 5.97412e-01, 00000, 4.04800e-01, 2.92287e-14, 
        -3.71185e-01, -1.71385e-02, 1.82110e-01, 00000, -1.77470e-01, -3.71185e-01, 
        -1.97438e-02, -9.47678e-02, -1.03671e-01, 00000, 2.34151e-01, -1.97438e-02, 
        2.45425e-02, -4.96137e-03, -1.94019e-02, 00000, 2.75517e-02, 2.45425e-02, 
        1.63390e-05, 1.46930e-02, 1.77370e-02, 00000, -3.81238e-02, 1.63390e-05, 
        -3.15376e-04, -2.03572e-03, -2.28639e-03, 00000, 5.09483e-03, -3.15376e-04, 
        -7.08626e-01, -1.44733e+00, 5.65348e-01, 00000, 8.81978e-01, -7.08626e-01, 
        1.86223e-12, -2.79574e-12, -3.72651e-12, 00000, 7.45413e-12, 1.86223e-12, 
        1.17950e-01, 1.07835e-01, 6.57947e-02, 00000, -2.09341e-01, 1.17950e-01, 
        4.30827e-14, 8.48837e-13, 1.00168e-12, 00000, -2.17781e-12, 4.30827e-14, 
        -4.39893e-03, -1.38070e-02, -1.42724e-02, 00000, 3.32039e-02, -4.39893e-03, 
        3.15376e-04, 2.03572e-03, 2.28639e-03, 00000, -5.09483e-03, 3.15376e-04, 
        -1.76459e-12, -1.10021e+00, 5.97412e-01, 00000, 4.04800e-01, -1.76459e-12, 
        3.71185e-01, 1.71385e-02, -1.82110e-01, 00000, 1.77470e-01, 3.71185e-01, 
        -1.97438e-02, -9.47678e-02, -1.03671e-01, 00000, 2.34151e-01, -1.97438e-02, 
        -2.45425e-02, 4.96137e-03, 1.94019e-02, 00000, -2.75517e-02, -2.45425e-02, 
        1.63390e-05, 1.46930e-02, 1.77370e-02, 00000, -3.81238e-02, 1.63390e-05, 
        1.02557e-03, -2.90664e-03, -4.07095e-03, 00000, 8.15702e-03, 1.02557e-03;

    cout<<"getpoly OK"<<endl;
}


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

void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for(int i = 0; i < 6;i++){
        nowRobotAngle(i)=msg->position[i];
    }

    cout<<"nowRobotAngle: "<<nowRobotAngle.transpose()<<endl;
}

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

    // VectorXd robotAngle(6);
    // int pubnum;
    // while(1){
    //     cout<<"select pubnum:";
    //     cin>>pubnum;
    //     if(pubnum < wp_list.size()){
    //         robotAngle = wp_list[pubnum];
    //         for(int i = 0; i < 6; i++){
    //             init_pos.data.at(i) = robotAngle(i);
    //         }
    //         pos_pub.publish(init_pos);
    //         cout<<"pub no."<<pubnum<<endl;
    //     }
    // }


    //genTrajectory(wp_list);

    Matrix<double, 54, 6> polyCoeff;
    getPolyCoeff(polyCoeff);
    double fseg_t = 4;
    double seg_t = 2.8;

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
    // cin>>cnt;
    while (ros::ok()){
        ros::Time nowtime=ros::Time::now();
        t = nowtime.toSec() - begin.toSec();
        //cout<<endl<<"-------------------"<<"t: "<<t<<endl;
        t -= cnt*seg_t*4;

        if(abs(t-(fseg_t+seg_t*6))<0.01){
            ring = true;
        }


        if(t > fseg_t+seg_t*8){
            startPoint = true;
            cnt++;
            t -= cnt*seg_t*4;
            if(cnt >= 8){
                break;
            }
        }
        

        getrobotv(Robotv, t, polyCoeff, ts);
        
        for(int i = 0; i < 6; i++){
            init_pos.data.at(i) = Robotv(i);// + (Robotv(i) - RobotvLast(i))*0.01;
        }
        //cout<<"pub: "<<Robotv.transpose()<<endl;

        All_roboAngle += Robotv * (t-tLast);

        RobotvLast = Robotv;
        tLast = t;

        vel_pub.publish(init_pos);


        if(startPoint){
            startPoint = false;
            ros::spinOnce();

            cout<<endl<<"-------------------"<<"t: "<<nowtime.toSec() - begin.toSec()<<endl;
            cout<<"No."<<cnt+2<<" Back To Start!!!!!"<<endl;

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


    if(startPoint){
        startPoint = false;
        ros::spinOnce();
        cout<<"Link6Pose: "<< Link6Pose.transpose()<<endl;

        double resuial = (nowRobotAngle - wp_list[1]).block(0,0,3,1).cwiseAbs().sum();
        cout<<"resuial to startPoint: "<<resuial<<endl;
    }

    cout<<"All_roboAngle"<<All_roboAngle.transpose()<<endl;

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