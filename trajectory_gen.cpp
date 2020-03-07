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
// 第二个点:[0.3, 0.25, 0.322, 1.57, -1.57, 0]
// 第三个点:[0.3, 0.1, 0.172, 1.57, -1.57, 0]
// 第四个点:[0.3, -0.1, 0.122, 1.57, -1.57, 0]









void initialWayPoints(vector<VectorXd>& wp_list){

    vector<Eigen::VectorXd> robotAngleSet;
    

    vector<VectorXd> pose_list;
    VectorXd onePose(6);
    onePose << 0.2289, 0, 0.454, 1.57, 0, 0;
    pose_list.push_back(onePose);
    onePose << 0.3, 0.25, 0.322, 1.57, -1.57, 0;
    pose_list.push_back(onePose);
    onePose << 0.3, 0.1, 0.172, 1.57, -1.57, 0;
    pose_list.push_back(onePose);
    onePose << 0.3, -0.1, 0.122, 1.57, -1.57, 0;
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

    MatrixXd waypoints(wp_list.size(), 3);
    int d_order = 3;
    MatrixXd polyCoeff;
    
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k) = wp_list[k];

    MatrixXd vel = MatrixXd::Zero(2, 3); 
    MatrixXd acc = MatrixXd::Zero(2, 3);

    // generete start and end derivative constraints 
    MatrixXd derivative_cons = MatrixXd::Zero(2*d_order, 3);
    derivative_cons << waypoints.row(0),
                       vel.row(0),
                       acc.row(0),
                       waypoints.row(waypoints.rows()-1),
                       vel.row(1),
                       acc.row(1);


    VectorXd ts(waypoints.rows() - 1);
    for (int k = 0; k < waypoints.rows()-1; k++) {
        ts(k) = 1;
    }


    MinimumSnapSolver minimum_snap_solver(d_order, waypoints, derivative_cons, ts);

    polyCoeff = minimum_snap_solver.QPSolver();

    cout<<"polyCoeff:"<<endl<<polyCoeff<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_gen");
    ros::NodeHandle nh;
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);
    ros::Rate loopHZ(50);

    InitialDH();

    vector<VectorXd> wp_list;
    initialWayPoints(wp_list);
    //genTrajectory(wp_list);



    /* Pub关节角 */
    std_msgs::Float64MultiArray init_pos;

    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);



    VectorXd robotAngle(6);
    int pubnum;
    while(1){
        cout<<"select pubnum:";
        cin>>pubnum;
        if(pubnum < wp_list.size()){
            robotAngle = wp_list[pubnum];
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