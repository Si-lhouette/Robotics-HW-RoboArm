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

VectorXd nowRobotAngle(6);
Vector3d Link6Pose;






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


void getPolyCoeff(Matrix<double, 18, 6>& polyCoeff){
    polyCoeff << 00000, 00000, 00000, 00000, 00000, 00000, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        00000, 00000, 00000, 00000, 00000, 00000, 
        3.04987e-03, -2.95869e-03, 1.38516e-03, 4.51506e-03, 9.80616e-03, -4.05767e-03, 
        -3.07095e-04, 3.03916e-04, -1.71136e-04, -4.73380e-04, -1.01242e-03, 4.53304e-04, 
        8.16666e-06, -8.53154e-06, 5.31256e-06, 1.30608e-05, 2.82854e-05, -1.31823e-05, 
        7.95586e-01, -7.72686e-01, 2.05056e-01, 1.08734e+00, 2.51046e+00, -8.42853e-01, 
        9.49137e-02, -9.85209e-02, -3.36754e-03, 1.14038e-01, 3.06422e-01, -6.31976e-02, 
        -1.10944e-02, 8.27334e-03, -8.00106e-03, -1.79684e-02, -3.04157e-02, 1.84296e-02, 
        -1.06728e-03, 6.66399e-04, -1.47706e-04, -1.35937e-03, -2.40541e-03, 8.92206e-04, 
        1.01238e-04, -1.22661e-04, 9.44925e-05, 1.79658e-04, 4.01846e-04, -2.05811e-04, 
        -1.92703e-06, 4.15605e-06, -3.68377e-06, -4.63720e-06, -1.27848e-05, 6.82848e-06, 
        3.87677e-01, -1.07517e+00, -1.99884e-01, 4.04374e-01, 2.86768e+00, -1.14923e-01, 
        -1.38559e-01, -1.59769e-02, -1.39193e-02, -1.66367e-01, -5.53712e-02, 9.12371e-02, 
        -1.64047e-03, -3.77094e-03, 7.42551e-03, 2.67359e-03, 1.06815e-02, -1.00059e-02, 
        1.05520e-03, -8.40012e-05, -5.17808e-05, 1.18976e-03, 8.83619e-04, -5.11746e-04, 
        4.88624e-06, 8.51412e-05, -8.96962e-05, -5.22017e-05, -2.37394e-04, 1.35613e-04, 
        -3.29472e-06, -4.47938e-06, 4.79457e-06, -7.04551e-07, 1.05247e-05, -5.60096e-06;
}


void getrobotv(VectorXd& Robotv, const double t, Matrix<double, 18, 6>& polyCoeff, VectorXd& ts){
    //Get Segment number
    int TheSeg = 0;
    int n_para_per_seg = 6;
    double t_inseg;
    double ts_acc = 0;
    for(int i = 0; i < ts.size(); i++){
        if(t <= ts_acc + ts(i)){
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

    //cout<<"nowRobotAngle: "<<nowRobotAngle.transpose()<<endl;
}

void LinkStatesCallBack(const gazebo_msgs::LinkStatesConstPtr& msg){
	geometry_msgs::Pose Link6msg = msg->pose[7];

	Link6Pose << Link6msg.position.x, Link6msg.position.y, Link6msg.position.z;
    // cout<<endl<<"-------------------------"<<endl;
    // cout<<msg->name[7]<<endl;
    // cout<<"Pose: "<<endl<<Link6Pose.transpose()<<endl;

    // Quaterniond Link6q(Link6msg.orientation.w, Link6msg.orientation.x, Link6msg.orientation.y, Link6msg.orientation.z);
	// Vector3d eulerAngle1=Link6q.matrix().eulerAngles(0,1,2);
    // cout << "EulerAngles:"<<endl<<eulerAngle1<<endl<<endl;

    geometry_msgs::Twist Tw6 = msg->twist[7];

    // //全局线速度
    // cout<<"linear speed: "<<Tw6.linear.x<<", "<<Tw6.linear.y<<", "<<Tw6.linear.z<<endl;
    // //全局角速度
    // cout<<"angle speed: "<<Tw6.angular.x<<", "<<Tw6.angular.y<<", "<<Tw6.angular.z<<endl;
    

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_gen");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/probot_anno/joint_states", 1, jointstatesCallback);
    ros::Subscriber sublink6 = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, LinkStatesCallBack);
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);
    ros::Rate loopHZ(200);

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

    Matrix<double, 18, 6> polyCoeff;
    getPolyCoeff(polyCoeff);

    VectorXd Robotv(6);
    VectorXd ts(3);
    ts << 10,10,10;


    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);

    ros::Time begin=ros::Time::now();
    double t;
    int cnt = 0;
    // cin>>cnt;
    while (ros::ok()){
        ros::Time nowtime=ros::Time::now();
        t = nowtime.toSec() - begin.toSec();
        if(t > 30.0){
            break;
        }
        //cout<<endl<<"-------------------"<<"t: "<<t<<endl;
        getrobotv(Robotv, t, polyCoeff, ts);
        for(int i = 0; i < 6; i++){
            init_pos.data.at(i) = Robotv(i);
        }
        //cout<<"pub: "<<Robotv.transpose()<<endl;

        vel_pub.publish(init_pos);

        if(abs(t-10)<0.02 || abs(t-20)<0.02 || abs(t-30)<0.02){
            cout<<endl<<"-------------------"<<"t: "<<t<<endl;
            ros::spinOnce();
            cout<<"Link6Pose: "<< Link6Pose.transpose()<<endl;
            cout<<"nowRobotAngle: "<<nowRobotAngle.transpose()<<endl;
            cnt = 0;
            for(int i = 1; i < wp_list.size(); i++){
                double resuial = (nowRobotAngle - wp_list[i]).block(0,0,3,1).cwiseAbs().sum();
                cout<<endl<<"resuial to No."<<i<<" : "<<resuial<<endl;
                if( resuial < 0.1 ){
                    cout<< "Get wayPoint No."<<i<<"   !!!!!!!!!!!!!!!"<<endl;
                }
            }
        }
        
        loopHZ.sleep();
        cnt++;
    }

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