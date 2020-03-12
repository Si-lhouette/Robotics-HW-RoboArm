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
        00000, 00000, 00000, 00000, 00000, 00000, 
        3.72225e-03, -3.70109e-03, 1.30617e-03, 5.28712e-03, 1.20705e-02, -4.43289e-03, 
        -4.89890e-04, 4.92963e-04, -1.95696e-04, -7.09994e-04, -1.61076e-03, 6.17226e-04, 
        7.95499e-01, -7.72673e-01, 2.04805e-01, 1.08572e+00, 2.51044e+00, -8.41728e-01, 
        3.30218e-01, -3.10036e-01, 4.15341e-02, 4.24830e-01, 1.00162e+00, -2.87616e-01, 
        -5.40253e-02, 6.10400e-02, -4.86948e-02, -9.44240e-02, -2.02876e-01, 1.06598e-01, 
        -1.87055e-02, 1.46261e-02, -5.41626e-03, -2.56358e-02, -5.03983e-02, 2.00233e-02, 
        3.20380e-03, -5.21416e-03, 3.36752e-03, 5.98537e-03, 1.61543e-02, -7.25237e-03, 
        -1.19238e-04, 3.68676e-04, -2.63247e-04, -3.14360e-04, -1.07994e-03, 4.89633e-04, 
        3.87524e-01, -1.07534e+00, -1.99874e-01, 4.03282e-01, 2.86851e+00, -1.14568e-01, 
        -3.83667e-01, -5.76533e-02, 9.47933e-03, -4.31782e-01, -1.04661e-01, 1.84032e-01, 
        -3.08554e-03, -4.08494e-02, 4.61304e-02, 2.58950e-02, 1.14373e-01, -6.88663e-02, 
        3.19276e-02, 2.09520e-02, -1.92105e-02, 2.41845e-02, -3.73763e-02, 1.28239e-02, 
        -6.32381e-03, -3.37341e-03, 2.91949e-03, -5.35481e-03, 5.18781e-03, -1.18746e-03, 
        3.78194e-04, 1.86065e-04, -1.56717e-04, 3.31646e-04, -2.65519e-04, 4.37011e-05;
}


void getrobotv(VectorXd& Robotv, const double t, Matrix<double, 18, 6>& polyCoeff, VectorXd& ts){
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
    ros::Rate loopHZ(50);

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
    ts << 5,5,5;


    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);

    TicToc Clock;
    double t;
    int cnt = 0;
    // cin>>cnt;
    while (ros::ok()){
        
        t = Clock.toc();
        if(t > 15.0){
            break;
        }
        //cout<<endl<<"-------------------"<<"t: "<<t<<endl;
        getrobotv(Robotv, t, polyCoeff, ts);
        for(int i = 0; i < 6; i++){
            init_pos.data.at(i) = Robotv(i);
        }
        //cout<<"pub: "<<Robotv.transpose()<<endl;

        vel_pub.publish(init_pos);


        if(abs(t-5)<0.02 || abs(t-10)<0.02 || abs(t-15)<0.02){
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




    cout<< "Done" <<endl;
    ros::spin();
    return 0;
}