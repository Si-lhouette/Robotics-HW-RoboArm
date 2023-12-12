#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "Invert.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Invert");
    ros::NodeHandle nh;
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);

    /* 初始化DH参数 */
    InitialDH();


    /* 设定目标末端位姿 */
    Vector3d aimendPose(0.3, -0.1, 0.122);
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
        inrange = AdjAndInrange(robotAngle);    //检验关节角是否在安全范围内，并补偿偏置


        /* 正运动学验证 */
        cout<<endl<<"----Check "<<i<<"----";
        Vector3d endPose;
        Vector3d endAngle;
        Motion(endPose ,endAngle, robotAngle);
        if( (endPose-aimendPose).cwiseAbs().sum() + (endAngle-aimendAngle).cwiseAbs().sum()< 1e-4){
            cout<<" v!"<<endl;  //验证成功，逆运动学通过正运动学检验
            cout<<"diffabs: "<<(endPose-aimendPose).cwiseAbs().sum() + (endAngle-aimendAngle).cwiseAbs().sum()<<endl;
            cout<<"resEndPose: "<<endl<<endPose.transpose()<<endl;
            cout<<"resEndAngle: "<<endl<<endAngle.transpose()<<endl;
        }else{
            cout<<" x!"<<endl;  //验证失败，逆运动学未通过正运动学检验
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
            robotAngleSetInrange.push_back(robotAngle); //存储在范围内的逆运动学解
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

    //选择pub的解序号
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
        else{
            cout<<"input must smaller than "<<robotAngleSetInrange.size()<<endl;
        }
    }

    ros::spin();
    return 0;
}