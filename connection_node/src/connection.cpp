//
// Created by jiashi on 2020/8/5.
// ROS过渡模块，运行于绝影mini的NUC
//
#include "PracticalSocket.h" // For UDPSocket and SocketException
#include <iostream>          // For cout and cerr
#include <cstdlib>           // For atoi()
#include <ros/ros.h>
#include <pthread.h>
#include <map>
#include <ctime>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include "tf/transform_datatypes.h"
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <move_base/>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class connection{
public:
    ros::NodeHandle n;
    // ROS订阅与发布器
    ros::Subscriber status_sub;
    ros::Subscriber pose_sub;
    ros::Publisher goal_pub;

    geometry_msgs::PoseStamped goal_pose;

    // 位姿矫正状态量
    bool if_correcting;
    // 判断现在是否有消息在堵塞
    bool if_blocked;
    // 目标点的欧拉角
    double goal_roll{}, goal_pitch{}, goal_yaw{};
    // Socket参数
    int ECHOMAX;     // Longest string to echo
    char *echoBuffer;         // Buffer for echo string
    int recvMsgSize;                  // Size of received message
    string sourceAddress;             // Address of datagram source
    unsigned short sourcePort;        // Port of datagram source
    unsigned short echoServPort;     // First arg:  local port
    // 每个动作对应的动作代码
    std::map<char, int[2]> mymap;

    connection(const ros::NodeHandle& nh){
        cout << "11111111111111111111111\n";
        ECHOMAX = (3);
        cout << "222222222222222\n";

        if_blocked = (false);
        cout << "3333\n";

        if_correcting = (false);
        cout << "444444\n";

        recvMsgSize = (0);
        cout << "5\n";

        sourceAddress = "";
        cout << "6\n";

        sourcePort = (0);
        cout << "7\n";

        echoServPort = (33177);
        cout << "8\n";

        echoBuffer = new char[ECHOMAX];
        cout << "9\n";

        // 对应动作代码和时间
        mymap['s'][0] = 0;        mymap['s'][1] = 5;
        mymap['c'][0] = 0;        mymap['c'][1] = 5;
        mymap['k'][0] = 0;        mymap['k'][1] = 5;
        mymap['w'][0] = 0;        mymap['w'][1] = 5;
        mymap['h'][0] = 0;        mymap['h'][1] = 5;

        cout << "11111111111111111111111\n";
        goal_pose.pose.position.x = 0.0;
        goal_pose.pose.position.y = 0.0;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 0.0;
        cout << "11111111111111111111111\n";

        tf::Quaternion quat;
        tf::quaternionMsgToTF(goal_pose.pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(goal_roll, goal_pitch, goal_yaw);

        n = nh;
        pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/ndt/current_pose", 2000, &connection::poseHandler, this, ros::TransportHints().tcpNoDelay());
        status_sub = n.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 2000, &connection::statusHandler, this, ros::TransportHints().tcpNoDelay());
        goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    }

    // TODO 查看发布的频率
    void statusHandler(const move_base_msgs::MoveBaseActionResult::ConstPtr& resultMsg){
        cout << "aaa\n";

    }

    // TODO 查看发布的频率
    void poseHandler(const geometry_msgs::PoseStamped::ConstPtr& PoseMsg){
        double delta_x = PoseMsg->pose.position.x - goal_pose.pose.position.x;
        double delta_y = PoseMsg->pose.position.y - goal_pose.pose.position.y;
        double delta_distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2) );
        double roll, pitch, yaw;
        tf::Quaternion current_qua;
        tf::quaternionMsgToTF(PoseMsg->pose.orientation, current_qua);
        tf::Matrix3x3(current_qua).getRPY(roll, pitch, yaw);
        // TODO 查看是不是yaw角
        double delta_yaw = abs(yaw - goal_yaw);
        // TODO 修改这个阈值
        if (delta_distance > 0.1 || delta_yaw > 0.1){
            if_correcting = true;
        }
    }

    static bool sendcommand(int content){
        UDPSocket sock;
        int echocontent[3] = {content, 0, 0};
        sock.sendTo(echocontent, 3, "192.168.1.120", 43893);
    }

    void start_connecting(){
        try {
            UDPSocket sock(echoServPort);

            for (;;) {
                // 接受动作识别传输回来的指令
                // NOTE 这里会一直等待消息进入，然后再执行下一步
                recvMsgSize = sock.recvFrom(echoBuffer, ECHOMAX, sourceAddress, sourcePort);

                cout << "Received packet from " << sourceAddress << ":"
                     << sourcePort << "Content::: " << echoBuffer[0]  << endl;
//                // 如果目前仍有消息在阻塞，未执行，就舍弃最新的动作
//                if(if_blocked)
//                    continue;

                // 根据收到的动作代码发送对应的动作
                sendcommand(mymap[echoBuffer[0]][0]);

                // 暂停这一动作对应的秒数
                sleep(mymap[echoBuffer[0]][1]);

//                // 查看一下新的数据，判断是否要进行位姿矫正
//                ros::spinOnce();
//                // TODO 频率之后要调整 或者删除
//                ros::Rate r(100);
//                r.sleep();
                // 如果需要矫正     以下if判断进行矫正
                if(if_correcting){
                    // 开始踏步
                    sendcommand(3);
                    // 切换导航模式
                    sendcommand(23);
//                    cout << "iii\n";
                    MoveBaseClient mc_("move_base", true);
                    move_base_msgs::MoveBaseGoal naviGoal;
                    naviGoal.target_pose.header.frame_id = "map";
                    naviGoal.target_pose.header.stamp = ros::Time::now();
                    naviGoal.target_pose.pose.position = goal_pose.pose.position;
                    naviGoal.target_pose.pose.orientation = goal_pose.pose.orientation;
                    mc_.sendGoal(naviGoal);
                    for(;;){
                        if(mc_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
                            usleep(500000);
                        } else{
                            if_correcting = false;
                            break;
                        }
                    }
                    // 切换手动模式
                    sendcommand(24);
                    // 切换成静止站立
                    sendcommand(3);
                }
            }
        } catch (SocketException &e) {
            cerr << e.what() << endl;
            exit(1);
        }
    }
};

void* check_pose(void* args){
    ros::spin();
}

int main(int argc, char **argv) {
    // 正常版本

    ros::init(argc, argv, "connection_node");
    ros::NodeHandle n("~");
    connection C(n);
    ROS_INFO("\033[1;32m----> start connecting.\033[0m");

    // 测试多线程版本
    pthread_t tids[3];
    // 第一个线程用于接收Topic消息，判断是否要矫正
    int ret = pthread_create(&tids[0], NULL, check_pose, NULL);

    // 用于接收动作识别工程的消息，进行中继
    C.start_connecting();


//    cout << "111\n";
//    ret = pthread_create(&tids[1], NULL, say_hello, NULL);
//    cout << "111\n";
//    ret = pthread_create(&tids[2], NULL, say_hello, NULL);
//    cout << "111\n";
//    ros::spin();

    return 0;
}

