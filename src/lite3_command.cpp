#include "sender.h"
#include <cstring>
#include <iostream>
#include <unistd.h> // 用于usleep
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Int16.h>
#include <atomic>

using namespace std;

static std::atomic<int16_t> g_go_flag{0};

void goFlagCallback(const std_msgs::Int16::ConstPtr &msg)
{
    g_go_flag.store(msg->data);
    ROS_DEBUG("Received go_flag = %d", msg->data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lite3_ego_udp");
    ros::NodeHandle nh("~");
    // 配置服务器IP和端口
    // wireless
    // const string SERVER_IP = "192.168.2.1";
    // ethernet
    const string SERVER_IP = "192.168.1.120";
    const int SERVER_PORT = 43893;

    // 创建发送器实例
    UdpCommandSender sender(SERVER_IP, SERVER_PORT);
    if (!sender.init()) {
        cerr << "Failed to initialize sender" << endl;
        return -1;
    }

    try {
        ROS_INFO("Starting to send commands...");
        ros::Rate rate(200); // ~200Hz, similar to 5ms sleeps
        // subscribe to go_flag published by the planner FSM
        ros::Subscriber go_flag_sub = nh.subscribe<std_msgs::Int16>("/ego_planner_node/go_flag", 10, goFlagCallback);

        int16_t last_go = -1;
        while (ros::ok()) {
            // 发送自动模式指令
            // sender.send_auto_mode();
            // 发送ComplexCmd
            // sender.send_complex_cmd(-0.5);    // 旋转
            // sender.send_complex_cmd(0.2, 0x145);   // 左右平移
            // 读取最新的 go_flag（非阻塞）
            int16_t go_val = g_go_flag.load();
            if (go_val != last_go) {
                ROS_INFO("go_flag changed: %d", go_val);
                last_go = go_val;
            }

            ros::spinOnce();
            rate.sleep();
        }
    } catch (const exception& e) {
        ROS_ERROR("Error in sender loop: %s", e.what());
    }

    // 停止所有指令（程序退出前）
    sender.send_stop_all();
    return 0;
}