#include "sender.h"
#include <cstring>
#include <iostream>
#include <unistd.h> // 用于usleep
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;

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
        while (ros::ok()) {
            // 发送自动模式指令
            sender.send_auto_mode();
            // 发送ComplexCmd
            sender.send_complex_cmd(-0.5);    // 旋转
            // sender.send_complex_cmd(0.2, 0x145);   // 左右平移
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