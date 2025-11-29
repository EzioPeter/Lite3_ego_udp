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
#include <algorithm>

using namespace std;

static std::atomic<int16_t> g_go_flag{0};
static geometry_msgs::TwistStamped cmd_vel;

void goFlagCallback(const std_msgs::Int16::ConstPtr &msg)
{
    g_go_flag.store(msg->data);
    ROS_DEBUG("Received go_flag = %d", msg->data);
}

void cmdVelQuadrupedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    ROS_INFO("[cmd_vel_quadruped] stamp=%.3f frame_id=%s lin=(%.3f,%.3f,%.3f) ang=(%.3f,%.3f,%.3f)",
             msg->header.stamp.toSec(), msg->header.frame_id.c_str(),
             msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
             msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);

    // 保存最新收到的消息到全局变量（直接赋值，适用于单线程 spinOnce 场景）
    cmd_vel = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lite3_ego_udp");
    ros::NodeHandle nh("~");
    // 限幅参数（可以通过 rosparam 或 launch 覆盖）
    double max_vel_x = 1.0;      // m/s
    double max_vel_y = 0.5;      // m/s
    double max_yaw_rate = 1.0;   // rad/s
    nh.param("max_vel_x", max_vel_x, max_vel_x);
    nh.param("max_vel_y", max_vel_y, max_vel_y);
    nh.param("max_yaw_rate", max_yaw_rate, max_yaw_rate);
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
        ros::Subscriber cmd_vel_quad_sub = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel_quadruped", 10, cmdVelQuadrupedCallback);

        // int16_t last_go = -1;
        while (ros::ok()) {
            // 发送自动模式指令
            // sender.send_auto_mode();
            // 发送ComplexCmd
            // sender.send_complex_cmd(-0.5);    // 旋转
            // sender.send_complex_cmd(0.2, 0x145);   // 左右平移
            // 读取最新的 go_flag（非阻塞）
            int16_t go_val = g_go_flag.load();
            // if (go_val != last_go) {
            //     ROS_INFO("go_flag changed: %d", go_val);
            //     last_go = go_val;
            // }
            if (go_val){
                double vel_x = cmd_vel.twist.linear.x;
                double vel_y = cmd_vel.twist.linear.y;
                double yaw_rate = cmd_vel.twist.angular.z;

                // 限幅
                if (vel_x > max_vel_x) vel_x = max_vel_x;
                if (vel_x < -max_vel_x) vel_x = -max_vel_x;
                if (vel_y > max_vel_y) vel_y = max_vel_y;
                if (vel_y < -max_vel_y) vel_y = -max_vel_y;
                if (yaw_rate > max_yaw_rate) yaw_rate = max_yaw_rate;
                if (yaw_rate < -max_yaw_rate) yaw_rate = -max_yaw_rate;

                sender.send_complex_cmd(-yaw_rate);
                sender.send_complex_cmd(vel_y, 0x145);
                sender.send_complex_cmd(vel_x, 0x140);
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