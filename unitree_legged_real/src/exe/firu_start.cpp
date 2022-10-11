#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

string odom_frame = "odom";
string base_frame = "base_link";

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)) // "192.168.123.161" To control via Ethernet
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

Custom custom;

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_high;
ros::Publisher pub_imu;
ros::Publisher pub_odom;

long high_state_seq = 0;
long cmd_vel_count = 0;

ros::Time last_update;

double pos_x_odom = 0.0;
double pos_y_odom = 0.0;

double yaw = 0.0;
double first_yaw = 0.0;
bool first_yaw_read = true;

uint8_t controller[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void highStateCallback(const ros::TimerEvent& event)
{   

    // Read the high state of the robot
    unitree_legged_msgs::HighState high_state_ros;
    high_state_ros = state2rosMsg(custom.high_state);

    if(high_state_ros.imu.rpy[2] != 0.0 && first_yaw_read){
        first_yaw = high_state_ros.imu.rpy[2];
        first_yaw_read = false;
    }
    
    // Get actual Time
    ros::Time actual_time = ros::Time::now();

    // Publish sensor_msgs/Imu message
    sensor_msgs::Imu imu_msg;

    imu_msg.header.seq = high_state_seq;
    imu_msg.header.stamp = actual_time;
    imu_msg.header.frame_id = base_frame;

    imu_msg.orientation.x = high_state_ros.imu.quaternion[0];
    imu_msg.orientation.y = high_state_ros.imu.quaternion[1];
    imu_msg.orientation.z = high_state_ros.imu.quaternion[2];
    imu_msg.orientation.w = high_state_ros.imu.quaternion[3];

    imu_msg.angular_velocity.x = high_state_ros.imu.gyroscope[0];
    imu_msg.angular_velocity.y = high_state_ros.imu.gyroscope[1];
    imu_msg.angular_velocity.z = high_state_ros.imu.gyroscope[2];

    imu_msg.linear_acceleration.x = high_state_ros.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = high_state_ros.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = high_state_ros.imu.accelerometer[2];

    pub_imu.publish(imu_msg);

    // Publish nav_msgs/Odometry message
    nav_msgs::Odometry odom_msg;

    odom_msg.header.seq = high_state_seq;
    odom_msg.header.stamp = actual_time;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;

    // odom_msg.pose.pose.position.x = high_state_ros.position[0];
    // odom_msg.pose.pose.position.y = high_state_ros.position[1];
    // odom_msg.pose.pose.position.z = high_state_ros.position[2];

    yaw = - (high_state_ros.imu.rpy[2] - first_yaw);

    double vel_x_robot = 0.0;
    if(sqrt(high_state_ros.velocity[0]*high_state_ros.velocity[0]) > 0.1 && sqrt(high_state_ros.velocity[0]*high_state_ros.velocity[0]) < 2.0)
    {
        if(high_state_ros.velocity[0] > 0.0)
        {
            vel_x_robot = 0.91 * high_state_ros.velocity[0];
        } else {
            vel_x_robot = 1.3 * high_state_ros.velocity[0];
        }
        
    }

    double vel_y_robot = 0.0;
    if(sqrt(high_state_ros.velocity[1]*high_state_ros.velocity[1]) > 0.05 && sqrt(high_state_ros.velocity[1]*high_state_ros.velocity[1]) < 1.1)
    {
        vel_y_robot = high_state_ros.velocity[1];
    }


    odom_msg.pose.pose.position.x = pos_x_odom + ((vel_x_robot * cos(yaw)) * (actual_time.toSec() - last_update.toSec()) + (vel_y_robot * sin(yaw)) * (actual_time.toSec() - last_update.toSec()));
    odom_msg.pose.pose.position.y = pos_y_odom + ((vel_y_robot * cos(yaw)) * (actual_time.toSec() - last_update.toSec()) + (vel_x_robot * sin(-yaw)) * (actual_time.toSec() - last_update.toSec()));
    odom_msg.pose.pose.position.z = high_state_ros.position[2];
    pos_x_odom = odom_msg.pose.pose.position.x;
    pos_y_odom = odom_msg.pose.pose.position.y;

    odom_msg.pose.pose.orientation.x = high_state_ros.imu.quaternion[0]; // Unitree doesn't provide orientation based on kinematics.
    odom_msg.pose.pose.orientation.y = high_state_ros.imu.quaternion[1];
    odom_msg.pose.pose.orientation.z = high_state_ros.imu.quaternion[2];
    odom_msg.pose.pose.orientation.w = high_state_ros.imu.quaternion[3];

    odom_msg.twist.twist.linear.x = high_state_ros.velocity[0];
    odom_msg.twist.twist.linear.y = high_state_ros.velocity[1];
    odom_msg.twist.twist.linear.z = 0.0; // Unitree doesn't provide z velocity.

    odom_msg.twist.twist.angular.x = 0.0; // Unitree doesn't provide roll.
    odom_msg.twist.twist.angular.y = 0.0; // Unitree doesn't provide pitch.
    odom_msg.twist.twist.angular.z = high_state_ros.yawSpeed;

    pub_odom.publish(odom_msg);
    
    // Alarm messages to stop autonomous behavior
    if(high_state_ros.wirelessRemote[2] != 0 || high_state_ros.wirelessRemote[3] != 0)
    {
        printf("Stop autonomous behavior\n");
    }

    /*printf("\n( ");
    for(uint8_t i = 0 ; i < 10 ; i++)
    {
        controller[i] = high_state_ros.wirelessRemote[i+2];
        printf("%d, ", controller[i]);
    }
    printf(")\n");*/

    // Publish high_state message


    pub_high.publish(high_state_ros);

    high_state_seq++;

    last_update = actual_time;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    // printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    // printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    // printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    // printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    // unitree_legged_msgs::HighState high_state_ros;

    // high_state_ros = state2rosMsg(custom.high_state);

    // pub_high.publish(high_state_ros);

    // printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_sub");

    ros::NodeHandle nh;

    ros::Timer timer = nh.createTimer(ros::Duration(0.01), highStateCallback); //100Hz

    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    ros::spin();

    return 0;
}
