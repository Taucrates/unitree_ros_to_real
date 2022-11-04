#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/ControllerJoystick.h>
#include <unitree_legged_msgs/Ranges.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>

string odom_frame = "odom";
string footprint_frame = "base_footprint";
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
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)) // "192.168.123.161" To control via Ethernet || "192.168.12.1" Via Wi-Fi
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
ros::Publisher pub_controller;
ros::Publisher pub_ranges;

long high_state_seq = 0;
long cmd_vel_count = 0;

ros::Time last_update;

double pos_x_odom = 0.0;
double pos_y_odom = 0.0;

double yaw = 0.0;
double first_yaw = 0.0;
bool first_yaw_read = true;
bool publish_odom_tf = false;
bool publish_footprint_tf = true;

float getFloat(uint8_t bytes[4]){
  float f;
  uint8_t b[4] = {bytes[0], bytes[1], bytes[2], bytes[3]};
  memcpy(&f, &b, sizeof(f));
  return f;
}

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
    if(sqrt(high_state_ros.velocity[0]*high_state_ros.velocity[0]) > 0.1 && sqrt(high_state_ros.velocity[0]*high_state_ros.velocity[0]) < 3.0)
    {
        if(high_state_ros.velocity[0] > 0.0)
        {
            vel_x_robot = 0.91 * high_state_ros.velocity[0];
        } else {
            vel_x_robot = 1.3 * high_state_ros.velocity[0];
        }
        
    } else {
        if(high_state_ros.velocity[0] < 0.0 && high_state_ros.velocity[0] > -0.05){
            vel_x_robot = 4.0 * high_state_ros.velocity[0];
        } else {
            vel_x_robot = 1.7 * high_state_ros.velocity[0];
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

    if(publish_odom_tf){

        // TF odom -> base_footprint
        static tf::TransformBroadcaster br;

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0));
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, -yaw);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, footprint_frame));
    }

    if(publish_footprint_tf){
        // TF base_footprint --> base_link
        static tf::TransformBroadcaster br_2;

        tf::Transform transform_2;
        transform_2.setOrigin(tf::Vector3(0.0, 0.0, odom_msg.pose.pose.position.z));
        tf::Quaternion q_2;
        q_2.setRPY(high_state_ros.imu.rpy[0], high_state_ros.imu.rpy[1], 0.0);
        transform.setRotation(q_2);

        br_2.sendTransform(tf::StampedTransform(transform_2, ros::Time::now(), footprint_frame, base_frame));
    }

    // Publish Joysticks Controller

    unitree_legged_msgs::ControllerJoystick controller;
    
    // printf("\n( ");
    uint8_t aux_controller[4];
    
    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        aux_controller[i] = high_state_ros.wirelessRemote[i+20];
        //printf("%d, ", controller.leftFrontal[i]);
    }
    //printf(")\n");
    controller.leftFrontal = getFloat(aux_controller);

    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        aux_controller[i] = high_state_ros.wirelessRemote[i+4];
        //printf("%d, ", controller.leftLateral[i]);
    }
    //printf(")\n");
    controller.leftLateral = getFloat(aux_controller);

    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        aux_controller[i] = high_state_ros.wirelessRemote[i+12];
        //printf("%d, ", controller.rightFrontal[i]);
    }
    //printf(")\n");
    controller.rightFrontal = getFloat(aux_controller);

    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        aux_controller[i] = high_state_ros.wirelessRemote[i+8];
        //printf("%d, ", controller.rightLateral[i]);
    }
    //printf(")\n");
    controller.rightLateral = getFloat(aux_controller);

    controller.stSelRsLs = high_state_ros.wirelessRemote[2];
    controller.arrowsLetters = high_state_ros.wirelessRemote[3];

    pub_controller.publish(controller);

    // Publish Ultra Sound Ranges
    unitree_legged_msgs::Ranges ranges;
    ranges.left = high_state_ros.rangeObstacle[1];
    ranges.front = high_state_ros.rangeObstacle[0];
    ranges.right = high_state_ros.rangeObstacle[2];

    pub_ranges.publish(ranges);

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
    ros::NodeHandle nh_private("~");

    ros::Timer timer = nh.createTimer(ros::Duration(0.01), highStateCallback); //100Hz

    nh_private.param<bool>("publish_tf", publish_tf, false);
    //ROS_INFO("Publish_odom_tf: %s", publish_tf ? "true" : "false");

    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("dog_imu/data", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("dog_odom", 1);
    pub_controller = nh.advertise<unitree_legged_msgs::ControllerJoystick>("controller", 1);
    pub_ranges = nh.advertise<unitree_legged_msgs::Ranges>("ranges", 1);

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();

    ros::spin();

    return 0;
}
