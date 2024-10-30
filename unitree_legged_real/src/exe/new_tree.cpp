#include <ros/ros.h>
#include "convert.h"
#include <chrono>
#include <pthread.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argcv){
    ros::init(argc, argcv, "go1_tf_structure_generator");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string fixed_frame;
    std::string odom_frame;
    std::string footprint_frame;
    std::string base_frame;
    std::string static_fast_lio_frame;
    std::string move_fast_lio_frame;

    bool first_loop = true;
    float initial_heigth = 0.0;

    nh_private.param<std::string>("fixed_frame", fixed_frame, "world");
    nh_private.param<std::string>("odom_frame", odom_frame, "odom");
    nh_private.param<std::string>("footprint_frame", footprint_frame, "base_footprint");
    nh_private.param<std::string>("base_frame", base_frame, "base_link");
    nh_private.param<std::string>("static_fast_lio", static_fast_lio_frame, "aux_odom");
    nh_private.param<std::string>("move_fast_lio", move_fast_lio_frame, "aux_base");

    tf::StampedTransform tf_od;
    tf::StampedTransform tf_bl;
    static tf::TransformListener listener;
    ros::Rate rate(40);

    while(ros::ok()){

        try{
            listener.lookupTransform(static_fast_lio_frame, move_fast_lio_frame, ros::Time(0), tf_od);
        }
        catch (tf::TransformException &ex) {
             ros::Duration(0.01).sleep();
        }

        // if(first_loop){
        //     try{
        //         listener.lookupTransform(footprint_frame, base_frame, ros::Time(0), tf_bl);
        //     }
        //     catch (tf::TransformException &ex) {
        //          ros::Duration(0.01).sleep();
        //     }
        //     initial_heigth = tf_bl.getOrigin().z();
        //     first_loop = false;
        // }
        // try{
        //     listener.lookupTransform(footprint_frame, base_frame, ros::Time(0), tf_bl);
        // }
        // catch (tf::TransformException &ex) {
        //         ros::Duration(0.01).sleep();
        // }

        // Fixed frame to odom frame
        // static tf::TransformBroadcaster br;
        // tf::Transform transform;
        // transform.setOrigin(tf::Vector3(0.0, 0.0, + 0.18)) ;
        // tf::Quaternion q = tf_od.getRotation();
        // transform.setRotation(q);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        // transform.setOrigin(tf::Vector3(0.0, 0.0, -tf_od.getOrigin().z()) );
        transform.setOrigin(tf::Vector3(tf_od.getOrigin().x(), tf_od.getOrigin().y(), tf_od.getOrigin().z()) );
        tf::Quaternion q = tf_od.getRotation();
        transform.setRotation(q);

        // Getting rotation in RPY
        // tf::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);
        // tf::Quaternion q2;
        // q2.setRPY(0, 0, yaw);
        // transform.setRotation(q2);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, base_frame));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
