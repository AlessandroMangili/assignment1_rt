#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <math.h>

const float THR = 1.0;

double x[2], y[2], z[2];

void turtlesim1Callback(const turtlesim::Pose::ConstPtr& msg) {
    //ROS_INFO("The position of the turtle 1 is: %f, %f, %f", msg->x, msg->y, msg->theta);
    x[0] = msg->x;
    y[0] = msg->y;
    z[0] = msg->theta;
}

void turtlesim2Callback(const turtlesim::Pose::ConstPtr& msg) {
    //ROS_INFO("The position of the turtle 2 is: %f, %f, %f", msg->x, msg->y, msg->theta);
    x[1] = msg->x;
    y[1] = msg->y;
    z[1] = msg->theta;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "turtlesim_listener2");
    ros::NodeHandle n;

    ros::Subscriber turtle1_sub = n.subscribe("/turtle1/pose", 10, turtlesim1Callback);
    ros::Subscriber turtle2_sub = n.subscribe("/turtle2/pose", 10, turtlesim2Callback);

    sleep(1);

    ros::Publisher turtles_distance_pub = n.advertise<std_msgs::Float32>("/turtles/rel_distance", 100);
    std_msgs::Float32 distance;
    float last_stop_distance;

    ros::Rate loop_rate(100);

    ros::Publisher turtle1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    ros::Publisher turtle2_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);
    geometry_msgs::Twist turtle1_vel, turtle2_vel;
    while (ros::ok()) {
        distance.data = sqrt(pow(x[0] - x[1], 2) + pow(y[0] - y[1], 2));
        turtles_distance_pub.publish(distance);

        ros::Time start_time = ros::Time::now();
        if (distance.data <= THR && distance.data != last_stop_distance) {
            while (ros::Time::now() - start_time < ros::Duration(1.0)) {
                ROS_INFO("STOP BOTH TURTLES BECAUSE THEY ARE TOO CLOSE [%f]: %f, %f, %f - %f, %f, %f", distance.data, x[0], y[0], z[0], x[1], y[1], z[1]);
                turtle1_vel.linear.x = 0;
                turtle1_vel.linear.y = 0;
                turtle1_vel.angular.z = 0;
                turtle2_vel.linear.x = 0;
                turtle2_vel.linear.y = 0;
                turtle2_vel.angular.z = 0;
                turtle1_pub.publish(turtle1_vel);
                turtle2_pub.publish(turtle2_vel);
            }
            last_stop_distance = distance.data;
        } else if (x[0] > 10.0 || x[0] < 1.0 || y[0] > 10.0 || y[0] < 1.0 && distance.data != last_stop_distance) {
            while (ros::Time::now() - start_time < ros::Duration(1.0)) {
                ROS_INFO("STOP THE TURTLE 1 BECAUSE HE HIT THE WALL: %f, %f, %f", x[0], y[0], z[0]);
                turtle1_vel.linear.x = 0;
                turtle1_vel.linear.y = 0;
                turtle1_vel.angular.z = 0;
                turtle1_pub.publish(turtle1_vel);
            }
            last_stop_distance = distance.data;
        } else if (x[1] > 10.0 || x[1] < 1.0 || y[1] > 10.0 || y[1] < 1.0 && distance.data != last_stop_distance) {
            while (ros::Time::now() - start_time < ros::Duration(1.0)) {
                ROS_INFO("STOP THE TURTLE 2 BECAUSE HE HIT THE WALL: %f, %f, %f", x[1], y[1], z[1]);
                turtle2_vel.linear.x = 0;
                turtle2_vel.linear.y = 0;
                turtle2_vel.angular.z = 0;
                turtle2_pub.publish(turtle2_vel);
            }
            last_stop_distance = distance.data;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

