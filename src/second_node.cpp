#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <obs_message/Double16.h>   // It does not exist in std_msgs
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <math.h>

const float THR = 1.0;      // Threshold to avoid collision between the two turtles
const double MIN = 1.0;     // Minimum threshold to avoid collisions with the edges
const double MAX = 10.0;    // Maximum threshold to avoid collisions with the edges

const double THR_OBS = 1.0  // Threshold to avoid collision with the obstacles
const int ANGLES = 16       // Number of different angles provided by the topic \obstacles

double x[2] = {5.544445, 2}, y[2] = {5.544445, 1}, z[2] = {0, 0};
double x_vel[2], y_vel[2], z_vel[2];    // Contain the speed of the two turtles
bool is_moving_t1 = false;
double x_prev, y_prev;

double[ANGLES] obs_distance;

void obstaclesCallback(const obs_message::Double16::ConstPtr& msg) {
    obs_potision = msg; // Maybe it needs a casting or a for loop to do this
}

void turtlesim1CallbackPose(const turtlesim::Pose::ConstPtr& msg) {
    x_prev = x[0];
    y_prev = y[0];

    x[0] = msg->x;
    y[0] = msg->y;
    z[0] = msg->theta;
    // Calculate if turtle 1 is moving
    is_moving_t1 = (fabs(x[0] - x_prev) > 0.01 || fabs(y[0] - y_prev) > 0.01);
}

void turtlesim2CallbackPose(const turtlesim::Pose::ConstPtr& msg) {
    x[1] = msg->x;
    y[1] = msg->y;
    z[1] = msg->theta;
}
// Set the speed of turtle 1 if it is not zero
void turtlesim1CallbackVel(const geometry_msgs::Twist::ConstPtr& msg) {
    if (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0) return;

    x_vel[0] = msg->linear.x;
    y_vel[0] = msg->linear.y;
    z_vel[0] = msg->angular.z;
}
// Set the speed of turtle 2 if it is not zero
void turtlesim2CallbackVel(const geometry_msgs::Twist::ConstPtr& msg) {
    if (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0) return;

    x_vel[1] = msg->linear.x;
    y_vel[1] = msg->linear.y;
    z_vel[1] = msg->angular.z;
}
// Stop the moving turtle by setting the velocity to zero
void stop_turtle(ros::Publisher& turtle_pub, geometry_msgs::Twist turtle_vel) {
    turtle_vel.linear.x = 0;
    turtle_vel.linear.y = 0;
    turtle_vel.angular.z = 0;
    turtle_pub.publish(turtle_vel);
}
// Send the turtle backward by publishing the opposite velocity
void reverse_velocity(int id, ros::Publisher& turtle_pub, geometry_msgs::Twist turtle_vel) {
    turtle_vel.linear.x = -x_vel[id];
    turtle_vel.linear.y = -y_vel[id];
    turtle_vel.angular.z = -z_vel[id];
    turtle_pub.publish(turtle_vel);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "turtlesim_listener2");
    ros::NodeHandle n;

    ros::Subscriber obstacles = n.subscribe("/obstacles", 1000, obstaclesCallback);

    // Subscribers to get the updated position of the two turtles
    ros::Subscriber turtle1_sub_pose = n.subscribe("/turtle1/pose", 1000, turtlesim1CallbackPose);
    ros::Subscriber turtle2_sub_pose = n.subscribe("/turtle2/pose", 1000, turtlesim2CallbackPose);
    // Subscribers to get the updated velocity of the two turtles
    ros::Subscriber turtle1_sub_vel = n.subscribe("/turtle1/cmd_vel", 1000, turtlesim1CallbackVel);
    ros::Subscriber turtle2_sub_vel = n.subscribe("/turtle2/cmd_vel", 1000, turtlesim2CallbackVel);
    // Publisher to publish the relative distance between the two turtles
    ros::Publisher turtles_distance_pub = n.advertise<std_msgs::Float32>("/turtles/rel_distance", 100);
    // Publishers to publish the speed of the two turtles
    ros::Publisher turtle1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Publisher turtle2_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1000);

    ros::Rate loop_rate(1000);

    geometry_msgs::Twist turtle1_vel, turtle2_vel;
    ros::Time start;
    std_msgs::Float32 distance;
    while (ros::ok()) {
        // Calculate the relative distance between the two turtles and publish it on the topic
        distance.data = sqrt(pow(x[0] - x[1], 2) + pow(y[0] - y[1], 2));
        turtles_distance_pub.publish(distance);

        // First check: verify that the relative distance is not less than or equal to the threshold, otherwise stop the turtle that is moving
        if (distance.data <= THR) {
            // Find the turtle that is moving and stop it by publishing its speed as zero
            int which_turtle = is_moving_t1 ? 1 : 2;
            ROS_INFO("Stop turtle %d because it's too close to the other turtle [distance: %f]: %f, %f, %f - %f, %f, %f", which_turtle, distance.data, x[0], y[0], z[0], x[1], y[1], z[1]);
            // Stop the moving turtle by setting the velocity to zero
            stop_turtle(is_moving_t1 ? turtle1_pub : turtle2_pub, is_moving_t1 ? turtle1_vel : turtle2_vel);
    
            // Set the speeds in reverse to make it go back for 0.1 seconds from where it came
            reverse_velocity(which_turtle - 1, which_turtle == 1 ? turtle1_pub : turtle2_pub, which_turtle == 1 ? turtle1_vel : turtle2_vel);
            ros::spinOnce();
            ros::Duration(0.1).sleep();

            // Stop the turtle's backward movement by publishing its speed as zero
            stop_turtle(which_turtle == 1 ? turtle1_pub : turtle2_pub, which_turtle == 1 ? turtle1_vel : turtle2_vel);
        }
        if (x[0] > MAX || x[0] < MIN || y[0] > MAX || y[0] < MIN) {
            // Stop the turtle 1 by publishing its speed as zero
            ROS_INFO("Stop turtle 1 because it's too close to the edge: %f, %f, %f", x[0], y[0], z[0]);
            // Stop the moving turtle by setting the velocity to zero
            stop_turtle(turtle1_pub, turtle1_vel);
            
            // Set the speeds in reverse to make it go back for 0.1 seconds from where it came
            reverse_velocity(0, turtle1_pub, turtle1_vel);
            ros::spinOnce();
            ros::Duration(0.1).sleep();

            // Stop the turtle's backward movement by publishing its speed as zero
            stop_turtle(turtle1_pub, turtle1_vel);
        } 
        if (x[1] > MAX || x[1] < MIN || y[1] > MAX || y[1] < MIN) {
            // Stop the turtle 2 by publishing its speed as zero
            ROS_INFO("Stop turtle 2 because it's too close to the edge: %f, %f, %f", x[1], y[1], z[1]);
            // Stop the moving turtle by setting the velocity to zero
            stop_turtle(turtle2_pub, turtle2_vel);

            // Set the speeds in reverse to make it go back for 0.1 seconds from where it came
            reverse_velocity(1, turtle2_pub, turtle2_vel);
            ros::spinOnce();
            ros::Duration(0.1).sleep();

            // Stop the turtle's backward movement by publishing its speed as zero
            stop_turtle(turtle2_pub, turtle2_vel);
        }
        for (int i = 0; i < ANGLES; i++) {
            if (obs_distance[i] < THR_OBS) {
                ROS_INFO("Stop turtle1 because it's too close to the obstacle %d: %f", i, obs_distance[i]);
                // Stop the moving turtle by setting the velocity to zero
                stop_turtle(turtle1_pub, turtle1_vel);
                // Set the speeds in reverse to make it go back for 0.1 seconds from where it came
                reverse_velocity(1, turtle1_pub, turtle1_vel);
                ros::spinOnce();
                ros::Duration(0.1).sleep();
                // Stop the turtle's backward movement by publishing its speed as zero
                stop_turtle(turtle1_pub, turtle1_vel);

                break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}