#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <unistd.h>
#include <iostream>
#include <string>

// Definition of the maximum limits
const float LINEAR_THR = 5.0;   // Maximum linear velocity
const float ANGULAR_THR = 10.0; // Maximum angular velocity

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "turtlesim_listener1");
    ros::NodeHandle n;
    // Service to spawn the turtle at a specific position
    ros::ServiceClient turtle_client_spawn = n.serviceClient<turtlesim::Spawn>("/spawn");
    turtle_client_spawn.waitForExistence();
    // Set the position where to spawn the new turtle
    turtlesim::Spawn turtle_spawn;
    turtle_spawn.request.x = 2.0;
    turtle_spawn.request.y = 1.0;
    turtle_spawn.request.theta = 0.0;
    turtle_spawn.request.name = "turtle2";
    // Check that the spawn call is executed correctly
    if (turtle_client_spawn.call(turtle_spawn)) {
        ROS_INFO("Spawned turtle 2 successfully at position (x: [%f], y: [%f], z: [%f])", turtle_spawn.request.x, turtle_spawn.request.y, turtle_spawn.request.theta);
    } else {
        ROS_ERROR("Failed to spawn turtle2");
        return 1;
    }
    // Publishers to publish the speed of the two turtles
    ros::Publisher turtle1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    ros::Publisher turtle2_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);

    int turtle;
    float x_vel, y_vel, z_vel;
    geometry_msgs::Twist turtle_vel;
    while (ros::ok()) {
        // Let the user choose which turtle to move
        do {
            std::cout << "Insert the turtle you want to control [1-2]: ";
            std::cin >> turtle;
        } while (turtle != 1 && turtle != 2);

        // Ask the user to set the turtle's speed
        std::cout << "Insert the linear velocity x (range: -5 to +5): ";
        std::cin >> x_vel;
        x_vel = min(x_vel, LINEAR_THR);
        x_vel = max(x_vel, -LINEAR_THR);
        std::cout << "Insert the linear velocity y (range: -5 to +5): ";
        std::cin >> y_vel;
        y_vel = min(y_vel, LINEAR_THR);
        y_vel = max(y_vel, -LINEAR_THR);
        std::cout << "Insert the angular velocity z (range: -10 to +10): ";
        std::cin >> z_vel;
        z_vel = std::min(z_vel, ANGULAR_THR);
        z_vel = std::max(z_vel, -ANGULAR_THR);

        turtle_vel.linear.x = x_vel;
        turtle_vel.linear.y = y_vel;
        turtle_vel.angular.z = z_vel;

        turtle == 1 ? turtle1_pub.publish(turtle_vel) : turtle2_pub.publish(turtle_vel);
        ros::spinOnce();
        // Move the turtle for one second
        ros::Duration(1.0).sleep();
    }
    return 0;
}

