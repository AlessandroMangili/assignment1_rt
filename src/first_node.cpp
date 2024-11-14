#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <unistd.h>
#include <iostream>
#include <string>

int main(int argc, char **argv){
    ros::init(argc, argv, "turtlesim_listener1");
    ros::NodeHandle n;

    ros::ServiceClient turtle_client_spawn = n.serviceClient<turtlesim::Spawn>("/spawn");
    turtle_client_spawn.waitForExistence();

    turtlesim::Spawn turtle_spawn;
    turtle_spawn.request.x = 2.0;
    turtle_spawn.request.y = 1.0;
    turtle_spawn.request.theta = 0.0;
    turtle_spawn.request.name = "turtle2";

    if (turtle_client_spawn.call(turtle_spawn)) {
        ROS_INFO("Spawned turtle2 successfully at position (x: [%f], y: [%f], z: [%f])", turtle_spawn.request.x, turtle_spawn.request.y, turtle_spawn.request.theta);
    } else {
        ROS_ERROR("Failed to spawn turtle2.");
        return 1;
    }

    int turtle;
     while (ros::ok()) {
        // Choosing which turtle
        do {
            std::cout << "Insert the turtle you want to control [1-2]: ";
            std::cin >> turtle;
        } while (turtle != 1 && turtle != 2);

        // Choosing the linear velocity of the turtle
        float x_vel, y_vel, z_vel;
        std::cout << "Insert the linear velocity x: ";
        std::cin >> x_vel;
        std::cout << "Insert the linear velocity y: ";
        std::cin >> y_vel;
        std::cout << "Insert the angular velocity z: ";
        std::cin >> z_vel;

        ros::Publisher turtle_pub = n.advertise<geometry_msgs::Twist>("/turtle" + std::to_string(turtle) + "/cmd_vel", 10);
        ros::Rate rate(10);

        geometry_msgs::Twist my_vel;
        my_vel.linear.x = x_vel;
        my_vel.linear.y = y_vel;
        my_vel.angular.z = z_vel;

        int count = 0;
        while (count < 10 && ros::ok()) {
            turtle_pub.publish(my_vel);
            rate.sleep();
            count++;
        }
        ros::spinOnce();
    }
    return 0;
}

