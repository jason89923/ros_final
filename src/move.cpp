#include <ros/ros.h>

#include "ros_final/driving_control.h"

class Rosky_control {
   private:
    ros::NodeHandle node_handle;
    ros::ServiceClient client;

   public:
    void linear_move(float distance, float velocity) {
        ros_final::driving_control srv;
        srv.request.distance_meter = distance;
        srv.request.velocity = velocity;
        client.call(srv);
    }

    void angular_move(int angle, float velocity) {
        ros_final::driving_control srv;
        srv.request.angle_degree = angle;
        srv.request.velocity = velocity;
        client.call(srv);
    }

    Rosky_control() {
        client = node_handle.serviceClient<ros_final::driving_control>("rosky_control");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosky_control_node");
    Rosky_control controller;
    int i = 0;
    while (i < 10) {
        controller.linear_move(1.0, 1.0);
        controller.angular_move(165, 1);
        controller.linear_move(1.0, 1.0);
        controller.angular_move(165, 1);
        i++;
    }  // while

    return 0;
}  // main
