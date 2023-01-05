#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <iostream>

using namespace std;

class Coord {
   public:
    double x, y;
    Coord(double x, double y) {
        this->x = x;
        this->y = y;
    }

    Coord(const nav_msgs::Odometry& odom) {
        this->x = odom.pose.pose.position.x;
        this->y = odom.pose.pose.position.y;
    }

    double distance(const Coord& other) {
        double distance_x = this->x - other.x;
        double distance_y = this->y - other.y;
        return sqrt(pow(distance_x, 2) + pow(distance_y, 2));
    }
};

class Driver {
   private:
    ros::NodeHandle node_handle;
    ros::Subscriber sub;
    ros::Publisher pub;

    double velocity, distance;

   public:
    Driver() {
        sub = node_handle.subscribe<nav_msgs::Odometry>("odom_rf2o", 1, &Driver::scanCallback, this);
        pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        node_handle.param<double>("velocity", velocity, 0.2);
        node_handle.param<double>("distance", distance, 1);
    }

    void scanCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        static nav_msgs::Odometry first = *odom;
        Coord coord_first = first, coord_current = *odom;
        float d = coord_first.distance(coord_current);
        cout << d << endl;
        if (d < distance) {
            geometry_msgs::Twist twist;
            twist.linear.x = velocity;
            pub.publish(twist);
        } else {
            geometry_msgs::Twist twist;
            twist.linear.x = 0;
            pub.publish(twist);
            exit(0);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosky_driver_node");
    Driver driver;

    ros::spin();

    return 0;
}