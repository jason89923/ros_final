#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros_final/driving_control.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>
#include <unistd.h>


#include <iostream>

using namespace std;

ros::NodeHandle* node_handle;

class Coord {
   public:
    double x, y;
    int orientation;
    bool init;

    Coord() {
        init = false;
    }

    Coord(const nav_msgs::Odometry& odom) {
        init = true;
        this->x = odom.pose.pose.position.x;
        this->y = odom.pose.pose.position.y;
        tf::
        tf::TransformListener listener_;
        this->orientation = int(4/cos(odom.pose.pose.orientation.w)*(180/M_PI));  //odom.pose.pose.orientation.z*(3.14159/180)+0.5
    }

    double distance(const Coord& other) {
        double distance_x = this->x - other.x;
        double distance_y = this->y - other.y;
        return sqrt(pow(distance_x, 2) + pow(distance_y, 2));
    }

    double angleRight(const Coord& other) {
        double angle = this->orientation - other.orientation;
        //cout << "angle --> " << angle << endl;
        return angle >= 0 ? angle : 360 + angle;
    }

    double angle(const Coord& other) {
        double angle = other.orientation - this->orientation;
        return angle > 0 ? angle : -angle;
    }
};

enum class ProcessState {
    IDLE,
    FORWARDING,
    ROTATING
};

class Driver {
   private:
    ros::Subscriber sub;
    ros::Publisher pub;
    ProcessState current_state;

    Coord coord_first;

    double velocity, distance, angle;

   public:
    Driver() {
        sub = node_handle->subscribe<nav_msgs::Odometry>("odom_rf2o", 1, &Driver::callback, this);
        pub = node_handle->advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    void forward(const Coord& current) {
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        if (coord_first.distance(current) < distance) {
            twist.linear.x = velocity;
        } else {
            coord_first.init = false;
            current_state = ProcessState::IDLE;
        }

        pub.publish(twist);
    }

    void rotate(const Coord& current) {
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        if (coord_first.angleRight(current) < 180) {
            twist.angular.z = velocity;
        } else {
            coord_first.init = false;
            current_state = ProcessState::IDLE;
        }

        pub.publish(twist);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& odom) {
        odom->twist.twist.
        Coord current = *odom;
        if (current_state == ProcessState::IDLE) {
            return;
        }

        if (!coord_first.init) {
            coord_first = current;
        }

        if (current_state == ProcessState::FORWARDING) {
            forward(current);
        } else {
            rotate(current);
        }
    }

    friend class Controller;
};

class Controller {
   private:
    ros::NodeHandle node_handle;
    ros::ServiceServer service;
    Driver driver;

   public:
    Controller() {
        service = node_handle.advertiseService("rosky_control", &Controller::callback, this);
    }

    bool callback(ros_final::driving_control::Request& request, ros_final::driving_control::Response& response) {
        cout << "start" << endl;
        cout << request.velocity << endl;
        driver.velocity = request.velocity;
        driver.distance = request.distance_meter;
        driver.angle = request.angle_degree;
        if (request.angle_degree != 0) {
            driver.current_state = ProcessState::ROTATING;
        } else {
            driver.current_state = ProcessState::FORWARDING;
        }

        while (driver.current_state != ProcessState::IDLE) {
            usleep(200 * 1000);
        }

        cout << "end" << endl;
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosky_driver_node");
    node_handle = new ros::NodeHandle;
    Controller controller;

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}