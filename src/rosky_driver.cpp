#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros_final/driving_control.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
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
        tf2::Quaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << std::endl;
        this->orientation = yaw * 180 / M_PI;
        if (this->orientation < 0) {
            this->orientation += 360;
        }
        // cout << "orientation: " << this->orientation << std::endl;
    }

    double distance(const Coord& other) {
        double distance_x = this->x - other.x;
        double distance_y = this->y - other.y;
        return sqrt(pow(distance_x, 2) + pow(distance_y, 2));
    }

    double angleRight(const Coord& other) {
        double angle = other.orientation - this->orientation;
        cout << "angle --> " << this->orientation << "  , other --> " << other.orientation << endl;
        // cout << "angle --> " << angle << endl;
        return angle >= 0 ? angle : 360 + angle;
    }

    double angle(const Coord& other) {
        double angle = this->orientation - other.orientation;
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

    string mode;

    double velocity, distance, angle;

   public:
    Driver() {
        sub = node_handle->subscribe<nav_msgs::Odometry>("odom_rf2o", 1, &Driver::callback, this);
        pub = node_handle->advertise<geometry_msgs::Twist>("cmd_vel", 1);
        mode = "relative";
    }

    void forward(const Coord& current) {
        geometry_msgs::Twist twist;
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
        if (mode == 'absolute') {
            int max = angle >= 355 ? angle - 355 : angle + 5;
            int min = angle <= 5 ? angle + 355 : angle - 5;
            if (max > min) {
                if (current.orientation > min && current.orientation < max) {
                    coord_first.init = false;
                    current_state = ProcessState::IDLE;
                } else {
                    twist.angular.z = velocity;
                }
            } else {
                if (current.orientation > min || current.orientation < max) {
                    coord_first.init = false;
                    current_state = ProcessState::IDLE;
                } else {
                    twist.angular.z = velocity;
                }
            }
        } else {
            if (coord_first.angleRight(current) < angle) {
                twist.angular.z = velocity;
            } else {
                coord_first.init = false;
                current_state = ProcessState::IDLE;
            }
        }

        pub.publish(twist);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& odom) {
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
        driver.mode = request.mode;
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