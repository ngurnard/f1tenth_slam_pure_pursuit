#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include "interfaces_hot_wheels/msg/waypoint.hpp"
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:
    void waypoint_callback(const interfaces_hot_wheels::msg::Waypoint::ConstSharedPtr waypoint)
    {
        // TODO: find the current waypoint to track using methods mentioned in lecture
        x = waypoint->x;
        y = waypoint->y;
        v = waypoint->v;
        L = waypoint->l;
        Kp = waypoint->kp;
        // RCLCPP_INFO(this->get_logger(), "Waypoint: %f, %f, %f, %f, %f", x, y, v, L, Kp);

        // TODO: calculate curvature/steering angle
        theta = 2 * (y)/pow(L, 2);
        
        // TODO: publish drive message, don't forget to limit the steering angle.
        // kp is proportional to arlength since s=r*theta --> theta is proportional to Kp/r
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = Kp * theta;
        drive_msg.drive.speed = v;
        drive_pub_->publish(drive_msg);
        
        // RCLCPP_INFO(this->get_logger(), "Steering Angle: %f", drive_msg.drive.steering_angle);
        // RCLCPP_INFO(this->get_logger(), "Next point in car frame: %f, %f", x, y);
        // // visualization
        // visualization_msgs::msg::Marker marker;

        // marker.type = visualization_msgs::msg::Marker::SPHERE;
        // marker.pose.position.x = x;
        // marker.pose.position.y = y;
        // marker.id = 0;
        // marker.scale.x = 0.1;
        // marker.scale.y = 0.1;
        // marker.scale.z = 0.1;
        // marker.color.a = 1.0;
        // marker.color.r = 0.0;
        // marker.color.g = 1.0;
        // marker.color.b = 0.0;
        // vis_pub_->publish(marker);
    }


    double x, y, v, L, Kp; // position [x,y] and velocity at point, v, adaptive lookahead distance, L, and proportional gain, Kp
    double theta;   // curvature

    std::string cur_wpt_topic_ = "/waypoint";
    std::string drive_topic_ = "/drive";


    rclcpp::Subscription<interfaces_hot_wheels::msg::Waypoint>::SharedPtr waypoint_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    


public:
    PurePursuit() :
        Node("pure_pursuit_node"),
        x(0.0),
        y(0.0),
        v(0.0),
        L(1.0),
        theta(0.0)
    {
        // TODO: create ROS subscribers and publishers
        waypoint_sub_ = this->create_subscription<interfaces_hot_wheels::msg::Waypoint>(
            cur_wpt_topic_, 1, std::bind(&PurePursuit::waypoint_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic_, 1);

        
        
 
        this->declare_parameter("Kp", 0.1);
        this->declare_parameter<std::string>("drive_topic", "/drive");
    }
    

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
