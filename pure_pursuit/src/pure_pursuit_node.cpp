#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:
    double x, y, v; // position [x,y] and velocity at point, v
    double theta;   // curvature

    
    std::string cur_wpt_topic_ = "/waypoint";
    std::string drive_topic_ = "/drive";
    std::string vis_topic_ = "/vis";

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr waypoint_sub_;
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Lookahead distance for Pure Pursuit";
        this->declare_parameter("L", 1.0, param_desc);  
        this->declare_parameter("Kp", 0.7); 
   
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            cur_wpt_topic_, 1, std::bind(&PurePursuit::waypoint_callback, this, std::placeholders::_1));
        
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic_, 1);

        vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            vis_topic_, 1);

     
    }

 

    
    
    void waypoint_callback(const geometry_msgs::msg::Vector3::ConstPtr &waypoint)
    {
        // TODO: find the current waypoint to track using methods mentioned in lecture
        x = waypoint->x;
        y = waypoint->y;
        v = waypoint->z;
        
        // TODO: transform goal point to vehicle frame of reference



        // TODO: calculate curvature/steering angle
        theta = 2 * abs(y)/pow(this->get_parameter("L").get_parameter_value().get<float>(), 2);
        
        // TODO: publish drive message, don't forget to limit the steering angle.
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = this->get_parameter("Kp").get_parameter_value().get<float>() * theta;
        drive_msg.drive.speed = v;
        drive_pub_->publish(drive_msg);
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