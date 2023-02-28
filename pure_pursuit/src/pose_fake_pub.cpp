#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp" // need this
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

class PosePubSub : public rclcpp::Node
{

private:
    std::string wpt_topic_ = "/waypoint";
    std::string odom_topic_ = "ego_racecar/odom";

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::vector<std::array<double, 2>> waypoints; // read this in once (2D points)

public:
    // default contructor
    PosePubSub() : Node("pose_fake_pub_node") {

        // auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        // param_desc.description = "Lookahead distance for Pure Pursuit";
        // this->declare_parameter("L", 1.0, param_desc);  

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            wpt_topic_, 1);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 1, std::bind(&PosePubSub::odom_callback, this, std::placeholders::_1));

    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        /// TODO: publish pose
        geometry_msgs::msg::PoseStamped p;
        p.pose = odom_msg->pose.pose;

        pose_pub_->publish(p);
    }

    ~PosePubSub() {}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePubSub>());
    rclcpp::shutdown();
    return 0;
}

