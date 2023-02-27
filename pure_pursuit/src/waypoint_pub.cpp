#include <sstream>
#include <string>
#include <cmath>
#include <vector>

using namespace std;

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp" // need this
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "interfaces_hot_wheels/msg.Waypoint.hpp"

class Waypoint : public rclcpp::Node
{

private:
    std::string wpt_topic_ = "/waypoint";
    std::string vis_topic_ = "/vis";
    std::string pose_topic_ = "/pose";

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
    rclcpp::Publisher<interfaces_hot_wheels::msg::Waypoint>::SharedPtr wpt_pub_;

    std::vector<interfaces_hot_wheels::msg::Waypoint> waypoints;

public:
    // default contructor
    Waypoint() : Node("waypoint_node") {

        // auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        // param_desc.description = "Lookahead distance for Pure Pursuit";
        // this->declare_parameter("L", 1.0, param_desc);  

        wpt_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            wpt_topic_, 1);

        vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            vis_topic_, 1);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 1, std::bind(&Waypoint::pose_callback, this, std::placeholders::_1));

        // populate the waypoints vector
        csv_to_waypoints();
    }

    void csv_to_waypoints()
    {
        string fname;
        cout<<"Enter the file name: ";
        cin>>fname;
    
        vector<vector<string>> content;
        vector<string> row;
        string line, word;
    
        fstream file (fname, ios::in);
        if(file.is_open())
        {
            while(getline(file, line))
            {
                row.clear();
    
                stringstream str(line);
    
                while(getline(str, word, ','))
                    row.push_back(word);
                content.push_back(row);
            }
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose)
    {
        // TODO: find current location
        x = pose->x;
        y = pose->y;
        v = pose->z;
        
        // TODO: publish waypoint message
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = this->get_parameter("Kp").get_parameter_value().get<float>() * theta;
        drive_msg.drive.speed = this->get_parameter("V").get_parameter_value().get<float>();
        drive_pub_->publish(drive_msg);

        // TODO: publish vis message
    }

    ~Waypoint() {}
};

