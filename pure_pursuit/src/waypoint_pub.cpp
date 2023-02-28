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
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp" // need this
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "interfaces_hot_wheels/msg/waypoint.hpp"

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

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string source_frame_;
    std::string target_frame_;
    

public:
    // default contructor
    Waypoint() : Node("waypoint_node") {

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Lookahead distance for Pure Pursuit";
        this->declare_parameter("L", 1.0, param_desc);  
        source_frame_ = this->declare_parameter<std::string>("source_frame", "/map");
        target_frame_ = this->declare_parameter<std::string>("target_frame", "/base_link");
        
        // populate the waypoints vector
        csv_to_waypoints();

        wpt_pub_ = this->create_publisher<interfaces_hot_wheels::msg::Waypoint>(
            wpt_topic_, 1);

        vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            vis_topic_, 1);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 1, std::bind(&Waypoint::pose_callback, this, std::placeholders::_1));


        // // populate the waypoints vector
        // csv_to_waypoints();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void csv_to_waypoints()
    {
        std::string fname = "waypoints/levine.csv";
        
        std::string line;
        fstream file (fname, ios::in);

        visualization_msgs::msg::MarkerArray marker_array;  
        visualization_msgs::msg::Marker marker;
        
        if(file.is_open())
        {
            int marker_id = 1;
            while(getline(file, line))
            {
                std::vector<std::string> vec;
                boost::algorithm::split(vec, line, boost::is_any_of(",")); // split str with delimiter
                interfaces_hot_wheels::msg::Waypoint p;

                p.x = std::stod(vec[0]); // str to double
                p.y = std::stod(vec[1]);
                p.v = std::stod(vec[2]);

                waypoints.push_back(p);        

                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.pose.position.x = p.x;
                marker.pose.position.y = p.y;
                marker.id = marker_id++;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker_array.markers.push_back(marker);
            }
        }
        vis_pub_->publish(marker_array);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    {
        float x = pose_msg->pose.position.x;
        float y = pose_msg->pose.position.y;
        geometry_msgs::msg::Quaternion quat = pose_msg->pose.orientation;

        // TODO: Find optimum waypoint

        // TODO: transform goal point to vehicle frame of reference
        geometry_msgs::msg::TransformStamped map_to_baselink;
        std::string SrcFrameRel = source_frame_.c_str();
        std::string TarFrameRel = target_frame_.c_str();

        try {
          map_to_baselink = tf_buffer_->lookupTransform(
            TarFrameRel, SrcFrameRel, // world waypoints in terms of cars frame. From world to car = ^{car}T_{world}
            tf2::TimePointZero); // get the latest available transform
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            TarFrameRel.c_str(), SrcFrameRel.c_str(), ex.what());
          return;
        }

        // draw it out to see
        // Pmap is given as x or y
        // T is the tranform ^{car}T_{world}
        // Pcar = Pmap + T

        // for each undefined waypoints

        double min_dist = std::numeric_limits<double>::max();
        interfaces_hot_wheels::msg::Waypoint next_point;
        
        for (auto wpt : waypoints) {
            
            geometry_msgs::msg::Pose wpt_transformed;
            wpt_transformed.position.x = wpt.x;
            wpt_transformed.position.y = wpt.y;
            wpt_transformed.position.z = 0;
            wpt_transformed.orientation.x = 0;
            wpt_transformed.orientation.y = 0;
            wpt_transformed.orientation.z = 0;
            wpt_transformed.orientation.w = 1;
            tf2::doTransform(wpt_transformed, wpt_transformed, map_to_baselink);

            if(wpt_transformed.x < 0.0){
                continue;
            }

            double dist = sqrt(pow(wpt_transformed.x, 2) + pow(wpt_transformed.y, 2));
            if(dist < min_dist && dist > this->get_parameter("L").get_parameter_value().get<float>()){
                min_dist = dist;
                next_point.x = wpt_transformed.x;
                next_point.y = wpt_transformed.y;
            }
        }

        next_point.v = 2.0;
        next_point.l = this->get_parameter("L").get_parameter_value().get<float>();
        wpt_pub_->publish(next_point);
        
    }
    
    ~Waypoint() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Waypoint>());
    rclcpp::shutdown();
    return 0;
}
