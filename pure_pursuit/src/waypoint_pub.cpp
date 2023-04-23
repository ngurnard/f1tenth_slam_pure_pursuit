#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <sstream>
// #include <boost/filesystem/path.hpp>

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "ament_index_cpp/visibility_control.h"

#include "interfaces_hot_wheels/msg/waypoint.hpp"

class Waypoint : public rclcpp::Node
{

private:
    std::string wpt_topic_ = "/waypoint";
    std::string vis_waypoint_topic_ = "/waypoint_vis";
    std::string vis_cur_point_topic_ = "/cur_point_vis";
    std::string pose_topic_ = "/pf/viz/inferred_pose";

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_waypoint_pub_;
    rclcpp::Publisher<interfaces_hot_wheels::msg::Waypoint>::SharedPtr wpt_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_cur_point_pub_;

    rclcpp::TimerBase::SharedPtr timer_;


    std::vector<interfaces_hot_wheels::msg::Waypoint> waypoints;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string source_frame_;
    std::string target_frame_;

    visualization_msgs::msg::MarkerArray marker_array; 

    void timer_callback()
    {
      vis_waypoint_pub_->publish(marker_array);
    }
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
    {
        // TODO: Find optimum waypoint
        // TODO: transform goal point to vehicle frame of reference
    
        geometry_msgs::msg::TransformStamped map_to_baselink;
        std::string SrcFrameRel = source_frame_.c_str();
        std::string TarFrameRel = target_frame_.c_str();

        // cout << "Pose Callback" << endl;

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

        double min_dist = MAXFLOAT;
        interfaces_hot_wheels::msg::Waypoint next_point;
        
        for (auto wpt : waypoints) {
            
            geometry_msgs::msg::PoseStamped wpt_transformed;
            wpt_transformed.header.stamp = pose_msg->header.stamp;
            wpt_transformed.pose.position.x = wpt.x;
            wpt_transformed.pose.position.y = wpt.y;
            wpt_transformed.pose.position.z = 0;
            wpt_transformed.pose.orientation.x = 0;
            wpt_transformed.pose.orientation.y = 0;
            wpt_transformed.pose.orientation.z = 0;
            wpt_transformed.pose.orientation.w = 1;
            tf2::doTransform(wpt_transformed, wpt_transformed, map_to_baselink);

            // skip if not in front of the vehicle
            if(wpt_transformed.pose.position.x < 0.0){
                continue;
            }

            double dist = sqrt(pow(wpt_transformed.pose.position.x, 2)
                            + pow(wpt_transformed.pose.position.y, 2)); // l2 norm

            double lookahead;
            if(!this->get_parameter("L_csv").get_parameter_value().get<int>())
                    lookahead = this->get_parameter("L").get_parameter_value().get<float>();
                else
                    lookahead = wpt.l;
            // RCLCPP_INFO(this->get_logger(), "L: %f", lookahead);
            if(dist < min_dist && dist > lookahead){
                min_dist = dist;
                next_point.x = wpt_transformed.pose.position.x;
                next_point.y = wpt_transformed.pose.position.y;
                next_point.kp = wpt.kp;
                next_point.l = sqrt(pow(next_point.x, 2) + pow(next_point.y, 2));
                if(!this->get_parameter("v_csv").get_parameter_value().get<int>())
                    next_point.v = this->get_parameter("v").get_parameter_value().get<float>();
                else
                    next_point.v = wpt.v;;
            }
        }

        // RCLCPP_INFO(this->get_logger(), "Next point: %f, %f", next_point.x, next_point.y);;

        visualization_msgs::msg::Marker marker;

        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.pose.position.x = next_point.x;
        marker.pose.position.y = next_point.y;
        marker.id = 0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.header.frame_id = target_frame_;

        vis_cur_point_pub_->publish(marker);
        wpt_pub_->publish(next_point);
        // vis_waypoint_pub_->publish(marker_array);
    }


    void csv_to_waypoints()
    {
        cout << this->get_parameter("waypoints_path").as_string() << endl;
        string relative_path = this->get_parameter("waypoints_path").as_string(); //"/sim_ws/src/pure_pursuit/pure_pursuit/waypoints/";
        string fname = this->get_parameter("waypoints_file").as_string(); // "waypoints1.csv";
        
        std::string line, s;
        std::ifstream file(relative_path + fname);
        // file.open("waypoints_drive.csv");
 
        visualization_msgs::msg::Marker marker;

        std::vector<double> line_vector;
        
        if(!file.is_open())
        {
            throw "waypoints/waypoints*.csv file failed to open, check relative path";
        }
        else
        {
            int marker_id = 1;
            while(getline(file, line))
            {
                interfaces_hot_wheels::msg::Waypoint p;

                stringstream ss(line);

                while(getline(ss, s, ',')) 
                {
                    // store token string in the vector
                    line_vector.push_back(stod(s));
                }

                p.x = line_vector[0]; // str to double
                p.y = line_vector[1];
                if(this->get_parameter("v_csv").get_parameter_value().get<int>())
                {
                    p.v = line_vector[3];
                }
                if(this->get_parameter("L_csv").get_parameter_value().get<int>())
                {
                    p.l = line_vector[4];
                }
                p.kp = line_vector[5];

                waypoints.push_back(p);        

                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.pose.position.x = p.x;
                marker.pose.position.y = p.y;
                marker.id = marker_id++;
                marker.scale.x = 0.15;
                marker.scale.y = 0.15;
                marker.scale.z = 0.15;
                marker.color.a = 0.5;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.header.frame_id = source_frame_;

                marker_array.markers.push_back(marker);


                line_vector.clear();
            }
            file.close();
        }
        // vis_waypoint_pub_->publish(marker_array);

    }

public:
    // default contructor
    Waypoint() : Node("waypoint_node") {
        // cout << "Current path" << ament_index_cpp::ameget_package_share_directory("pure_pursuit") << endl;
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Lookahead distance for Pure Pursuit";
        this->declare_parameter("L", 1.0, param_desc);
        this->declare_parameter("v", 2.0);
        param_desc.description = "Integer flag to use velocities from .csv";
        this->declare_parameter("v_csv", 0, param_desc);
        this->declare_parameter("L_csv", 1, param_desc);

        source_frame_ = this->declare_parameter<string>("global_frame", "map");
        target_frame_ = this->declare_parameter<string>("local_frame", "ego_racecar/laser_model");

        this->declare_parameter("waypoints_path");
        // this->declare_parameter("waypoints path", "/sim_ws/src/pure_pursuit/pure_pursuit/waypoints");
        this->declare_parameter("waypoints_file", "waypoints1.csv");    


        wpt_pub_ = this->create_publisher<interfaces_hot_wheels::msg::Waypoint>(
            wpt_topic_, 1);


        vis_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            vis_waypoint_topic_, 1);
        
        vis_cur_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            vis_cur_point_topic_, 1);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 1, std::bind(&Waypoint::pose_callback, this, std::placeholders::_1));


        timer_ = this->create_wall_timer(
        500ms, std::bind(&Waypoint::timer_callback, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // populate the waypoints vector
        csv_to_waypoints();
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
