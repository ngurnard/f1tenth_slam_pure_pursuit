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

#define named(blockname) goto blockname; \
                         blockname##_skip: if (0) \
                         blockname:

#define break(blockname) goto blockname##_skip


class Waypoint : public rclcpp::Node
{

private:
    std::string wpt_topic_ = "/waypoint";
    std::string vis_waypoint_topic_ = "/waypoint_vis";
    std::string vis_cur_point_topic_ = "/cur_point_vis";
    std::string pose_topic_ = "/pf/viz/inferred_pose";
    // std::string pose_topic_ = "/pf/odom/pose";
    std::string lidarscan_topic = "/scan";

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_waypoint_pub_;
    rclcpp::Publisher<interfaces_hot_wheels::msg::Waypoint>::SharedPtr wpt_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_cur_point_pub_;

    float theta_min, theta_increment;
    float theta;
    int number_of_rays;

    std::vector<interfaces_hot_wheels::msg::Waypoint> lane1;
    std::vector<interfaces_hot_wheels::msg::Waypoint> lane2;


    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string global_frame_;
    std::string local_frame_;

    visualization_msgs::msg::MarkerArray marker_array; 
    
    /*
    LANES:
    1: optimized raceline assuring minimum time (TUMUNICH optimizer)
    2: manual, seemingly good raceline
    */
    int LANE_NUMBER = 1; // 1, 2 or 3// 1, 2 or 3

    /*
    REGIONS:
    1: 1 < 2 < 3 (Optimal on the left)
    2: 3 < 2 < 1 (Optimal on the right)
    */
    int REGION = 1; // 1 or 2

    
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
    {
        // TODO: Find optimum waypoint
        // TODO: transform goal point to vehicle frame of reference
    
        geometry_msgs::msg::TransformStamped map_to_baselink;
        std::string SrcFrameRel = global_frame_.c_str();
        std::string TarFrameRel = local_frame_.c_str();

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

        // Find region
        // if( pose_msg->pose.position.x > 2.0 && pose_msg->pose.position.x < 7.5 &&
        //     pose_msg->pose.position.y > 2.0 && pose_msg->pose.position.y < 9.0)
        //     REGION = 2;
        // else
        //     REGION = 1;

        // RCLCPP_INFO(this->get_logger(), "Lane number: %d and Region: %d", LANE_NUMBER, REGION);

        double min_dist = MAXFLOAT;
        interfaces_hot_wheels::msg::Waypoint next_point;
        std::vector<interfaces_hot_wheels::msg::Waypoint> points_to_track;

        // choose points to track
        // different lanes for overtake maneuver
        if(LANE_NUMBER == 1) {
            RCLCPP_INFO(this->get_logger(), "should now be tracking lane 1");
            points_to_track = lane1;
        } else {//if(LANE_NUMBER == 2) 
            RCLCPP_INFO(this->get_logger(), "should now be tracking lane 2");
            points_to_track = lane2;
        }
        // else
        // {
        //     // points_to_track = waypoints_lane1;
        //     throw("Invalid lane number - LINE 130");
        // }

        // track chosen points
        for (auto wpt : points_to_track) {
            
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
                next_point.l = sqrt(pow(next_point.x, 2) + pow(next_point.y, 2));
                if(!this->get_parameter("v_csv").get_parameter_value().get<int>())
                    next_point.v = this->get_parameter("v").get_parameter_value().get<float>();
                else
                    next_point.v = wpt.v;

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
        marker.header.frame_id = local_frame_;

        vis_cur_point_pub_->publish(marker);
        wpt_pub_->publish(next_point);
        // vis_waypoint_pub_->publish(marker_array);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        
        theta_min = scan_msg->angle_min;
        theta_increment = scan_msg->angle_increment;
        number_of_rays = scan_msg->ranges.size();

        const float *range_data = scan_msg->ranges.data();

        // add some logic to detect obstacles and update LANE_NUMBER global variable

        // LANE_NUMBER = 1 should be default
        // LANE_NUMBER = 2 for overtake maneuver
        // LANE_NUMBER = 3 for wide overtake maneuver

        // LiDAR ray indices corresponding to angle in degrees
        int idx_0   = 540;
        int idx_n15 = 479;
        int idx_p15 = 600;
        int idx_n35 = 399;
        int idx_p35 = 690;
        int idx_n60 = 299;
        int idx_p60 = 780;

        // TODO: Add logic to detect obstacles and update LANE_NUMBER global variable
        bool flag_obstacle = false;
        named (outer)
        for(int r=idx_n15; r<idx_p15; r++)
        {
            // Check if there is an obstacle in front of the car
            if(range_data[r] < this->get_parameter("straight_opp_dist").get_parameter_value().get<float>())
            {
                if (LANE_NUMBER == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "obstacle detected. switching lane to 2");
                    LANE_NUMBER = 2;
                    break(outer);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "obstacle detected. switching lane to 1");
                    LANE_NUMBER = 1;
                    break(outer);
                }
            }
        }

    }

    void csv_to_waypoints()
    {
        cout << this->get_parameter("waypoints_path").as_string() << endl;
        string relative_path = this->get_parameter("waypoints_path").as_string(); //"/sim_ws/src/pure_pursuit/pure_pursuit/waypoints/";
        // string fname = this->get_parameter("waypoints_file").as_string(); // "waypoints1.csv";
        
        std::string line, s;
        // std::ifstream file(relative_path + fname);
        // file.open("waypoints_drive.csv");
        string fname = "waypoints_recollect_1.csv";
        std::ifstream file1(relative_path + fname);
 
        visualization_msgs::msg::Marker marker;

        std::vector<double> line_vector;
        int marker_id = 1;
        
        if(!file1.is_open())
        {
            throw "waypoints/waypoints_lane1.csv file failed to open, check relative path";
        }
        else
        {   
            while(getline(file1, line))
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
               
                

                lane1.push_back(p);        

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
                marker.header.frame_id = global_frame_;

                marker_array.markers.push_back(marker);


                line_vector.clear();
            }
            file1.close();

        }
        
        fname = "waypoints_recollect_2.csv";
        std::ifstream file2(relative_path + fname);

        if(!file2.is_open())
        {
            throw "waypoints/waypoints_lane2.csv file failed to open, check relative path";
        }
        else
        {
            while(getline(file2, line))
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
                

                lane2.push_back(p);        

                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.pose.position.x = p.x;
                marker.pose.position.y = p.y;
                marker.id = marker_id++;
                marker.scale.x = 0.15;
                marker.scale.y = 0.15;
                marker.scale.z = 0.15;
                marker.color.a = 0.5;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.header.frame_id = global_frame_;

                marker_array.markers.push_back(marker);


                line_vector.clear();
            }
            file2.close();
        }

        vis_waypoint_pub_->publish(marker_array);

    }

public:
    // default contructor
    Waypoint() : Node("waypoint_laneswitcher_node") {
        // cout << "Current path" << ament_index_cpp::ameget_package_share_directory("pure_pursuit") << endl;
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Lookahead distance for Pure Pursuit";
        this->declare_parameter("L", 1.0, param_desc);
        this->declare_parameter("v", 2.0);
        param_desc.description = "Integer flag to use velocities from .csv";
        this->declare_parameter("v_csv", 0, param_desc);
        this->declare_parameter("L_csv", 1, param_desc);



        global_frame_ = this->declare_parameter<std::string>("gloabal_frame", "map");
        local_frame_ = this->declare_parameter<std::string>("local_frame", "laser");

        this->declare_parameter("waypoints_path");
        // this->declare_parameter("waypoints path", "/sim_ws/src/pure_pursuit/pure_pursuit/waypoints");
        // this->declare_parameter("waypoints_file", "waypoints_raceline_1.csv");   

        param_desc.description = "Distance in front of car to check for opponent";
        this->declare_parameter("straight_opp_dist", 2.0, param_desc);
        param_desc.description = "Distance on the side of car to check for opponent";
        this->declare_parameter("side_opp_dist", 0.5, param_desc);


        wpt_pub_ = this->create_publisher<interfaces_hot_wheels::msg::Waypoint>(
            wpt_topic_, 1);

        vis_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            vis_waypoint_topic_, 1);
        
        vis_cur_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            vis_cur_point_topic_, 1);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 1, std::bind(&Waypoint::pose_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 1, std::bind(&Waypoint::lidar_callback, this, std::placeholders::_1));

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
