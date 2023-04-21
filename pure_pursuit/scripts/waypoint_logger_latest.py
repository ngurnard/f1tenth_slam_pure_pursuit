#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import math
from tf_transformations import euler_from_quaternion  # sudo apt install ros-foxy-tf-transformations 
import time


relative_path = "/home/nvidia/f1tenth_ws/src/pure_pursuit/pure_pursuit/waypoints/"
# relative_path = "/sim_ws/src/pure_pursuit/pure_pursuit/waypoints/"
fname = "centerline_1"
print(relative_path+fname+'.csv')
# file = open(strftime(relative_path+'waypoint-%Y-%m-%d-%H-%M-%S', gmtime())+'.csv', 'w')
file_wpt = open(relative_path+fname+'_waypoints.csv', 'w')
# file_lidar = open(relative_path+fname+'_lidar.csv', 'w')


previous_time = 0.0

class WaypointLogger(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.wpt_subscription = self.create_subscription(
            PoseStamped,
            'pf/viz/inferred_pose',
            self.save_waypoint,
            10)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.save_lidar,
            10)
        self.wpt_subscription  # prevent unused variable warning
        self.lidar_subscription # prevent unused variable warning


    def save_waypoint(self,  data):
        global previous_time
        collection_time = 0.5
        if(data.header.stamp.sec - previous_time >= collection_time):
            print("next point", data.header.stamp.sec)
            # print('%f,%f,%f,%f,%f\n'%( data.pose.position.x,
            #                                     data.pose.position.y,
            #                                     euler[2],
            #                                     speed,
            #                                     data.header.stamp.sec,
            #                                     ))
            quaternion = np.array([data.pose.orientation.x, 
                                data.pose.orientation.y, 
                                data.pose.orientation.z, 
                                data.pose.orientation.w])

            euler = euler_from_quaternion(quaternion)
            speed = 3.0
            lookahead = 1.0

            file_wpt.write('%f,%f,%f,%f,%f\n'%( data.pose.position.x,
                                                data.pose.position.y,
                                                euler[2],
                                                speed,
                                                lookahead, 
                                                data.header.stamp.sec
                                                ))

            previous_time = data.header.stamp.sec
    
    def save_lidar(self, data):
        global previous_time
        collection_time = 0.5
        if(data.header.stamp.sec - previous_time >= collection_time):
            print("next lidar", data.header.stamp.sec)
            left_idx = int(((-math.pi/2.0) - data.angle_min) / data.angle_increment)
            right_idx = int(((math.pi/2.0) - data.angle_min) / data.angle_increment)
            left = data.ranges[left_idx]
            right = data.ranges[right_idx]
            # file_lidar.write('%f,%f,%f\n'%( right,
            #                                 left,
            #                                 data.header.stamp.sec,
            #                                 ))

            previous_time = data.header.stamp.sec

 
def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    rclpy.spin(waypoint_logger)
    file_wpt.close()
    # file_lidar.close()
    rclpy.shutdown()

if __name__ == '__main__':
    print('Saving waypoints...')
    main()
