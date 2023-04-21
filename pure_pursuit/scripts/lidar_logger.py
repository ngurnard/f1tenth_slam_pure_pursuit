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
fname = "lidar_final1"
print(relative_path+fname+'.csv')
# file = open(strftime(relative_path+'waypoint-%Y-%m-%d-%H-%M-%S', gmtime())+'.csv', 'w')
file = open(relative_path+fname+'.csv', 'w')


previous_time = 0.0

class LidarLogger(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        # self.subscription = self.create_subscription(
        #     PoseStamped,
        #     'pf/viz/inferred_pose',
        #     self.save_waypoint,
        #     10)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.save_lidar,
            10)
        self.lidar_subscription  # prevent unused variable warning


    def save_lidar(self, data):
        global previous_time
        collection_time = 0.5
        if(data.header.stamp.sec - previous_time >= collection_time):
            print("next lidar", data.header.stamp.sec)
            left_idx = int(((-math.pi/2.0) - data.angle_min) / data.angle_increment)
            right_idx = int(((math.pi/2.0) - data.angle_min) / data.angle_increment)
            left = data.ranges[left_idx]
            right = data.ranges[right_idx]
            file.write('%f,%f,%f\n'%( right,
                                            left,
                                            data.header.stamp.sec,
                                            ))

            previous_time = data.header.stamp.sec
 
def main(args=None):
    rclpy.init(args=args)
    lidar_logger = LidarLogger()
    rclpy.spin(lidar_logger)
    file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    print('Saving lidar...')
    main()