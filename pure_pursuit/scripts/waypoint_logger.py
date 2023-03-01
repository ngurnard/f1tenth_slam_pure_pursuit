#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion  # sudo apt install ros-foxy-tf-transformations 
import time


relative_path = "/sim_ws/src/pure_pursuit/pure_pursuit/waypoints/"
fname = "waypoints1.csv"
# file = open(strftime(relative_path+'waypoint-%Y-%m-%d-%H-%M-%S', gmtime())+'.csv', 'w')
file = open(relative_path+'waypoint_drive.csv', 'w')


previous_time = 0.0

class WaypointLogger(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.save_waypoint,
            10)
        self.subscription  # prevent unused variable warning


    def save_waypoint(self,  data):
        global previous_time
        if(data.header.stamp.sec - previous_time >= 2):
            quaternion = np.array([data.pose.pose.orientation.x, 
                                data.pose.pose.orientation.y, 
                                data.pose.pose.orientation.z, 
                                data.pose.pose.orientation.w])

            euler = euler_from_quaternion(quaternion)
            speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                    data.twist.twist.linear.y, 
                                    data.twist.twist.linear.z]),2)

            file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                            data.pose.pose.position.y,
                                            euler[2],
                                            speed))
            print("Every 2sec", previous_time)

            previous_time = data.header.stamp.sec
 
def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    rclpy.spin(waypoint_logger)
    file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    print('Saving waypoints...')
    main()