#!/usr/bin/env python3
'''
@author Jaysh Khan
'''

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class WallFollowerNode:  
    def __init__(self):
        rospy.init_node('wall_follower_node')

        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_callback, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.obstacle_threshold = 0.55 
        self.move_speed = 0.2
        self.rotation_speed = 0.45  # rad/s
        
        self.pid_controller = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.previous_time = rospy.Time.now() # rospy.Time for ROS1

    def laser_callback(self, msg):
        cmd = Twist()
        ranges = np.array(msg.ranges) 
        ranges = ranges - 0.180

        current_time = rospy.Time.now()
        dt = (current_time - self.previous_time).to_sec() # Get time in seconds
        self.previous_time = current_time

        if all(ranges[0:20] > self.obstacle_threshold) and all(ranges[-20:] > self.obstacle_threshold):
            cmd.linear.x = self.move_speed
            cmd.angular.z = 0.0
        else:
            front_distance = min(min(ranges[0:20]), min(ranges[-20:]))
            error = self.obstacle_threshold - front_distance

            angular_z = self.pid_controller.compute(0, error, dt)

            cmd.linear.x = self.move_speed / 2.0
            cmd.angular.z = angular_z

        self.cmd_vel_pub.publish(cmd)

if __name__ == '__main__':
    wall_follower_node = WallFollowerNode() 
    rospy.spin() 
