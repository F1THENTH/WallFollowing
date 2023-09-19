#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers

        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        self.kp = 0.5
        self.kd = 1.0
        self.ki = 0.0

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        # print(range_data.angle_min, range_data.angle_max)
        angle = angle / 180 * np.pi

        index90 = int((np.pi/2 - range_data.angle_min) / range_data.angle_increment)
        b_range = range_data.ranges[index90]

        index = int(((np.pi/2 - angle) - range_data.angle_min) / range_data.angle_increment)
        a_range = range_data.ranges[index]

        self.alpha = np.arctan2((a_range * np.cos(angle) - b_range) , (a_range * np.sin(angle)))

        D = b_range * np.cos(self.alpha)
        if np.isinf(D) or np.isnan(D):
            return 0.0
        # print(D)
        # exit()
        return D


    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        #TODO:implement
        D_t1 = self.get_range(range_data, 20) - dist * np.sin(self.alpha)

        return -(dist - D_t1) 

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # angle = -1.0
        # TODO: Use kp, ki & kd to implement a PID controller
        angle = self.kp * error + self.kd * (error - self.prev_error) + self.ki * self.integral
        print(angle/np.pi*180)
        # Store the current error for the next iteration
        self.prev_error = error

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = angle
        if np.abs(angle) >= 0 and np.abs(angle) < 10/180 * np.pi:
            drive_msg.drive.speed = 1.5
        elif np.abs(angle) >= 10/180 * np.pi and np.abs(angle) < 20/180 * np.pi:
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        desired_distance = 1
        error = self.get_error(msg, desired_distance) # TODO: replace with error calculated by get_error()
        print(error)
        
        velocity = 0.5 # TODO: calculate desired car velocity based on error
        self.integral += error  
        self.pid_control(error, velocity) # TODO: actuate the car with PID  
        print(drive_msg.drive.steering_angle)
        exit()

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()