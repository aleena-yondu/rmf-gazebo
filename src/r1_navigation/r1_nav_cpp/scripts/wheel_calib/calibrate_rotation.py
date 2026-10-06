#!/usr/bin/env python3
"""
Odometry Rotation Calibration Tool

This script helps you calibrate the yaw_scale_factor by comparing
commanded rotation vs. actual rotation reported by odometry.

Usage:
  1. Start your robot with localization running
  2. Run this script: python3 calibrate_rotation.py
  3. The robot will rotate in place
  4. Script will calculate the recommended yaw_scale_factor
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class RotationCalibrator(Node):
    def __init__(self):
        super().__init__('rotation_calibrator')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Use raw synchro odometry, not EKF-fused /odom
        self.odom_sub = self.create_subscription(
            Odometry, '/synchro_odom', self.odom_callback, 10)
        
        self.last_yaw = None
        self.total_rotation = 0.0  # Accumulated rotation in radians
        
    def odom_callback(self, msg):
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if self.last_yaw is not None:
            # Calculate delta, handling wraparound
            delta = current_yaw - self.last_yaw
            
            # Detect and correct wraparound
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            
            # Accumulate total rotation
            self.total_rotation += delta
        
        self.last_yaw = current_yaw
    
    def get_rotation(self):
        """Get total rotation since start in degrees"""
        return math.degrees(self.total_rotation)
    
    def rotate(self, angular_vel=0.3, duration=10.0):
        """Rotate the robot and measure actual rotation"""
        self.get_logger().info('Starting rotation...')
        self.get_logger().info(f'Angular velocity: {angular_vel} rad/s')
        self.get_logger().info(f'Duration: {duration}s')
        
        # Reset measurement
        self.last_yaw = None
        self.total_rotation = 0.0
        time.sleep(0.5)
        
        # Rotate
        twist = Twist()
        twist.angular.z = angular_vel
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)
        
        # Stop
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        
        time.sleep(1.0)
        
        # Report
        actual_rotation = self.get_rotation()
        expected_rotation = math.degrees(angular_vel * duration)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('CALIBRATION RESULTS:')
        self.get_logger().info(f'  Expected rotation: {expected_rotation:.1f}°')
        self.get_logger().info(f'  Actual rotation:   {actual_rotation:.1f}°')
        self.get_logger().info(f'  Error:             {expected_rotation - actual_rotation:.1f}°')
        
        if abs(actual_rotation) > 5:
            scale_factor = expected_rotation / actual_rotation
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'RECOMMENDED yaw_scale_factor: {scale_factor:.3f}')
            self.get_logger().info('=' * 60)
            self.get_logger().info('Apply this by launching with:')
            self.get_logger().info(f'  ros2 launch r1_nav_cpp localization.launch.py yaw_scale_factor:={scale_factor:.3f}')
        else:
            self.get_logger().error('Rotation too small! Is odometry working?')

def main():
    rclpy.init()
    calibrator = RotationCalibrator()
    
    print("\n" + "=" * 60)
    print("ODOMETRY ROTATION CALIBRATION")
    print("=" * 60)
    print("\nThis will rotate the robot in place to measure odometry accuracy.")
    print("Reading from: /synchro_odom (raw wheel odometry)")
    print("\nMake sure:")
    print("  1. Robot has clear space around it")
    print("  2. synchro_drive_odometry node is running")
    print("  3. You're ready to observe the rotation")
    print("\nPress ENTER to start...")
    input()
    
    # Do a full rotation
    calibrator.rotate(angular_vel=0.4, duration=15.0)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
