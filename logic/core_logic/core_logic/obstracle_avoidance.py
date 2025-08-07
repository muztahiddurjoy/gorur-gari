#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FrontDistanceDetector(Node):
    def __init__(self):
        super().__init__('front_distance_detector')
        
        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('angle_tolerance', 5.0)  # degrees
        
        # Get parameters
        scan_topic = self.get_parameter('scan_topic').value
        self.angle_tolerance = math.radians(
            self.get_parameter('angle_tolerance').value)
        
        # Subscribe to LaserScan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        self.get_logger().info(f"Listening to {scan_topic} for front distance (0°)")
    
    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages.
        Finds the distance measurement closest to 0 degrees (front).
        """
        front_distance = None
        closest_angle_diff = float('inf')
        
        current_angle = msg.angle_min
        
        for i, distance in enumerate(msg.ranges):
            # Calculate absolute angle difference from 0 degrees
            angle_diff = abs(self.normalize_angle(current_angle))
            
            # Check if this measurement is closer to 0 than previous ones
            if (angle_diff < self.angle_tolerance and 
                angle_diff < closest_angle_diff and
                msg.range_min <= distance <= msg.range_max):
                
                closest_angle_diff = angle_diff
                front_distance = distance
            
            # Increment angle for next measurement
            current_angle += msg.angle_increment
        
        if front_distance is not None:
            self.get_logger().info(
                f"Front distance: {front_distance:.2f}m (angle diff: {math.degrees(closest_angle_diff):.1f}°)",
                throttle_duration_sec=1)  # Throttle to 1Hz to avoid spamming
        else:
            self.get_logger().warn(
                "No valid front distance measurement found within tolerance",
                throttle_duration_sec=1)
    
    def normalize_angle(self, angle):
        """
        Normalize angle to [-π, π] range.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = FrontDistanceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()