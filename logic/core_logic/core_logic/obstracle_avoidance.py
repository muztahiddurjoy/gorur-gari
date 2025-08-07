#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FrontDistanceDetector(Node):
    front_distance = None
    front_left_distance = None
    front_right_distance = None
    back_distance = None
    back_left_distance = None
    back_right_distance = None

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
        # first_distance = msg.ranges[720-60] if msg.ranges else None
        self.handle_front(msg.ranges)
        self.handle_front_left(msg.ranges)
        self.handle_front_right(msg.ranges)
        self.handle_back(msg.ranges)
        self.handle_back_left(msg.ranges)
        self.handle_back_right(msg.ranges)
        self.print_distances()
        # if first_distance is not None:
        #     if(first_distance < 0.2):
        #         self.get_logger().warn("Stop car")
        #     else:
        #         self.get_logger().info("Keep driving")
        # self.get_logger().warn(f"Angle distance {msg.angle_increment*180/math.pi} degrees")
        
        for i, distance in enumerate(msg.ranges):
            # Calculate absolute angle difference from 0 degrees
            angle_diff = abs(self.normalize_angle(current_angle))
            # Check if this measurement is closer to 0 than previous ones
            if (angle_diff < self.angle_tolerance and 
                angle_diff < closest_angle_diff and
                msg.range_min <= distance <= msg.range_max):
                closest_angle_diff = angle_diff
            
            # Increment angle for next measurement
            current_angle += msg.angle_increment
        
        # if front_distance is not None:
        #     self.get_logger().info(
        #         f"Front distance: {front_distance:.2f}m (angle diff: {math.degrees(closest_angle_diff):.1f}°)")  # Throttle to 1Hz to avoid spamming
        # else:
        #     self.get_logger().warn(
        #         "No valid front distance measurement found within tolerance")
    
    def normalize_angle(self, angle):
        """
        Normalize angle to [-π, π] range.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    #Handle for all regions
    def handle_front(self,ranges):
        avg_distance = 0.0
        valid_count = 0
        for i in range(660,780):
            if i > 719:
                i -= 720
            if ranges[i] not in (float('inf'), float('nan')):
                avg_distance += ranges[i]
                valid_count += 1
        avg_distance /= float(valid_count)
        self.front_distance = avg_distance
    
    def handle_front_left(self, ranges):
        avg_distance = 0.0
        valid_count = 0
        for i in range(60, 180):
            if ranges[i] not in (float('inf'), float('nan')):
                avg_distance += ranges[i]
                valid_count += 1
        if valid_count > 0:
            avg_distance /= float(valid_count)
            self.front_left_distance = avg_distance

    def handle_front_right(self, ranges):
        avg_distance = 0.0
        valid_count = 0
        for i in range(540, 660):
            if ranges[i] not in (float('inf'), float('nan')):
                avg_distance += ranges[i]
                valid_count += 1
        if valid_count > 0:
            avg_distance /= float(valid_count)
            self.front_right_distance = avg_distance
    
    def handle_back_left(self, ranges):
        avg_distance = 0.0
        valid_count = 0
        for i in range(180, 300):
            if ranges[i] not in (float('inf'), float('nan')):
                avg_distance += ranges[i]
                valid_count += 1
        if valid_count > 0:
            avg_distance /= float(valid_count)
            self.back_left_distance = avg_distance
    
    def handle_back(self, ranges):
        avg_distance = 0.0
        valid_count = 0
        for i in range(300, 420):
            if ranges[i] not in (float('inf'), float('nan')):
                avg_distance += ranges[i]
                valid_count += 1
        if valid_count > 0:
            avg_distance /= float(valid_count)
            self.back_right_distance = avg_distance
    
    def handle_back_right(self, ranges):
        avg_distance = 0.0
        valid_count = 0
        for i in range(420, 540):
            if ranges[i] not in (float('inf'), float('nan')):
                avg_distance += ranges[i]
                valid_count += 1
        if valid_count > 0:
            avg_distance /= float(valid_count)
            self.back_distance = avg_distance
    
    def print_distances(self):
        self.get_logger().info(
            f"Front: {self.front_distance:.2f}m, "
            f"Front Left: {self.front_left_distance:.2f}m, "
            f"Front Right: {self.front_right_distance:.2f}m, "
            f"Back: {self.back_distance:.2f}m, "
            f"Back Left: {self.back_left_distance:.2f}m, "
            f"Back Right: {self.back_right_distance:.2f}m"
        )

    
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