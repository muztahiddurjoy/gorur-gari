#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FrontDistanceDetector(Node):
    zones= list(range(12))
    # 0 and 11 index for front distance
    # 1 and 2 index for left and right distance

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
        self.zones = [
            self.handle_zone_distance(msg.ranges, 660, 780),
            self.handle_zone_distance(msg.ranges, 600, 660),
            self.handle_zone_distance(msg.ranges, 540, 600),
            self.handle_zone_distance(msg.ranges, 480, 540),
            self.handle_zone_distance(msg.ranges, 420, 480),
            self.handle_zone_distance(msg.ranges, 360, 420),
            self.handle_zone_distance(msg.ranges, 300, 360),
            self.handle_zone_distance(msg.ranges, 240, 300),
            self.handle_zone_distance(msg.ranges, 180, 240),
            self.handle_zone_distance(msg.ranges, 120, 180),
            self.handle_zone_distance(msg.ranges, 60, 120),
            self.handle_zone_distance(msg.ranges, 0, 60)]
        closest_angle_diff = float('inf')
        
        current_angle = msg.angle_min
        # first_distance = msg.ranges[720-60] if msg.ranges else None
        self.zones[0] = self.handle_zone_distance(msg.ranges, 0, 60)
        self.zones[1] = self.handle_zone_distance(msg.ranges, 60, 120)
        self.zones[2] = self.handle_zone_distance(msg.ranges, 120, 180)
        self.zones[3] = self.handle_zone_distance(msg.ranges, 180, 240)
        self.zones[4] = self.handle_zone_distance(msg.ranges, 240, 300)
        self.zones[5] = self.handle_zone_distance(msg.ranges, 300, 360)
        self.zones[6] = self.handle_zone_distance(msg.ranges, 360, 420)
        self.zones[7] = self.handle_zone_distance(msg.ranges, 420, 480)
        self.zones[8] = self.handle_zone_distance(msg.ranges, 480, 540)
        self.zones[9] = self.handle_zone_distance(msg.ranges, 540, 600)
        self.zones[10] = self.handle_zone_distance(msg.ranges, 600, 660)
        self.zones[11] = self.handle_zone_distance(msg.ranges, 660, 720)
        
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
    def handle_zone_distance(self,ranges,start,end):
        avg_distance = 0.0
        valid_count = 0
        for i in range(start, end):
            if i > 719:
                i -= 720
            if ranges[i] not in (float('inf'), float('nan')):
                avg_distance += ranges[i]
                valid_count += 1
        avg_distance /= float(valid_count)
        return avg_distance if valid_count > 0 else float('inf')
    
    # def print_distances(self):
    #     max_dis_index = 0
    #     for i in range(len(self.zones)):
    #         if self.zones[i] > self.zones[max_dis_index]:
    #             max_dis_index = i
    #     if(self.zones[0]<0.4):
    #         if(self.zones[1]>)
    #     else:
    #         print("Go forward")
    #     self.get_logger().info(f"Move to {((max_dis_index+1)*30)-30} degree with index {max_dis_index} and distance {self.zones[max_dis_index]:.2f}m")
    
    def linear_movement_handler(self):
        max_dis_index = 0
        

        

    
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