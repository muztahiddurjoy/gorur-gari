#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class LidarZoneAnalyzer(Node):
    def __init__(self):
        super().__init__('lidar_zone_analyzer')
        
        # Parameters
        self.num_zones = 12
        self.max_valid_range = 30.0  # Maximum valid range in meters
        self.min_valid_range = 0.1   # Minimum valid range in meters
        
        # Pre-allocate arrays for efficiency
        self.zone_ranges = np.zeros(self.num_zones, dtype=np.float32)
        self.zone_counts = np.zeros(self.num_zones, dtype=np.int32)
        
        # Publishers and Subscribers
        self.zone_pub = self.create_publisher(Float32MultiArray, '/lidar_zones', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        
        self.get_logger().info("Lidar Zone Analyzer initialized with 12 zones")

    def scan_callback(self, msg):
        """Process LiDAR scan and calculate zone averages"""
        try:
            # Reset arrays for new calculation
            self.zone_ranges.fill(0.0)
            self.zone_counts.fill(0)
            
            # Convert ranges to numpy array for efficient processing
            ranges = np.array(msg.ranges, dtype=np.float32)
            angle_increment = msg.angle_increment
            angle_min = msg.angle_min
            
            # Pre-calculate angles for each measurement
            num_points = len(ranges)
            angles = angle_min + np.arange(num_points) * angle_increment
            angles = np.mod(angles, 2 * np.pi)  # Normalize angles to [0, 2π)
            
            # Calculate zone for each point efficiently
            zone_indices = ((angles / (2 * np.pi)) * self.num_zones).astype(np.int32)
            zone_indices = np.clip(zone_indices, 0, self.num_zones - 1)
            
            # Filter valid ranges (handle inf, nan, and out-of-range values)
            valid_mask = (
                (ranges >= self.min_valid_range) & 
                (ranges <= self.max_valid_range) & 
                np.isfinite(ranges)
            )
            
            valid_ranges = ranges[valid_mask]
            valid_zones = zone_indices[valid_mask]
            
            # Calculate sums and counts for each zone using numpy operations
            if len(valid_ranges) > 0:
                for zone in range(self.num_zones):
                    zone_mask = (valid_zones == zone)
                    if np.any(zone_mask):
                        zone_ranges = valid_ranges[zone_mask]
                        self.zone_ranges[zone] = np.mean(zone_ranges)
                        self.zone_counts[zone] = len(zone_ranges)
                    else:
                        self.zone_ranges[zone] = 0.0  # No valid data
                        self.zone_counts[zone] = 0
            
            # Publish results
            self.publish_zone_averages()
            
        except Exception as e:
            self.get_logger().error(f"Error processing scan: {str(e)}")

    def publish_zone_averages(self):
        """Publish zone averages as Float32MultiArray"""
        zone_msg = Float32MultiArray()
        zone_msg.data = self.zone_ranges.tolist()
        
        # Add zone counts as additional data for debugging/validation
        # zone_msg.layout.dim.append(MultiArrayDimension())
        # zone_msg.layout.dim[0].label = "zone_averages"
        # zone_msg.layout.dim[0].size = self.num_zones
        # zone_msg.layout.dim[0].stride = 1
        
        self.zone_pub.publish(zone_msg)
        
        # Optional: Log zone information (comment out for maximum performance)
        # self.log_zone_info()

    def log_zone_info(self):
        """Log zone information for debugging"""
        zone_info = " | ".join([f"Z{i}:{dist:.2f}({cnt})" 
                               for i, (dist, cnt) in enumerate(zip(self.zone_ranges, self.zone_counts))])
        self.get_logger().info(f"Zone averages: {zone_info}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LidarZoneAnalyzer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()