#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
import serial
import threading
import time

class ESP32CarBridge(Node):

    def __init__(self):
        super().__init__('esp32_car_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('encoder_topic', '/encoder_data')
        self.declare_parameter('button_topic', '/button_state')
        self.declare_parameter('publish_rate', 10.0)  # Hz - Rate to send commands to ESP32
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publishers (for data from ESP32)
        self.encoder_pub = self.create_publisher(Float32, self.get_parameter('encoder_topic').value, 10)
        self.button_pub = self.create_publisher(Int32, self.get_parameter('button_topic').value, 10)
        
        # Subscriber (for control commands)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_vel_callback,
            10)
        
        # Last received command and timestamp
        self.last_twist_msg = Twist()
        self.last_twist_time = self.get_clock().now()
        self.has_received_twist = False
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Serial communication setup
        try:
            self.serial_connection = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=1.0  # seconds
            )
            self.get_logger().info(f"Successfully connected to serial port {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {serial_port}: {str(e)}")
            rclpy.shutdown()
            return
        
        # Start threads
        self.serial_read_thread = threading.Thread(target=self.read_serial)
        self.serial_read_thread.daemon = True
        self.serial_read_thread.start()
        
        self.command_send_thread = threading.Thread(target=self.send_commands_continuously)
        self.command_send_thread.daemon = True
        self.command_send_thread.start()
        
        self.get_logger().info(f"ESP32 Car Bridge node has started. Command rate: {self.publish_rate} Hz")
    
    def cmd_vel_callback(self, msg):
        # Update the last received command with thread safety
        with self.lock:
            self.last_twist_msg = msg
            self.last_twist_time = self.get_clock().now()
            self.has_received_twist = True
        
        self.get_logger().debug(f"Received Twist: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")
    
    def send_commands_continuously(self):
        """Continuously send the last received command at a fixed rate"""
        rate = 1.0 / self.publish_rate  # Convert Hz to seconds
        
        while rclpy.ok():
            try:
                current_time = self.get_clock().now()
                
                with self.lock:
                    # Check if we have a recent command (within last 2 seconds)
                    time_since_last_cmd = (current_time - self.last_twist_time).nanoseconds / 1e9
                    
                    if self.has_received_twist and time_since_last_cmd < 2.0:
                        # Use linear.x for velocity and angular.z for steering (servo angle)
                        velocity = self.last_twist_msg.linear.x
                        angle = self.last_twist_msg.angular.z
                        
                        # Format the command string
                        command_string = f"{velocity},{angle}\n"
                        
                        # Send command
                        self.serial_connection.write(command_string.encode('utf-8'))
                        self.get_logger().debug(f"Sent to ESP32: {command_string.strip()}")
                    else:
                        # Send stop command if no recent commands
                        if self.has_received_twist and time_since_last_cmd >= 2.0:
                            self.get_logger().warn("No recent Twist commands, sending stop")
                            command_string = "0.0,0.0\n"
                            self.serial_connection.write(command_string.encode('utf-8'))
                            self.has_received_twist = False  # Stop sending until new command
                
                # Sleep to maintain rate
                time.sleep(rate)
                
            except Exception as e:
                self.get_logger().error(f"Error in command send thread: {str(e)}")
                time.sleep(0.1)  # Prevent tight loop on error
    
    def read_serial(self):
        buffer = ""
        while rclpy.ok():
            try:
                if self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.read(self.serial_connection.in_waiting).decode('utf-8')
                    buffer += data
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self.get_logger().info(f"Received from ESP32: {line}")  # Print all serial messages
                            self.process_serial_line(line)
            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {str(e)}")
                break
    
    def process_serial_line(self, line):
        # Example line from your ESP32 code: "encoder_count,encoder_velocity,button_state"
        # And a separate log line: "Servo angle: X"
        if line.startswith('Servo angle:'):
            # Extract and log servo angle
            try:
                angle_str = line.split(':')[1].strip()
                servo_angle = float(angle_str)
                self.get_logger().info(f"Servo angle: {servo_angle} degrees")
            except ValueError as e:
                self.get_logger().warn(f"Could not parse servo angle: {line}")
        else:
            try:
                parts = line.split(',')
                if len(parts) >= 3:
                    encoder_count = int(parts[0])
                    encoder_velocity = float(parts[1])
                    button_state = int(parts[2])
                    
                    # Print the received data
                    self.get_logger().info(f"ESP32 Data - Count: {encoder_count}, Velocity: {encoder_velocity:.2f}, Button: {button_state}")
                    
                    # Publish encoder velocity
                    encoder_msg = Float32()
                    encoder_msg.data = encoder_velocity
                    self.encoder_pub.publish(encoder_msg)
                    
                    # Publish button state
                    button_msg = Int32()
                    button_msg.data = button_state
                    self.button_pub.publish(button_msg)
                    
            except ValueError as e:
                self.get_logger().warn(f"Could not parse serial line: {line}. Error: {e}")
            except IndexError as e:
                self.get_logger().warn(f"Index error parsing line: {line}. Error: {e}")
    
    def destroy_node(self):
        self.get_logger().info("Shutting down ESP32 Car Bridge node...")
        # Send one final stop command
        try:
            if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
                self.serial_connection.write("0.0,0.0\n".encode('utf-8'))
                self.serial_connection.close()
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {str(e)}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32CarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by KeyboardInterrupt")
    except Exception as e:
        node.get_logger().error(f"Node stopped by exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()