import serial
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
import threading    
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Replace with your serial port
    baudrate=115200
)

class SerialControlNode(Node):
    def __init__(self):
        super().__init__('serial_control_node')
        
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
        
        # Timer for sending commands at a fixed rate
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def cmd_vel_callback(self, msg):
        with self.lock:
    