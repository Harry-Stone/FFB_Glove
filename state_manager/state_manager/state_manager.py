import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from threading import Lock
from ament_index_python.packages import get_package_share_directory

config_package_name = 'shared_config' 
config_file_name = 'glove_settings.json'
share_directory = get_package_share_directory(config_package_name)
config_file_path = os.path.join(share_directory, 'config', config_file_name)

try:
    with open(config_file_path, 'r') as f:
        dynamixelSettings = json.load(f)
except FileNotFoundError:
    print(f"Error: config.json not found at {config_file_path}")
    exit()


class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        
        # Thread safety for data access
        self.lock = Lock()
        
        # Data storage for both topics
        self.serial_data = None
        self.dynamixel_positions = None
        
        # Create subscriptions
        self.serial_sub = self.create_subscription(
            String,
            'serial/data',
            self.serial_data_callback,
            10
        )
        
        self.dynamixel_pos_sub = self.create_subscription(
            Float32MultiArray,
            'dynamixel_pos',
            self.dynamixel_pos_callback,
            10
        )
        
        self.get_logger().info('StateManager Node initialized')
    
    def serial_data_callback(self, msg):
        """Callback for serial/data topic"""
        with self.lock:
            self.serial_data = msg.data
            self.get_logger().debug(f'Received serial data: {self.serial_data}')
    
    def dynamixel_pos_callback(self, msg):
        """Callback for dynamixel_pos topic"""
        with self.lock:
            self.dynamixel_positions = list(msg.data)
            self.get_logger().debug(f'Received dynamixel positions: {self.dynamixel_positions}')
    
    def read_sensor_data(self):
        """
        Read both serial/data and dynamixel_pos topics.
        
        Returns:
            tuple: (serial_data, dynamixel_positions)
                - serial_data (str): Latest serial data from serial/data topic
                - dynamixel_positions (list): Latest motor positions [T1, T2, F1, F2]
        """
        with self.lock:
            return self.serial_data, self.dynamixel_positions

