import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from threading import Lock
from ament_index_python.packages import get_package_share_directory
import numpy as np

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


class ChainPoint:
    """Represents a point in a kinematic chain with rotation variables"""
    
    def __init__(self, name, length_from_parent=0, parent=None):
        """
        Args:
            name: Name of the point (e.g., 'l1', 'l2')
            length_from_parent: Distance from parent point
            parent: Parent ChainPoint in the chain
        """
        self.name = name
        self.length_from_parent = length_from_parent
        self.parent = parent
        
        # Position relative to parent
        self.position = np.array([0.0, 0.0, 0.0])
        
        # Rotation angles (in degrees) around local axes
        self.a1 = 0.0  # Rotation around X axis
        self.a2 = 0.0  # Rotation around Y axis
        self.a3 = 0.0  # Rotation around Z axis
        
        # Rotation axes configuration (can be 'x', 'y', or 'z')
        self.rotation_axes = ['z', 'y', 'x']  # Default rotation order
        
        # World position (calculated)
        self.world_position = np.array([0.0, 0.0, 0.0])
    
    def set_rotation_axes(self, axes):
        """
        Set the order and types of rotation axes
        Args:
            axes: List of 3 axes (e.g., ['z', 'y', 'x'])
        """
        if len(axes) == 3 and all(ax in ['x', 'y', 'z'] for ax in axes):
            self.rotation_axes = axes
        else:
            raise ValueError("Axes must be a list of 3 elements from ['x', 'y', 'z']")
    
    def calculate_world_position(self):
        """Calculate world position based on rotations and parent position"""
        # Start with local offset along first axis (before rotation)
        local_pos = np.array([self.length_from_parent, 0.0, 0.0])
        
        # Apply rotations around local axes
        rotation_matrix = self._get_rotation_matrix()
        rotated_pos = rotation_matrix @ local_pos
        
        # Add parent's world position
        if self.parent is not None:
            self.world_position = self.parent.world_position + rotated_pos
        else:
            self.world_position = rotated_pos
        
        return self.world_position
    
    def _get_rotation_matrix(self):
        """Get combined rotation matrix from rotation angles"""
        # Convert degrees to radians
        a1_rad = np.radians(self.a1)
        a2_rad = np.radians(self.a2)
        a3_rad = np.radians(self.a3)
        
        # Create rotation matrices for each axis
        rx = np.array([
            [1, 0, 0],
            [0, np.cos(a1_rad), -np.sin(a1_rad)],
            [0, np.sin(a1_rad), np.cos(a1_rad)]
        ])
        
        ry = np.array([
            [np.cos(a2_rad), 0, np.sin(a2_rad)],
            [0, 1, 0],
            [-np.sin(a2_rad), 0, np.cos(a2_rad)]
        ])
        
        rz = np.array([
            [np.cos(a3_rad), -np.sin(a3_rad), 0],
            [np.sin(a3_rad), np.cos(a3_rad), 0],
            [0, 0, 1]
        ])
        
        # Combine rotations in the order specified by rotation_axes
        rotation_matrix = np.eye(3)
        for axis in self.rotation_axes:
            if axis == 'x':
                rotation_matrix = rotation_matrix @ rx
            elif axis == 'y':
                rotation_matrix = rotation_matrix @ ry
            elif axis == 'z':
                rotation_matrix = rotation_matrix @ rz
        
        return rotation_matrix
    
    def get_state_dict(self):
        """Return current state as dictionary"""
        return {
            'name': self.name,
            'world_position': self.world_position.tolist(),
            'angles': {'a1': self.a1, 'a2': self.a2, 'a3': self.a3},
            'rotation_axes': self.rotation_axes
        }


class Finger:
    """Represents a finger or thumb with a kinematic chain"""
    
    def __init__(self, name, config_dict):
        """
        Args:
            name: Name of finger/thumb (e.g., 'thumb', 'finger1')
            config_dict: Configuration dictionary from JSON
        """
        self.name = name
        self.config = config_dict
        
        # Extract origin
        origin = config_dict['links']['origin']
        self.origin = np.array([origin['x'], origin['y'], origin['z']], dtype=float)
        
        # Create chain points
        self.chain = []
        self._create_chain()
    
    def _create_chain(self):
        """Create the kinematic chain from config"""
        parent = None
        
        # Create root point at origin
        root = ChainPoint('origin', 0, None)
        root.world_position = self.origin.copy()
        self.chain.append(root)
        parent = root
        
        # Create chain points for each link
        link_index = 1
        while f'l{link_index}' in self.config['links']:
            length = self.config['links'][f'l{link_index}']
            point = ChainPoint(f'l{link_index}', length, parent)
            self.chain.append(point)
            parent = point
            link_index += 1
    
    def update_positions(self):
        """Recalculate all world positions based on current rotation angles"""
        for point in self.chain:
            if point.parent is not None:
                point.calculate_world_position()
    
    def set_rotation(self, link_name, a1=None, a2=None, a3=None):
        """
        Set rotation angles for a specific link
        Args:
            link_name: Name of the link (e.g., 'l1', 'l2')
            a1, a2, a3: Rotation angles in degrees
        """
        for point in self.chain:
            if point.name == link_name:
                if a1 is not None:
                    point.a1 = a1
                if a2 is not None:
                    point.a2 = a2
                if a3 is not None:
                    point.a3 = a3
                self.update_positions()
                return
        raise ValueError(f"Link '{link_name}' not found in {self.name}")
    
    def get_chain_state(self):
        """Return the state of all points in the chain"""
        return {
            'name': self.name,
            'origin': self.origin.tolist(),
            'chain': [point.get_state_dict() for point in self.chain]
        }


# Create finger and thumb objects from config
fingers = {}
fingers['thumb'] = Finger('thumb', dynamixelSettings['thumb'])
fingers['finger1'] = Finger('finger1', dynamixelSettings['finger1'])
fingers['finger2'] = Finger('finger2', dynamixelSettings['finger2'])


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

