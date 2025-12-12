import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import dynamixel_sdk
import json
import os
import time
import signal
import sys
from threading import Lock
import math

from ament_index_python.packages import get_package_share_directory

config_package_name = 'shared_config' 
config_file_name = 'glove_settings.json'
share_directory = get_package_share_directory(config_package_name)
config_file_path = os.path.join(share_directory, 'config', config_file_name)


#script_dir = os.path.dirname(os.path.realpath(__file__))
#config_path = os.path.join(script_dir, "../../config.json")
#config_path = "/glove/src/config.json"

try:
    with open(config_file_path, 'r') as f:
        dynamixelSettings = json.load(f)
except FileNotFoundError:
    print(f"Error: config.json not found at {config_file_path}")
    exit()

class Dynamixel:
    def __init__(self, ID, upperLimit, lowerLimit, zeroPoint, maxCurrentMilliamps, minCurrentMilliamps, forceScaleFactor,mechanicalScaleFactor):
        self.ID=ID
        self.upperLimit=upperLimit
        self.lowerLimit=lowerLimit
        self.zeroPoint=zeroPoint
        self.maxCurrentMilliamps=maxCurrentMilliamps
        self.minCurrentMilliamps=minCurrentMilliamps
        self.forceScaleFactor=forceScaleFactor
        self.mechanicalScaleFactor=mechanicalScaleFactor


def _dynamixel_from_dict(d):
    try:
        return Dynamixel(
            d["ID"],
            d["upperLimit"],
            d["lowerLimit"],
            d["zeroPoint"],
            d["maxCurrentMilliamps"],
            d["minCurrentMilliamps"],
            d["forceScaleFactor"],
            d["mechanicalScaleFactor"],
        )
    except KeyError as e:
        raise KeyError(f"Missing expected key in dynamixel config: {e}")

#load in all dynamixel instances
try: 
    try:
        T1 = _dynamixel_from_dict(dynamixelSettings["thumb"]["d1"])
    except KeyError:
        print("Error: thumb->d1 not found or missing fields in config.json")
        T1 = None
    try:
        T2 = _dynamixel_from_dict(dynamixelSettings["thumb"]["d2"])
    except KeyError:
        print("Error: thumb->d2 not found or missing fields in config.json")
        T2 = None
    try:
        F1 = _dynamixel_from_dict(dynamixelSettings["finger1"]["d1"])
    except KeyError:
        print("Error: finger1->d1 not found or missing fields in config.json")
        F1 = None
    try:
        F2 = _dynamixel_from_dict(dynamixelSettings["finger2"]["d1"])
    except KeyError:
        print("Error: finger2->d1 not found or missing fields in config.json")
        F2 = None
except Exception as e:
    print(f"Unexpected error while initializing Dynamixel instances: {e}")

class DynamixelInterface:
    def __init__(self, port='/dev/ttyUSB0', baudrate=57600, protocolVersion=2.0):
        self.port = port
        self.baudrate = baudrate
        self.protocolVersion = protocolVersion

        self.ADDR_TORQUE_ENABLE  = 64
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_GOAL_CURRENT   = 102
        self.ADDR_PRESENT_POS    = 132
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.portHandler = dynamixel_sdk.PortHandler(self.port)
        self.packetHandler = dynamixel_sdk.PacketHandler(self.protocolVersion)
        self.portHandler.openPort()
        self.portHandler.setBaudRate(self.baudrate)

    def setup_motor(self, dxl : Dynamixel):
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl.ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl.ID, self.ADDR_OPERATING_MODE, 0)  # 0 = current mode
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl.ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

    def set_motor_current(self, dxl : Dynamixel, current_mA): # clamp current to +- max
        if current_mA > dxl.maxCurrentMilliamps:
            current_mA = dxl.maxCurrentMilliamps
        elif current_mA < -dxl.maxCurrentMilliamps:
            current_mA = -dxl.maxCurrentMilliamps
        self.packetHandler.write2ByteTxRx(self.portHandler, dxl.ID, self.ADDR_GOAL_CURRENT, int(current_mA))

    def get_motor_position(self, dxl : Dynamixel):
        pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl.ID, self.ADDR_PRESENT_POS)
        return pos
    
class DynamixelManagerNode(Node):
    def __init__(self):
        super().__init__('dynamixel_manager')
        
        # Initialize interface
        self.interface = DynamixelInterface(
            dynamixelSettings["U2D2"]["port"],
            dynamixelSettings["U2D2"]["baudrate"],
            dynamixelSettings["U2D2"]["protocolVersion"]
        )
        
        # Setup motors
        for motor in [T1, T2, F1, F2]:
            if motor is not None:
                self.interface.setup_motor(motor)
        
        # Create publishers and subscribers
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'dynamixel_pos', 10)
        self.ffb_subscription = self.create_subscription(Float32MultiArray, 'FFB', self.ffb_callback, 10)
        
        # Motor list for easier iteration
        self.motors = [T1, T2, F1, F2]
        
        # Thread safety and timeout handling
        self.lock = Lock()
        self.last_command_time = time.time()
        self.current_commands = [0.0, 0.0, 0.0, 0.0]  # Current for each motor
        self.timeout_threshold = 1.0  # seconds
        
        # Create a timer for the main control loop
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        
        self.get_logger().info('Dynamixel Manager Node initialized')
    
    def shutdown(self):
        """Graceful shutdown: disable torque and close port"""
        self.get_logger().info('Shutting down Dynamixel Manager...')
        try:
            for motor in self.motors:
                if motor is not None:
                    # Disable torque on all motors
                    self.interface.packetHandler.write1ByteTxRx(
                        self.interface.portHandler,
                        motor.ID,
                        self.interface.ADDR_TORQUE_ENABLE,
                        self.interface.TORQUE_DISABLE
                    )
                    self.get_logger().info(f'Torque disabled for motor ID {motor.ID}')
            # Close port
            self.interface.portHandler.closePort()
            self.get_logger().info('Torque disabled. Port closed.')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')
    
    def ffb_callback(self, msg):
        """Handle incoming force feedback (current) commands"""
        with self.lock:
            if len(msg.data) >= 4:
                self.current_commands = list(msg.data[:4])
                self.last_command_time = time.time()
            else:
                self.get_logger().warn(f'FFB message has {len(msg.data)} values, expected 4')
    
    def control_loop(self):
        """Main control loop: read positions, apply currents, publish data"""
        with self.lock:
            # Check for timeout - if no command received for 1 second, zero out currents
            elapsed = time.time() - self.last_command_time
            commands_to_apply = self.current_commands if elapsed < self.timeout_threshold else [0.0, 0.0, 0.0, 0.0]
            
            if elapsed >= self.timeout_threshold:
                self.get_logger().warn(f'Timeout detected (no FFB for {elapsed:.2f}s), disabling motors')
        
        # Read positions and apply currents for each motor
        positions = []
        for i, motor in enumerate(self.motors):
            if motor is not None:
                try:
                    # Get current position
                    pos = self.interface.get_motor_position(motor)
                    
                    # Apply safety limits on position
                    if pos > motor.upperLimit:
                        commands_to_apply[i]=0.0
                    elif pos < motor.lowerLimit:
                        commands_to_apply[i]=0.0
                    
                    
                    positions.append(float(pos))
                    
                    # Apply current command with safety limits
                    current_cmd = commands_to_apply[i] if i < len(commands_to_apply) else 0.0
                    self.get_logger().debug(f'Applying current {current_cmd} mA to motor ID {motor.ID}')
                    self.interface.set_motor_current(motor, current_cmd)
                    
                except Exception as e:
                    self.get_logger().error(f'Error controlling motor {i}: {e}')
                    positions.append(-1)
            else:
                positions.append(-1)
        
        # Publish positions
        pos_msg = Float32MultiArray()
        pos_msg.data = positions
        self.pos_publisher.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelManagerNode()
    
    def signal_handler(sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\nKeyboard interrupt received. Shutting down...")
        node.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



