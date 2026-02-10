import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
import os
from ament_index_python.packages import get_package_share_directory
import asyncio
import websockets
import orjson
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
import time

config_package_name = 'shared_config'
config_file_name = 'glove_settings.json'
share_directory = get_package_share_directory(config_package_name)
config_file_path = os.path.join(share_directory, 'config', config_file_name)

try:
    with open(config_file_path, 'r') as f:
        config = json.load(f)
except FileNotFoundError:
    raise RuntimeError(f"Config not found: {config_file_path}")


class HaplyConnectionManager(Node):
    def __init__(self):
        super().__init__('haply_connection_manager')
        logger = self.get_logger()
        logger.info("Haply Connection Manager initialized")
        
        # WebSocket configuration
        self.uri = 'ws://localhost:10001'
        self.inverse3_device_id = None
        self.verse_grip_device_id = None
        self.handedness = None
        
        # Force tracking for timeout safety
        self.last_force_time = time.time()
        self.force_timeout = 1.0  # seconds
        self.current_force = {"x": 0, "y": 0, "z": 0}
        
        # Publishers
        self.position_publisher = self.create_publisher(
            Float32MultiArray, 'haply_position', 10)
        self.velocity_publisher = self.create_publisher(
            Float32MultiArray, 'haply_velocity', 10)
        self.orientation_publisher = self.create_publisher(
            Float32MultiArray, 'haply_orientation', 10)
        self.buttons_publisher = self.create_publisher(
            Float32MultiArray, 'haply_buttons', 10)
        
        # Subscriber for force commands
        self.force_subscriber = self.create_subscription(
            Float32MultiArray, 'haply_forces', self.force_callback, 10)
        
        # Logging timer (every 5 seconds)
        self.log_timer = self.create_timer(5.0, self.log_status)
        
        # Start the async websocket loop in a separate thread
        self.ws_thread = Thread(target=self.run_websocket_loop, daemon=True)
        self.ws_thread.start()
    
    def force_callback(self, msg: Float32MultiArray):
        """Handle incoming force commands from haply_forces topic"""
        if len(msg.data) >= 3:
            self.current_force = {
                "x": float(msg.data[0]),
                "y": float(msg.data[1]),
                "z": float(msg.data[2])
            }
            self.last_force_time = time.time()
    
    def log_status(self):
        """Log status every 5 seconds"""
        logger = self.get_logger()
        logger.info(
            f"Inverse3 ID: {self.inverse3_device_id}, "
            f"Verse Grip ID: {self.verse_grip_device_id}, "
            f"Handedness: {self.handedness}, "
            f"Current Force: {self.current_force}"
        )
    
    def get_safe_force(self) -> dict:
        """Get force with timeout safety - returns zeros if no recent force input"""
        elapsed = time.time() - self.last_force_time
        if elapsed > self.force_timeout:
            return {"x": 0, "y": 0, "z": 0}
        return self.current_force
    
    def publish_state(self, position, velocity, orientation, buttons):
        """Publish device state to ROS topics"""
        # Publish position
        pos_msg = Float32MultiArray()
        pos_msg.data = [
            float(position.get("x", 0)),
            float(position.get("y", 0)),
            float(position.get("z", 0))
        ]
        self.position_publisher.publish(pos_msg)
        
        # Publish velocity
        vel_msg = Float32MultiArray()
        vel_msg.data = [
            float(velocity.get("x", 0)),
            float(velocity.get("y", 0)),
            float(velocity.get("z", 0))
        ]
        self.velocity_publisher.publish(vel_msg)
        
        # Publish orientation
        ori_msg = Float32MultiArray()
        ori_msg.data = [
            float(orientation.get("x", 0)),
            float(orientation.get("y", 0)),
            float(orientation.get("z", 0)),
            float(orientation.get("w", 1))
        ]
        self.orientation_publisher.publish(ori_msg)
        
        # Publish buttons
        btn_msg = Float32MultiArray()
        btn_msg.data = [float(buttons.get(key, 0)) for key in sorted(buttons.keys())]
        self.buttons_publisher.publish(btn_msg)
    
    def run_websocket_loop(self):
        """Run the async websocket loop"""
        asyncio.run(self.websocket_main())
    
    async def websocket_main(self):
        """Main asynchronous websocket loop"""
        logger = self.get_logger()
        first_message = True
        
        try:
            async with websockets.connect(self.uri) as ws:
                while True:
                    try:
                        # Receive data from the device
                        response = await ws.recv()
                        data = orjson.loads(response)
                        
                        # Get devices list from the data
                        inverse3_devices = data.get("inverse3", [])
                        verse_grip_devices = data.get("wireless_verse_grip", [])
                        
                        # Get the first device from each list
                        inverse3_data = inverse3_devices[0] if inverse3_devices else {}
                        verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}
                        
                        # Handle the first message to get device IDs
                        if first_message:
                            first_message = False
                            
                            if not inverse3_data:
                                logger.error("No Inverse3 device found.")
                                break
                            
                            self.inverse3_device_id = inverse3_data.get("device_id")
                            self.handedness = inverse3_data.get("config", {}).get("handedness")
                            logger.info(
                                f"Inverse3 device ID: {self.inverse3_device_id}, "
                                f"Handedness: {self.handedness}"
                            )
                            
                            if verse_grip_data:
                                self.verse_grip_device_id = verse_grip_data.get("device_id")
                                logger.info(f"Wireless Verse Grip device ID: {self.verse_grip_device_id}")
                        
                        # Extract state data
                        position = inverse3_data.get("state", {}).get("cursor_position", {})
                        velocity = inverse3_data.get("state", {}).get("cursor_velocity", {})
                        orientation = verse_grip_data.get("state", {}).get("orientation", {})
                        buttons = verse_grip_data.get("state", {}).get("buttons", {})
                        
                        # Publish the state
                        self.publish_state(position, velocity, orientation, buttons)
                        
                        # Get force with timeout safety
                        safe_force = self.get_safe_force()
                        
                        # Prepare and send force command message
                        request_msg = {
                            "inverse3": [
                                {
                                    "device_id": self.inverse3_device_id,
                                    "commands": {
                                        "set_cursor_force": {
                                            "values": safe_force
                                        }
                                    }
                                }
                            ]
                        }
                        
                        await ws.send(orjson.dumps(request_msg))
                        
                    except asyncio.TimeoutError:
                        logger.warn("WebSocket receive timeout")
                        continue
                    
        except Exception as e:
            logger.error(f"WebSocket connection error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HaplyConnectionManager()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()