import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
import os
from ament_index_python.packages import get_package_share_directory


# ---------------- Config loading ----------------

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

def main(args=None):
    rclpy.init(args=args)
    node = HaplyConnectionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
