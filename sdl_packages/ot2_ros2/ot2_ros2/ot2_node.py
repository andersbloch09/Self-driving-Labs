import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ot2_ros2.ot2_client import OT2ClientROS
import os
from ament_index_python.packages import get_package_share_directory

def get_custom_labware_path():
    package_share = get_package_share_directory('ot2_ros2')
    return os.path.join(package_share, 'custom_labware')

class OT2Node(Node):
    def __init__(self):
        super().__init__('ot2_node')
        self.client = OT2ClientROS("169.254.122.228")

        # Subscriptions: listen for commands
        self.subscription = self.create_subscription(
            String,
            '/ot2/run_protocol',
            self.run_protocol_callback,
            10
        )
        
        # Add subscription for stop commands
        self.stop_subscription = self.create_subscription(
            String,
            '/ot2/stop_run',
            self.stop_run_callback,
            10
        )

        # Publications: report status
        self.status_pub = self.create_publisher(String, '/ot2/status', 10)

        self.get_logger().info('OT2 Node initialized and ready.')

    def stop_run_callback(self, msg):
        """Callback to handle stop requests"""
        self.client.request_stop()
        self.get_logger().info('Stop requested for current run')
        self.status_pub.publish(String(data='stop_requested'))

    def run_protocol_callback(self, msg):
        labware_folder = "/home/sdl/sdl_ws/src/ot2_ros2/ot2_ros2/custom_labware"

        protocol_path = msg.data
        self.get_logger().info(f'Received protocol path: {protocol_path}')

        try:
            status, run_id = self.client.run_protocol(protocol_path, 5.0, labware_folder)
            self.get_logger().info(f'Run finished with status {status}, ID {run_id}')
            self.status_pub.publish(String(data=f'{status}:{run_id}'))
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.status_pub.publish(String(data='error'))


def main(args=None):
    rclpy.init(args=args)
    node = OT2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()