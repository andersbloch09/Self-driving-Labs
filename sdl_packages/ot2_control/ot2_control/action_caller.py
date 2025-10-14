#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import os
from ot2_interfaces.action import RunProtocol
from ot2_interfaces.srv import StopProtocol


class SimpleClient(Node):
    def __init__(self):
        super().__init__('simple_client')
        self.client = ActionClient(self, RunProtocol, 'run_protocol')


         # Inside your node's __init__ method:
        self.stop_protocol_srv = self.create_service(
            StopProtocol,
            'stop_protocol',
            self.stop_protocol_callback
        )


    def stop_protocol_callback(self, request, response):
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            self.get_logger().info("StopProtocol service called — attempting to cancel goal.")
            self.cancel_goal()
            response.success = True
            response.message = "Cancel requested."
        else:
            self.get_logger().warn("StopProtocol service called but no active goal to cancel.")
            response.success = False
            response.message = "No active goal to cancel."

        return response




    def send_goal(self):
        self.client.wait_for_server()

        pkg_dir = os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control')
        protocol_path = os.path.join(pkg_dir, 'protocols', 'Macro_cuvette_test.py')
        custom_labware_folder = os.path.join(pkg_dir, 'custom_labware')

        goal = RunProtocol.Goal()
        goal.protocol_path = protocol_path
        goal.custom_labware_folder = custom_labware_folder

        self._send_goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=lambda f: self.get_logger().info(f"Feedback: {f.feedback.status}")
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)


    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle

        # Cancel after 5 seconds (or whatever you want)
        #self._cancel_timer = self.create_timer(20, self.cancel_goal)


    def cancel_goal(self):
        self.get_logger().info('Canceling goal...')
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)


    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = SimpleClient()
    client.send_goal()
    rclpy.spin(client)

if __name__ == "__main__":
    main()
