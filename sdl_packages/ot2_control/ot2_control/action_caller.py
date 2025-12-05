#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import os
import sys
import argparse
from ot2_interfaces.action import RunProtocol
from ot2_interfaces.srv import StopProtocol
import json


class SimpleClient(Node):
    def __init__(self, protocol_path, custom_labware_folder, params):
        super().__init__('simple_client')
        self.client = ActionClient(self, RunProtocol, 'run_protocol')
        self.protocol_path = protocol_path
        self.custom_labware_folder = custom_labware_folder
        self.params = params


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

        goal = RunProtocol.Goal()
        goal.protocol_path = self.protocol_path
        goal.custom_labware_folder = self.custom_labware_folder
        goal.parameters_json = json.dumps(self.params)
        
        self.get_logger().info(f"Sending goal with protocol: {self.protocol_path}")
        self.get_logger().info(f"Parameters: {json.dumps(self.params, indent=2)}")

        self._send_goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=lambda f: self.get_logger().info(f"Feedback: {f.feedback.status}")
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)


    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle

        # Request the final result when the action completes
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

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


    def result_callback(self, future):
        """Called when the action completes with final result"""
        goal_result = future.result()
        result = goal_result.result

        self.get_logger().info('='*60)
        self.get_logger().info('RunProtocol Action Completed!')
        self.get_logger().info('='*60)

        # Display result fields
        try:
            self.get_logger().info(f"Success: {result.success}")
            self.get_logger().info(f"Message: {result.message}")
            if hasattr(result, 'final_status') and result.final_status:
                self.get_logger().info(f"Final Status: {result.final_status}")
            if hasattr(result, 'comments') and result.comments:
                self.get_logger().info(f"Comments: {result.comments}")
        except Exception as e:
            self.get_logger().error(f"Error reading result: {e}")

        self.get_logger().info('='*60)
        
        # Shutdown the node cleanly
        self.get_logger().info('Shutting down action caller...')
        rclpy.shutdown()


def default_protocol_path():
    pkg_dir = os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control')
    return os.path.join(pkg_dir, 'protocols', 'full_protocol.py')

def default_labware_folder():
    pkg_dir = os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control')
    return os.path.join(pkg_dir, 'custom_labware')

def main(argv=None):
    parser = argparse.ArgumentParser(description='Call RunProtocol action with custom parameters')
    parser.add_argument('--protocol', '-p', default=default_protocol_path(), 
                        help='Path to protocol .py file')
    parser.add_argument('--labware', '-l', default=default_labware_folder(),
                        help='Custom labware folder path')
    parser.add_argument('--sample-count', type=int, default=1,
                        help='Number of samples to process (default: 1)')
    parser.add_argument('--heating-time', type=int, default=60,
                        help='Heating time in seconds (default: 60)')
    parser.add_argument('--large-tips-used', type=int, default=0,
                        help='Number of large tips already used (default: 0)')
    parser.add_argument('--max-concentration', type=float, default=5.0,
                        help='Maximum concentration (default: 5.0)')
    parser.add_argument('--min-concentration', type=float, default=1.0,
                        help='Minimum concentration (default: 1.0)')
    
    args = parser.parse_args(argv)
    
    # Build parameters dictionary
    params = {
        "sample_count": args.sample_count,
        "heating_time": args.heating_time,
        "large_tips_used": args.large_tips_used,
        "max_concentration": args.max_concentration,
        "min_concentration": args.min_concentration,
    }
    
    rclpy.init()
    client = SimpleClient(args.protocol, args.labware, params)
    client.send_goal()
    rclpy.spin(client)
    return 0

if __name__ == "__main__":
    sys.exit(main())
