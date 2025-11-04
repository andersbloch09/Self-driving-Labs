#!/usr/bin/env python3

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ot2_interfaces.action import InitializeProtocol


class InitCaller(Node):
    def __init__(self, protocol_path: str, custom_labware_folder: str | None = None):
        super().__init__('init_caller')
        self._client = ActionClient(self, InitializeProtocol, 'initialize_protocol')
        self.protocol_path = protocol_path
        self.custom_labware_folder = custom_labware_folder or ''

        # State
        self._goal_handle = None
        self._result_received = False

    def send_init(self, timeout_sec: float = 5.0):
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('initialize_protocol action server not available (timeout)')
            return False

        goal = InitializeProtocol.Goal()
        goal.protocol_path = self.protocol_path
        goal.custom_labware_folder = self.custom_labware_folder

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)
        return True

    def _feedback_cb(self, feedback_msg):
        try:
            self.get_logger().info(f"Feedback: {feedback_msg.feedback.status}")
        except Exception:
            pass

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Initialize goal rejected')
            self._result_received = True
            return

        self.get_logger().info('Initialize goal accepted')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        goal_result = future.result()
        result = goal_result.result

        # Print commonly expected fields
        try:
            self.get_logger().info(f"Result: success={result.success}, message='{result.message}'")
        except Exception:
            pass

        if hasattr(result, 'run_id'):
            self.get_logger().info(f"run_id: {result.run_id}")

        if hasattr(result, 'labware_list') and result.labware_list:
            for lw in result.labware_list:
                try:
                    self.get_logger().info(f"Labware: id={lw.id}, load_name={lw.load_name}, slot={lw.slot_name}")
                except Exception:
                    pass

        self._result_received = True

    def cancel(self):
        if self._goal_handle is None:
            self.get_logger().warn('No active goal to cancel')
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda f: self.get_logger().info('Cancel requested'))


def default_protocol_path():
    pkg_dir = os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control')
    return os.path.join(pkg_dir, 'protocols', 'fehlings_test_protocol.py')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Call InitializeProtocol action')
    parser.add_argument('--protocol', '-p', default=default_protocol_path(), help='Path to protocol .py')
    parser.add_argument('--labware', '-l', default=os.path.join(os.path.expanduser('~'), 'sdl_ws', 'src', 'ot2_control', 'ot2_control', 'custom_labware'), help='Custom labware folder')
    args = parser.parse_args(argv)

    rclpy.init()
    node = InitCaller(args.protocol, args.labware)

    ok = node.send_init(timeout_sec=5.0)
    if not ok:
        node.destroy_node()
        rclpy.shutdown()
        return 1

    try:
        # Spin until we receive result
        while rclpy.ok() and not node._result_received:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, attempting to cancel goal...')
        node.cancel()
    finally:
        node.get_logger().info('Shutting down init caller')
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
