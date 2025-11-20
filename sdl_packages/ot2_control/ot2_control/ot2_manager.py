#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from ot2_control.ot2_client import OT2Client
from ot2_interfaces.action import RunProtocol
import time
import asyncio
import json

class OT2Manager(Node):
    def __init__(self):
        super().__init__('ot2_manager')

        # Use your fixed OT-2 IP
        #self.ip = "169.254.122.228"
        self.ip = "192.168.1.152"
        self.client = OT2Client(self.ip)

        self.current_run_id = None
        self.status = None
        self.active_goal = None


        self.action_server = ActionServer(
            self,
            RunProtocol,
            'run_protocol',
            execute_callback=self.execute_callback,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel
        )


        self.get_logger().info(f"OT2 Manager Node initialized (target IP: {self.ip})")


    # ---------- ACTION CALLBACKS ----------
    def execute_callback(self, goal_handle):
        # self.get_logger().info(f"Executing goal: {goal_handle.request.protocol_path}")
        self.active_goal = goal_handle
        # print("The active goal is:   ", self.active_goal)
        result = RunProtocol.Result()

        protocol_path = goal_handle.request.protocol_path
        custom_labware_folder = goal_handle.request.custom_labware_folder
        parameters_json = goal_handle.request.parameters_json
        params = json.loads(parameters_json)

        # Example: start your OT2 run
        self.lights = self.client.turn_lights_on()
        self.status, self.current_run_id, self.current_protocol_id, self.labware = self.client.run_protocol(self, protocol_path, custom_labware_folder, params)
        self.get_logger().info(f"Started OT-2 protocol with run ID: {self.current_run_id}")
        done_commands = []
        while rclpy.ok():
            # Check external stop or client cancel
            #print(goal_handle.is_cancel_requested)
            #print(f"Started OT-2 protocol with run ID: {self.current_run_id}")
            #self.get_logger().info(f"Protocol status: {self.status}")
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel requested — stopping OT-2 protocol...")
                self.client.stop_run(self, self.current_run_id)
                self.lights = self.client.turn_lights_off()

                goal_handle.canceled()
                result.message = "Cancelled via action"
                result.success = False
                self.active_goal = None
                return result

            current_status = self.client.get_run_status(self.current_run_id)

            results, current_command = self.client.get_commands(self.current_protocol_id, self.current_run_id)
            
            total_commands = len(results)-3


            if current_command not in done_commands:
                done_commands.append(current_command)
            command_no = len(done_commands)
            progressPercent = (command_no/total_commands)*100.0
            progress = ("Protocol Completion: " + str(round(progressPercent, 1)) + "%")

            #Check if OT-2 finished normally
            if current_status == "succeeded":
                self.client.blink_lights(3, 0.5)

                for attempt in range(5):
                    commands = self.client.get_finished(self.current_run_id)
                    if commands:
                        break
                    time.sleep(2)

                comment_messages = [
                    cmd["params"]["message"]
                    for cmd in commands
                    if cmd.get("commandType") == "comment"
                ]

                comment_messages_str = json.dumps(comment_messages, indent=2)

                #print(json.dumps(results, indent=2))
                self.get_logger().info("Goal completed successfully")
                goal_handle.succeed()
                result.success = True
                result.message = "Completed successfully"
                result.final_status = current_status
                result.comments = comment_messages_str
                # Before returning
                self.active_goal = None
                return result

            #Normal feedback
            feedback_msg = RunProtocol.Feedback()
            #feedback_msg.status = current_status
            feedback_msg.status = progress
            goal_handle.publish_feedback(feedback_msg)

            # Process pending callbacks once and sleep briefly
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        
        

    def handle_goal(self, goal_request):
        if self.active_goal is not None:
            self.get_logger().warn("Rejecting new goal: another goal is already active.")
            return GoalResponse.REJECT
        self.get_logger().info(f"Accepting new goal: {goal_request.protocol_path}")
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        self.get_logger().info("Cancel request received.")
        return CancelResponse.ACCEPT

 
def main(args=None):
    rclpy.init(args=args)
    node = OT2Manager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
