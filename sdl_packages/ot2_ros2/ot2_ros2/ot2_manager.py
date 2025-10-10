import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from threading import Thread, Event
from ot2_ros2.action import RunProtocol
from ot2_ros2.srv import StopProtocol
from ot2_ros2.ot2_client import OT2Client
import time

class OT2Manager(Node):
    def __init__(self):
        super().__init__('ot2_manager')

        # Use your fixed OT-2 IP
        self.ip = "169.254.122.228"
        self.client = OT2Client(self.ip)

        self.current_run_id = None
        self.stop_event = Event()
        self.action_server = ActionServer(
            self,
            RunProtocol,
            'run_protocol',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.stop_service = self.create_service(
            StopProtocol,
            'stop_protocol',
            self.stop_protocol_callback
        )

        self.get_logger().info(f"✅ OT2 Manager Node initialized (target IP: {self.ip})")


    # ---------- ACTION CALLBACKS ----------

    def goal_callback(self, goal_request):
        if self.current_run_id is not None:
            self.get_logger().warn("A protocol is already running. Rejecting new goal.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received for protocol.")
        self.stop_event.set()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        protocol_path = goal_handle.request.protocol_path
        custom_labware_folder = goal_handle.request.custom_labware_folder
        poll_interval = goal_handle.request.poll_interval or 5.0

        self.stop_event.clear()
        self.get_logger().info(f"Starting OT-2 protocol: {protocol_path}")

        result = RunProtocol.Result()

        try:
            # Upload protocol
            custom_labware = self.client.verify_labware(protocol_path, custom_labware_folder)
            protocol_id = self.client.upload_protocol(protocol_path, custom_labware)
            run_id = self.client.create_run(protocol_id)
            self.current_run_id = run_id
            self.client.start_run(run_id)
            self.get_logger().info(f"Run started with ID: {run_id}")

            # Monitor loop
            while rclpy.ok():
                status = self.client.get_run_status(run_id)
                goal_handle.publish_feedback(RunProtocol.Feedback(status=status))

                if self.stop_event.is_set():
                    self.get_logger().warn("Stop requested — stopping run...")
                    self.client.stop_run(run_id)
                    status = self.client.get_run_status(run_id)
                    result.success = False
                    result.message = "Protocol stopped manually."
                    result.final_status = status
                    self.current_run_id = None
                    goal_handle.canceled()
                    return result

                if goal_handle.is_cancel_requested:
                    self.get_logger().warn("Action cancel requested — stopping run...")
                    self.client.stop_run(run_id)
                    status = self.client.get_run_status(run_id)
                    result.success = False
                    result.message = "Protocol canceled."
                    result.final_status = status
                    self.current_run_id = None
                    goal_handle.canceled()
                    return result

                if status in ("succeeded", "failed", "stopped"):
                    result.success = status == "succeeded"
                    result.final_status = status
                    result.message = f"Protocol finished with status: {status}"
                    self.current_run_id = None
                    goal_handle.succeed()
                    return result

                time.sleep(poll_interval)

        except Exception as e:
            self.get_logger().error(f"Error during protocol run: {e}")
            result.success = False
            result.message = str(e)
            result.final_status = "error"
            self.current_run_id = None
            goal_handle.abort()
            return result

    # ---------- SERVICE CALLBACK ----------

    def stop_protocol_callback(self, request, response):
        """Stops the currently running protocol (if any)."""
        if self.current_run_id is None:
            response.success = False
            response.message = "No active protocol run to stop."
            return response

        try:
            self.client.stop_run(self.current_run_id)
            self.stop_event.set()
            response.success = True
            response.message = f"Run {self.current_run_id} stopped successfully."
            self.get_logger().info(response.message)
            self.current_run_id = None
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop run: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = OT2Manager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
