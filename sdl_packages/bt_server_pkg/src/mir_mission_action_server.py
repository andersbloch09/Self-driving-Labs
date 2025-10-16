#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from btcpp_ros2_interfaces.action import GoToMission
from btcpp_ros2_interfaces.msg import NodeStatus

# Import the MiR API
from mir.mir_api import MiR_API
import time

class MirMissionActionServer(Node):
    def __init__(self):
        super().__init__('mir_mission_action_server')
        
        # Initialize MiR API
        self.mir_url = "http://192.168.1.81/api/v2.0.0/"
        self.mir = MiR_API()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            GoToMission,
            'go_to_mission',
            self.execute_callback
        )
        
        self.get_logger().info('MiR Mission Action Server started')

    def execute_callback(self, goal_handle):
        feedback_msg = GoToMission.Feedback()
        result = GoToMission.Result()
        
        try: 
            # Queue the mission
            response = self.mir.post_to_mission_queue(self.mir_url, goal_handle.request.mission_id)
            
            # Wait for mission to complete
            max_wait_time = 120  # 2 minutes timeout
            start_time = time.time()
            
            while time.time() - start_time < max_wait_time:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Mission canceled')
                    result.node_status = NodeStatus.FAILURE
                    result.return_message = "Mission canceled by user"
                    return result
                
                # Check mission status
                mission_done = self.mir.get_mission_latest_mission_status(self.mir_url)
    
                # Check if mission is complete
                if mission_done:
                    goal_handle.succeed()
                    result.node_status = NodeStatus()
                    result.node_status.status = NodeStatus.SUCCESS
                    result.return_message = f"Mission SUCCESS."
                    return result
                
                time.sleep(2)
            
            # Mission timed out
            self.get_logger().error('Mission execution timed out')
            goal_handle.abort()
            result.node_status = NodeStatus()
            result.node_status.status = NodeStatus.FAILURE
            result.return_message = f"Mission {goal_handle.request.mission_id} timed out after {max_wait_time} seconds"
            return result
            
        except Exception as e:
            self.get_logger().error(f'Error during mission execution: {str(e)}')
            goal_handle.abort()
            result.node_status = NodeStatus()
            result.node_status.status = NodeStatus.FAILURE
            result.return_message = f"Mission {goal_handle.request.mission_id} failed: {str(e)}"
            return result

def main(args=None):
    rclpy.init(args=args)
    action_server = MirMissionActionServer()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()