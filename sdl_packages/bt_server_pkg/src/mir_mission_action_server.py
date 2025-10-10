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
        self.get_logger().info(f'Executing mission: {goal_handle.request.mission_id}')
        
        feedback_msg = GoToMission.Feedback()
        result = GoToMission.Result()
        
        try:
            # Clear any existing missions
            self.get_logger().info('Clearing mission queue...')
            self.mir.delete_mission_queue(self.mir_url)
            time.sleep(1)
            
            # Get current position for logging
            mir_pos_x, mir_pos_y, mir_orientation = self.mir.get_current_position(self.mir_url)
            self.get_logger().info(f'Current position: x={mir_pos_x:.2f}, y={mir_pos_y:.2f}, orientation={mir_orientation:.2f}°')
            
            # Queue the mission
            feedback_msg.message = f"Queuing mission {goal_handle.request.mission_id}"
            goal_handle.publish_feedback(feedback_msg)
            
            response = self.mir.post_to_mission_queue(self.mir_url, goal_handle.request.mission_id)
            self.get_logger().info(f'Mission queued, response: {response.status_code if hasattr(response, "status_code") else "Success"}')
            
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
                status = self.mir.get_system_info(self.mir_url)
                queue = self.mir.get_mission_queue(self.mir_url)
                
                current_x, current_y, current_orientation = self.mir.get_current_position(self.mir_url)
                
                feedback_msg.message = f"Mission in progress - State: {status.get('state_text', 'unknown')}, Queue: {len(queue)}"
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(f'State: {status.get("state_text", "unknown")}, '
                                     f'Position: ({current_x:.1f}, {current_y:.1f}, {current_orientation:.1f}°), '
                                     f'Queue: {len(queue)}, Mission done: {mission_done}')
                
                # Check if mission is complete
                if mission_done:
                    self.get_logger().info('Mission completed successfully!')
                    final_x, final_y, final_orientation = self.mir.get_current_position(self.mir_url)
                    
                    goal_handle.succeed()
                    result.node_status = NodeStatus()
                    result.node_status.status = NodeStatus.SUCCESS
                    result.return_message = f"Mission {goal_handle.request.mission_id} completed. Final position: ({final_x:.2f}, {final_y:.2f}, {final_orientation:.2f}°)"
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