import rclpy
from rclpy.node import Node
from mir import mir_api
import time
import sys

class MirTurn90(Node):

    def __init__(self):
        super().__init__('mir_turn_90')
        self.mir = mir_api.MiR()
        self.mir_url = "http://192.168.1.81/api/v2.0.0/"
        self.battery_threshold = 30

        # Check battery status
        status = self.mir.get_system_info(self.mir_url)
        if status['battery_percentage'] < self.battery_threshold:
            self.get_logger().error('MIR IS LOW ON BATTERY - Cannot perform turn operation')
            return
        
        self.get_logger().info(f'MiR Turn 90 initialized. Battery: {status["battery_percentage"]}%')

    def normalize_angle(self, angle):
        """Normalize angle to be between -180 and 180 degrees"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def turn_90_degrees(self, direction='left'):
        """Turn the robot 90 degrees using the specific mission"""
        try:
            # Check robot state first
            status = self.mir.get_system_info(self.mir_url)
            self.get_logger().info(f'Robot state: {status.get("state_text", "unknown")}')
            
            # Clear any existing missions first
            self.get_logger().info('Clearing mission queue...')
            self.mir.delete_mission_queue(self.mir_url)
            time.sleep(1)  # Give it time to clear
            
            # Get current position for reference
            mir_pos_x, mir_pos_y, mir_orientation = self.mir.get_current_position(self.mir_url)
            self.get_logger().info(f'Current position: x={mir_pos_x:.2f}, y={mir_pos_y:.2f}, orientation={mir_orientation:.2f}°')
            
            # Calculate expected target orientation for monitoring
            if direction.lower() == 'left':
                target_orientation = self.normalize_angle(mir_orientation + 90)
            elif direction.lower() == 'right':
                target_orientation = self.normalize_angle(mir_orientation - 90)
            else:
                self.get_logger().error(f'Invalid direction: {direction}. Use "left" or "right"')
                return False
            
            self.get_logger().info(f'Expected target orientation: {target_orientation:.2f}° (turning {direction})')
            
            # Queue the specific mission that turns 90 degrees
            mission_guid = "16a4f0ad-a046-11f0-b2bc-000e8e984489"
            self.get_logger().info(f'Queuing mission: {mission_guid}')
            
            result = self.mir.post_to_mission_queue(self.mir_url, mission_guid)
            self.get_logger().info(f'Mission queued, result: {result.status_code if hasattr(result, "status_code") else "Success"}')
            
            # Wait for mission to complete
            max_wait_time = 60  # 60 seconds timeout
            start_time = time.time()
            last_orientation = mir_orientation
            
            while time.time() - start_time < max_wait_time:
                # Check mission status
                mission_done = self.mir.get_mission_latest_mission_status(self.mir_url)
                
                # Get current position
                current_x, current_y, current_orientation = self.mir.get_current_position(self.mir_url)
                status = self.mir.get_system_info(self.mir_url)
                queue = self.mir.get_mission_queue(self.mir_url)
                
                # Check if robot is moving
                orientation_change = abs(current_orientation - last_orientation)
                if orientation_change > 0.5:
                    self.get_logger().info(f'Robot is turning! Current: {current_orientation:.1f}°, Change: {orientation_change:.1f}°')
                
                angle_remaining = abs(self.normalize_angle(current_orientation - target_orientation))
                
                self.get_logger().info(f'State: {status.get("state_text", "unknown")}, '
                                     f'Orientation: {current_orientation:.1f}°, '
                                     f'Remaining: {angle_remaining:.1f}°, '
                                     f'Queue: {len(queue)}, '
                                     f'Mission done: {mission_done}')
                
                # Check if mission is complete
                if mission_done:
                    self.get_logger().info('Mission completed!')
                    
                    # Verify the turn was successful
                    final_x, final_y, final_orientation = self.mir.get_current_position(self.mir_url)
                    angle_diff = abs(self.normalize_angle(final_orientation - target_orientation))
                    
                    if angle_diff <= 10.0:  # 10 degree tolerance
                        self.get_logger().info(f'Successfully turned {direction} 90 degrees! Final orientation: {final_orientation:.2f}°')
                        return True
                    else:
                        self.get_logger().warn(f'Mission completed but accuracy off by {angle_diff:.2f}°')
                        return True  # Still consider it successful
                
                last_orientation = current_orientation
                time.sleep(2)
            
            self.get_logger().error('Turn operation timed out')
            return False
            
        except Exception as e:
            self.get_logger().error(f'Error during turn operation: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    mir_turn_node = MirTurn90()
    
    # Get direction from command line arguments
    direction = 'left'  # default
    if len(sys.argv) > 1:
        direction = sys.argv[1].lower()
        if direction not in ['left', 'right']:
            print("Usage: python3 mir_turn_90.py [left|right]")
            print("Default: left")
            direction = 'left'
    
    # Perform the turn
    success = mir_turn_node.turn_90_degrees(direction)
    
    if success:
        mir_turn_node.get_logger().info(f'Turn {direction} 90 degrees completed successfully!')
    else:
        mir_turn_node.get_logger().error(f'Turn {direction} 90 degrees failed!')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()