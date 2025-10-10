import rclpy
from rclpy.node import Node
from . import mir_api
import time
import sys

class MirGoTo(Node):

    def __init__(self):
        super().__init__('mir_goto')
        self.mir = mir_api.MiR_API()
        self.mir_url = "http://192.168.1.81/api/v2.0.0/"
        self.battery_threshold = 30

        # Check battery status
        status = self.mir.get_system_info(self.mir_url)
        if status['battery_percentage'] < self.battery_threshold:
            self.get_logger().error('MIR IS LOW ON BATTERY - Cannot perform goto operation')
            return
        
        self.get_logger().info(f'MiR GoTo initialized. Battery: {status["battery_percentage"]}%')

    def execute_mission(self, mission_id, mission_name="Unknown"):
        """Execute a specific mission by ID"""
        try:
            self.get_logger().info(f'Starting execution of mission: {mission_name} (ID: {mission_id})')
            
            # Get current position
            mir_pos_x, mir_pos_y, mir_orientation = self.mir.get_current_position(self.mir_url)
            self.get_logger().info(f'Current position: x={mir_pos_x:.2f}, y={mir_pos_y:.2f}, orientation={mir_orientation:.1f}°')
            
            # Clear any existing missions in queue
            self.get_logger().info('Clearing mission queue...')
            self.mir.delete_mission_queue(self.mir_url)
            time.sleep(1)
            
            # Add mission to queue
            self.get_logger().info(f'Adding mission {mission_id} to queue...')
            response = self.mir.post_to_mission_queue(self.mir_url, mission_id)
            self.get_logger().info(f'Mission queued, response code: {response.status_code}')
            
            if response.status_code == 201:  # Created successfully
                self.get_logger().info('Mission queued successfully!')
                self.monitor_mission_progress()
                return True
            else:
                self.get_logger().error(f'Failed to queue mission. Response: {response.status_code}')
                self.get_logger().error(f'Response text: {response.text}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Mission execution failed: {str(e)}')
            return False

    def monitor_mission_progress(self):
        """Monitor the mission execution progress"""
        self.get_logger().info('Monitoring mission progress...')
        
        max_wait_time = 120  # 2 minutes timeout
        start_time = time.time()
        
        while time.time() - start_time < max_wait_time:
            try:
                # Get current status
                status = self.mir.get_system_info(self.mir_url)
                pos_x, pos_y, orientation = self.mir.get_current_position(self.mir_url)
                queue = self.mir.get_mission_queue(self.mir_url)
                
                self.get_logger().info(f'State: {status.get("state_text", "unknown")}, '
                                     f'Position: ({pos_x:.1f}, {pos_y:.1f}, {orientation:.1f}°), '
                                     f'Queue length: {len(queue)}')
                
                # Check if mission queue is empty (mission completed)
                if len(queue) == 0:
                    self.get_logger().info('Mission queue is empty - mission likely completed!')
                    break
                    
                # Check mission status
                try:
                    mission_done = self.mir.get_mission_latest_mission_status(self.mir_url)
                    if mission_done:
                        self.get_logger().info('Mission completed successfully!')
                        break
                except Exception as e:
                    # Continue monitoring even if status check fails
                    pass
                
                time.sleep(2)  # Wait 2 seconds before next check
                
            except Exception as e:
                self.get_logger().error(f'Error during monitoring: {str(e)}')
                break
                
        self.get_logger().info('Mission monitoring completed')

    def goto_home(self):
        """Execute the GoToHome mission"""
        goto_home_mission_id = "76638485-a4f7-11f0-b2e5-000e8e984489"
        return self.execute_mission(goto_home_mission_id, "GoToHome")

    def goto_storage(self):
        """Execute the GoToStorage mission"""
        goto_storage_mission_id = "94c9f0cf-a4f7-11f0-b2e5-000e8e984489"
        return self.execute_mission(goto_storage_mission_id, "GoToStorage")

    def goto_opentrons2(self):
        """Execute the GoToOpenTrons2 mission"""
        goto_opentrons2_mission_id = "a5c110d4-a4f7-11f0-b2e5-000e8e984489"
        return self.execute_mission(goto_opentrons2_mission_id, "GoToOpenTrons2")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        mir_goto = MirGoTo()
        
        # Test GoToHome mission
        mir_goto.get_logger().info('Testing GoToHome mission...')
        success = mir_goto.goto_home()
        
        if success:
            mir_goto.get_logger().info('GoToHome mission test completed successfully!')
        else:
            mir_goto.get_logger().error('GoToHome mission test failed!')
        
        # Keep the node alive for a bit to see all logs
        time.sleep(5)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()