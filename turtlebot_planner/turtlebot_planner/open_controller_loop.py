import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import os

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.actions = []
        action_file = 'actions.txt'
        
        if os.path.exists(action_file):
            with open(action_file, 'r') as f:
                for line in f:
                    ul, ur = map(float, line.strip().split(','))
                    self.actions.append((ul, ur))
            self.get_logger().info(f"Loaded {len(self.actions)} actions.")
        else:
            self.get_logger().error("actions.txt not found! Run Part 1 first.")
            return

        self.timer = self.create_timer(2.0, self.execute_trajectory)

    def execute_trajectory(self):
        self.timer.cancel() 
        
        R = 0.038
        L = 0.354
        ACTION_TIME = 1.15
        self.get_logger().info(f"Expected total run time: {len(self.actions) * ACTION_TIME} seconds.")
        
        for i, (ul_rpm, ur_rpm) in enumerate(self.actions):
            ul_rad = ul_rpm * (2 * math.pi / 60.0)
            ur_rad = ur_rpm * (2 * math.pi / 60.0)
            
            v = (R / 2.0) * (ul_rad + ur_rad)
            w = (R / L) * (ur_rad - ul_rad)
            
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            
            self.get_logger().info(f"Step {i+1}/{len(self.actions)} | Cmd: v={v:.3f} m/s, w={w:.3f} rad/s")
            
            start_time = self.get_clock().now().nanoseconds
            duration_ns = ACTION_TIME * 1e9

            while (self.get_clock().now().nanoseconds - start_time) < duration_ns:
                self.publisher_.publish(msg)
                time.sleep(0.05) 
            
        self.publisher_.publish(Twist())
        self.get_logger().info('Path finished')

def main(args=None):
    print("RUNNING ANUSHA")
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()