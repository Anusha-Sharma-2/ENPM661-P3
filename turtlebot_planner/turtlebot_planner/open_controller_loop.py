import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import os

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('p_waypoint_controller')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.x = None
        self.y = None
        self.theta = None

        self.waypoints = []
        self.current_waypoint = 0
        self.valid = True
        self.start_time = None
        self.timer = self.create_timer(0.05, self.control_loop)

        # Change this only if folder moves
        waypoint_file = '/root/proj3_ws/src/ENPM661-P3/Project 3/waypoints.txt'

        self.get_logger().info(f"Current working directory: {os.getcwd()}")
        self.get_logger().info(f"Reading waypoints from: {waypoint_file}")

        if not os.path.exists(waypoint_file):
            self.get_logger().error("waypoints.txt not found. Run main.py first.")
            return

        with open(waypoint_file, 'r') as f:
            for line in f:
                line = line.strip()

                if not line:
                    continue

                x, y = map(float, line.split(','))
                self.waypoints.append((x, y))

        if len(self.waypoints) == 0:
            self.get_logger().error("waypoints.txt exists, but no waypoints were loaded.")
            return

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        self.get_logger().info(f"First 5 waypoints: {self.waypoints[:5]}")
        self.get_logger().info(f"First waypoint: {self.waypoints[0]}")
        self.get_logger().info(f"Last waypoint: {self.waypoints[-1]}")

        self.debug_log_path = "/root/proj3_ws/src/ENPM661-P3/Project 3/odom_waypoint_debug.csv"
        self.debug_log = open(self.debug_log_path, "w")
        self.debug_log.write("idx,target_x,target_y,odom_x,odom_y,dist_error,theta\n")
        self.debug_log.flush()

        self.get_logger().info(f"Writing odom/waypoint debug to: {self.debug_log_path}")

        # Tune these
        self.kp_linear = 1.6
        self.kp_angular = 0.9

        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.9

        self.waypoint_threshold = 0.8  # meters
        # self.kp_linear = 1.0
        # self.kp_angular = 0.45

        # self.max_linear_speed = 0.45
        # self.max_angular_speed = 0.8

        # self.waypoint_threshold = 0.15

        self.valid = True

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = yaw_from_quaternion(msg.pose.pose.orientation)

    def control_loop(self):
        if self.x is None or self.y is None or self.theta is None:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        if self.current_waypoint >= len(self.waypoints):
            self.publisher_.publish(Twist())
            self.get_logger().info("Finished all waypoints.")
            self.get_logger().info(f"Final odom pose: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")
            self.timer.cancel()
            return

        #target_x, target_y = self.waypoints[self.current_waypoint]
        lookahead = 3

        target_index = min(
            self.current_waypoint + lookahead,
            len(self.waypoints) - 1
        )

        target_x, target_y = self.waypoints[target_index]

        dx = target_x - self.x
        dy = target_y - self.y

        distance_error = math.sqrt(dx * dx + dy * dy)

        desired_theta = math.atan2(dy, dx)
        theta_error = wrap_angle(desired_theta - self.theta)

        if distance_error < self.waypoint_threshold:
            self.get_logger().info(
                f"Reached waypoint {self.current_waypoint + 1}/{len(self.waypoints)} "
                f"at dist error {distance_error:.3f}"
            )

            self.current_waypoint += 1
            self.publisher_.publish(Twist())
            return

        msg = Twist()

        # P controller
        linear_speed = self.kp_linear * distance_error
        angular_speed = self.kp_angular * theta_error

        # # Slow down during sharp turns
        # turn_scale = max(0.25, 1.0 - abs(theta_error))

        # linear_speed *= turn_scale

        # If robot is pointed very wrong, turn first instead of driving into wall
        # if abs(theta_error) > 1.2:
        #     linear_speed = 0.0

        # Clamp speeds
        linear_speed = max(min(linear_speed, self.max_linear_speed), 0.0)
        angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        self.publisher_.publish(msg)
        end_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Total run time: {end_time - self.start_time:.2f} seconds")


def main(args=None):
    print("RUNNING 100")

    rclpy.init(args=args)
    node = TurtleBotController()

    if node.valid:
        rclpy.spin(node)
    else:
        node.get_logger().error("Controller did not start because setup failed.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()