import math
from math import sin, cos, atan2, asin, pi
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped

from aster_action_interfaces.action import GoToGoal
from aster_controller.helper.pid import PidController

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('aster_go_to_goal_node')
        self.get_logger().info("Aster's go to goal node is running")

        self.create_subscription(
            Odometry,
            '/odom',
            self.get_odometry,
            qos_profile=qos_profile_system_default
        )

        self.twist_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=qos_profile_system_default
        )

        self.go_to_goal_action_service = ActionServer(
            self,
            GoToGoal,
            'go_to_goal',
            execute_callback = self.execute_callback,
            callback_group = ReentrantCallbackGroup(),
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback
        )

        self.is_moving = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_r = 0.0
        self.goal_theta = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.linear_command = 0.0
        self.angular_command = 0.0

        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('alpha', 5)
        self.declare_parameter('delta_time', 0.1)
        self.declare_parameter('Kp', 0.1)
        self.declare_parameter('Ki', 0.001)
        self.declare_parameter('Kd', 0.01)
        self.declare_parameter('goal_tolerance', 0.07)

        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.alpha = self.get_parameter('alpha').value
        self.delta_time = self.get_parameter('delta_time').value
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.angle_pid = PidController(self.Kp, self.Ki, self.Kd, self.delta_time, True)
        self.angle_pid.set_output_limits(-3, 3)

    def get_odometry(self, odom):
        self.current_x = odom.pose.pose.position.x
        self.current_y = odom.pose.pose.position.y
        self.current_theta = self._get_euler_from_quaternion(odom.pose.pose.orientation)[2]

        self.goal_r, self.goal_theta = self._get_goal_vector()

    def goal_callback(self, goal):
        if self.is_moving:
            return GoalResponse.REJECT

        self.goal_x = goal.x
        self.goal_y = goal.y

        self.get_logger().info(f'Received goal request: ({self.goal_x}, {self.goal_y})')

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        self.linear_command = 0.0
        self.angular_command = 0.0
        self.send_twist_command()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal):
        self.get_logger().info('Executing action...')

        feedback_msg = GoToGoal.Feedback()

        while not self.update_control_loop():
            self.is_moving = True

            feedback_msg.current_x = self.current_x
            feedback_msg.current_y = self.current_y
            feedback_msg.distance = self._get_distance_to_goal()
            self.send_twist_command()
            goal.publish_feedback(feedback_msg)
            time.sleep(self.delta_time)

        self.is_moving = False
        self.send_twist_command()
        goal.succeed()
        result = GoToGoal.Result()
        result.goal_reached = True
        self.get_logger().info(f'Returning result: {result.goal_reached}')
        return result

    def update_control_loop(self):
        self.desired_r = self.goal_r
        distance_to_goal = self._get_distance_to_goal()

        if distance_to_goal <= self.goal_tolerance:
            self.linear_command = 0.0
            self.angular_command = 0.0
            self.get_logger().info("Already near the goal")
            return True

        self.angle_pid.set_input(self.current_theta)
        self.angle_pid.set_setpoint(self.goal_theta)
        self.angle_pid.compute()

        self.linear_command = self.desired_r
        self.angular_command = self.angle_pid.output
        return False

    def send_twist_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear_command)
        twist_msg.angular.z = float(self.angular_command)
        self.twist_pub.publish(twist_msg)

    def _get_euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * x + y * z)
        pitch = asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _get_goal_vector(self):
        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y

        distance_to_goal = math.sqrt(math.pow(error_x, 2) + math.pow(error_y, 2))

        K = (self.max_linear_vel * (1 - math.exp(-self.alpha * (distance_to_goal * distance_to_goal))) / (distance_to_goal * distance_to_goal)) if distance_to_goal > 0.0001 else 0

        u_x = K * error_x
        u_y = K * error_y

        r = math.sqrt((u_x * u_x) + (u_y * u_y))
        theta = math.atan2(u_y, u_x)
        return r, theta

    def _get_distance_to_goal(self):
        return math.sqrt(math.pow((self.goal_x - self.current_x), 2) + math.pow((self.goal_y - self.current_y), 2))

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(go_to_goal_node, executor=executor)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
