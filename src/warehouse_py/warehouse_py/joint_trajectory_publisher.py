import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        # self.publisher_ = self.create_publisher(JointTrajectory, '/arm_trajectory_controller/joint_trajectory', 10)
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def send_goal(self):
        # Wait for the action server to be available
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return
        
        goal_msg = FollowJointTrajectory.Goal()

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6',]

        # Create a point for the trajectory
        point = JointTrajectoryPoint()
        point.positions = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        # point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        # Add the point to the trajectory
        trajectory_msg.points.append(point)

        goal_msg.trajectory = trajectory_msg

        self.get_logger().info('Sending goal...')
        return self.action_client.send_goal_async(goal_msg)

        
    # def send_goal_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().error('Goal rejected')
    #         return
    #     self.get_logger().info('Goal accepted')

    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.result_callback)

    # def result_callback(self, future):
    #     result = future.result().result
    #     if result.error_code == 0:  # SUCCESS
    #         self.get_logger().info('Goal succeeded')
    #     else:
    #         self.get_logger().error(f'Goal failed with error code: {result.error_code}')

class PendulumTrajectory(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        # self.publisher_ = self.create_publisher(JointTrajectory, '/arm_trajectory_controller/joint_trajectory', 10)
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
        )
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def send_goal(self):
        # Wait for the action server to be available
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return
        
        goal_msg = FollowJointTrajectory.Goal()

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['slider_to_cart', 'cart_to_pendulum']

        # Create a point for the trajectory
        point = JointTrajectoryPoint()
        point.positions = [5.0, -1.0]
        # point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        # Add the point to the trajectory
        trajectory_msg.points.append(point)

        goal_msg.trajectory = trajectory_msg

        self.get_logger().info('Sending goal...')
        return self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    action_client = JointTrajectoryPublisher()
    # action_client = PendulumTrajectory()

    future = action_client.send_goal()
    
    # action_client.send_goal_callback(future=future)
    rclpy.spin_until_future_complete(action_client, future=future)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
