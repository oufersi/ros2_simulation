import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectory, JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectoryPoint

class JointTrajectorySender(Node):
    def __init__(self):
        super().__init__('joint_trajectory_sender')
        self.publisher = self.create_publisher(JointTrajectory, '/controller_name/joint_trajectory', 10)

    def send_trajectory(self):
        # Create a JointTrajectory message
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']  # Define joint names here

        # Create the trajectory points
        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5, -0.5]  # Define target positions for joints
        point.velocities = [0.0, 0.0, 0.0]  # Define velocities (optional)
        point.time_from_start.sec = 2  # Time for this trajectory point to complete

        # Add the point to the trajectory
        joint_trajectory.points.append(point)

        # Publish the trajectory
        self.publisher.publish(joint_trajectory)
        self.get_logger().info('Sent joint trajectory')

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_sender = JointTrajectorySender()

    # Send a trajectory
    joint_trajectory_sender.send_trajectory()

    rclpy.spin_once(joint_trajectory_sender)

    joint_trajectory_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
