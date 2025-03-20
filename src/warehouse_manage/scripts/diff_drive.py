# go_to_goal.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math
from tf_transformations import euler_from_quaternion

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        # Configuration parameters
        self.target = Point(x=5.0, y=3.0)  # Set your target coordinates here
        self.linear_speed = 0.5           # Fixed linear speed (m/s)
        self.angular_kp = 1.0             # Proportional gain for angular velocity
        self.distance_tolerance = 0.1     # Stop when within this distance (meters)
        self.angular_tolerance = 0.1      # Angular tolerance (radians)
        
        # ROS 2 Jazzy specific topic names
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        
        self.current_pose = None
        self.get_logger().info(f"Navigating to target: ({self.target.x}, {self.target.y})")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # Calculate distance to target
        dx = self.target.x - current_x
        dy = self.target.y - current_y
        distance = math.hypot(dx, dy)
        
        if distance < self.distance_tolerance:
            self.stop_robot()
            return

        # Calculate target orientation
        target_yaw = math.atan2(dy, dx)
        current_yaw = self.get_current_yaw()
        
        # Calculate angular error (normalized to [-π, π])
        error = target_yaw - current_yaw
        error = (error + math.pi) % (2 * math.pi) - math.pi
        
        # Generate twist command
        twist = Twist()
        
        if abs(error) > self.angular_tolerance:
            # Adjust orientation first
            twist.angular.z = self.angular_kp * error
        else:
            # Move forward with fixed speed
            twist.linear.x = self.linear_speed
        
        self.cmd_vel_pub.publish(twist)

    def get_current_yaw(self):
        """Extract yaw from quaternion"""
        quaternion = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw

    def stop_robot(self):
        """Stop the robot and shutdown node"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Target reached! Shutting down.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()

if __name__ == '__main__':
    main()