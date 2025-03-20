import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.move_group = MoveGroupCommander('arm')
        self.scene = PlanningSceneInterface()

    def go_to_pose(self, x, y, z):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'base_link'
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.w = 1.0

        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return plan

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()

    x, y, z = 0.5, 0.0, 0.5  # Example coordinates
    success = node.go_to_pose(x, y, z)
    if success:
        node.get_logger().info('Reached the target pose')
    else:
        node.get_logger().info('Failed to reach the target pose')

    rclpy.shutdown()

if __name__ == '__main__':
    main()