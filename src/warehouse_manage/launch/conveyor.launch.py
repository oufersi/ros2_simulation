import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    ld = LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                        'launch',
                                        'gz_sim.launch.py'])]),
            # launch_arguments=[('gz_args', [f' -r -v 4 {warehouse_pkg}/worlds/warehouse.sdf'])]),
            launch_arguments=[('gz_args', [f' -r -v 4 conveyor.sdf'])]
        )
    ])
    # ld.add_action(OpaqueFunction(function=robot_state_publisher))

    return ld

if __name__ == "__main__":
    abb_irb4600_support_pkg = get_package_share_directory('abb_irb4600_support')
    warehouse_pkg = get_package_share_directory('warehouse_manage')

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join(abb_irb4600_support_pkg)

    print(os.environ['GZ_SIM_RESOURCE_PATH'])