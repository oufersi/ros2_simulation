# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

    abb_irb4600_support_pkg = get_package_share_directory('abb_irb4600_support')
    warehouse_pkg = get_package_share_directory('warehouse_manage')
    moveit_pkg = get_package_share_directory('abb_irb4600_60_205_moveit_config')

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join(os.path.dirname(abb_irb4600_support_pkg))
    os.environ['GZ_SIM_RESOURCE_PATH'] = '%s:%s' % (os.environ['GZ_SIM_RESOURCE_PATH'], os.path.join(os.path.dirname(warehouse_pkg)))

    print(f"\n{os.environ['GZ_SIM_RESOURCE_PATH']}\n")

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    with open(os.path.join(warehouse_pkg, 'models', 'abb_irb4600_60_205.urdf')) as f:
        robot_description_content = f.read()
    robot_description = {"robot_description" : robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            get_package_share_directory('warehouse_manage'),
            'config',
            'abb_irb4600_60_205.yaml',
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # gz_spawn_box = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     output='screen',
    #     arguments=['-file', os.path.join(warehouse_pkg, 'models', 'test_box.urdf'),
    #                 '-name','box_1',
    #                 '-allow_renaming', 'true',
    #                 '-x', '1.0',
    #                 '-y', '1.0',
    #                 '-z', '4.0',
    #                 '-R', '0.0',
    #                 '-P', '0.0',
    #                 '-Y', '0.0',
    #                 ],
    # )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'abb_irb4600_60_205',
            '-allow_renaming', 'true',
            '-x', '0.0',  # X Position (meters)
            '-y', '0.0',  # Y Position (meters)
            '-z', '0.5',  # Z Position (meters)
            '-R', '0.0',  # Roll (radians)
            '-P', '0.0',  # Pitch (radians)
            '-Y', '0.0'  # Yaw (radians, 90 degrees)
        ],
    )


    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_trajectory_controller'],
    #     output='screen'
    # )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            robot_controllers,
        ],
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # ----------- MoveIt Config ---------------
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="abb_irb4600_60_205", package_name="abb_irb4600_60_205_moveit_config"
        )
        .robot_description(file_path="config/abb_irb4600_60_205.urdf.xacro")
        .robot_description_semantic(file_path="config/abb_irb4600_60_205.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict.update({"use_sim_time": True})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict
        ]
    )

    rviz_config_file = os.path.join(
        moveit_pkg, "config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic
        ]
    )

    ld = LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                        'launch',
                                        'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [f' -r -v 4 {warehouse_pkg}/worlds/warehouse.sdf'])]),
            # launch_arguments=[('gz_args', [f' -r -v 4 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        robot_state_publisher_node,
        gz_spawn_entity,
        bridge,
        move_group_node,
        rviz_node,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
    ])
    # ld.add_action(OpaqueFunction(function=robot_state_publisher))

    return ld

if __name__ == "__main__":
    abb_irb4600_support_pkg = get_package_share_directory('abb_irb4600_support')
    warehouse_pkg = get_package_share_directory('warehouse_manage')

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join(abb_irb4600_support_pkg)

    print(os.environ['GZ_SIM_RESOURCE_PATH'])