# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# reffer from
# nav2_minimal_turtlebot_simulation/nav2_minimal_tb3_sim/launch/spawn_tb3.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    # 'tugbot'
    #TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    TURTLEBOT3_MODEL = 'tugbot'
    model_folder = TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('tugbot_gazebo_my'),
        'models',
        'tugbot.sdf'
    )

    # add for Bridge ROS tioic
    bridge_params = os.path.join(
        get_package_share_directory('tugbot_gazebo_my'),
        'params',
        #'turtlebot3_waffle_bridge.yaml'
        'tugbot_bridge.yaml'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')

    roll = LaunchConfiguration('roll', default='0.00')
    pitch = LaunchConfiguration('pitch', default='0.00')
    yaw = LaunchConfiguration('yaw', default='0.00')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify the robot x_pose')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify the robot y_pose')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.01',
        description='Specify the robot z_pose')


    declare_roll_cmd = DeclareLaunchArgument(
        'roll', default_value='0.00',
        description='Specify the robot roll')

    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch', default_value='0.00',
        description='Specify the robot pitch')

    declare_yaw_cmd = DeclareLaunchArgument(
        #'yaw', default_value='3.14159',
        'yaw', default_value='0.00',
        description='Specify the robot yaw')

    #start_gazebo_ros_spawner_cmd = Node(
    #    package='gazebo_ros',
    #    executable='spawn_entity.py',
    #    arguments=[
    #        '-entity', TURTLEBOT3_MODEL,
    #        '-file', urdf_path,
    #        '-x', x_pose,
    #        '-y', y_pose,
    #        '-z', '0.01'
    #    ],
    #    output='screen',
    #)

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen',
    )

    # add for Bridge ROS tioic
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # add for camara image Bridge
    #start_gazebo_ros_image_bridge_cmd = Node(
    #    package='ros_gz_image',
    #    executable='image_bridge',
    #    arguments=['/camera/image_raw'],
    #    output='screen',
    #)

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)

    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    #ld.add_action(start_gazebo_ros_image_bridge_cmd)

    return ld
