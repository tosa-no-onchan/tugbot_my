#!/usr/bin/env python3
#
# turtlebot3_simulations/turtlebot3_gazebo/launch/warehouse.launch.py
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool
#
# 1. build
# $ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select tugbot_gazebo_my
# $ . install/setup.bash
#
# 2. run
# $ sudo ufw disable
# $ ros2 launch tugbot_gazebo_my warehouse.launch.py
#
# 3. control
#  1)  Teleop keyboard
#   $ ros2 run turtlebot3_teleop teleop_keyboard
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
# add by nhishi
from launch.actions import AppendEnvironmentVariable,DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

#from launch.actions import GroupAction
#from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    #ROBOT_MODEL = os.environ['ROBOT_MODEL']

    #spawn_py_path = 'spawn_'+ROBOT_MODEL+'.launch.py'
    spawn_py_path = 'spawn_tugbot.launch.py'

    launch_file_dir = os.path.join(get_package_share_directory('tugbot_gazebo_my'), 'launch')
    #pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # warehouse.world
    #x_pose = LaunchConfiguration('x_pose', default='13.9')
    #y_pose = LaunchConfiguration('y_pose', default='-10.6')
    #z_pose = LaunchConfiguration('z_pose', default='0.10000000000000001')
    #roll = LaunchConfiguration('roll', default='0.00')
    #pitch = LaunchConfiguration('pitch', default='0.000')
    #yaw = LaunchConfiguration('yaw', default='3.14159')
    # 13.9 -10.6 0.10000000000000001

    # warehouse_depot.world
    x_pose = LaunchConfiguration('x_pose', default='-7.9999803934734377')
    y_pose = LaunchConfiguration('y_pose', default='-2.7093603668824591e-13')
    z_pose = LaunchConfiguration('z_pose', default='-0.0044496538355151202')
    roll = LaunchConfiguration('roll', default='-5.107683199815339e-11')
    pitch = LaunchConfiguration('pitch', default='-0.0043979429695369431')
    yaw = LaunchConfiguration('yaw', default='1.9533289609010453e-12')
    #  <pose>-7.9999803934734377 -2.7093603668824591e-13 -0.0044496538355151202 -5.107683199815339e-11 -0.0043979429695369431 1.9533289609010453e-12</pose>

    # test use nav2_minimal_tb3_sim
    #sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    # use wearhouse world
    world = os.path.join(
        get_package_share_directory('tugbot_gazebo_my'),
        'worlds',
        'warehouse_depot.world'
    )

    #world = os.path.join(
    #    get_package_share_directory('nav2_minimal_tb4_sim'),
    #    'worlds',
    #    'depot.sdf'
    #)

    #gzserver_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    #    ),
    #    launch_arguments={'world': world}.items()
    #)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    #gzclient_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #    )
    #)
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # spawn turtlebot3 or tugbot
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            #os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            #os.path.join(launch_file_dir, 'spawn_tugbot.launch.py')
            os.path.join(launch_file_dir,spawn_py_path)
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'roll': roll,
            'pitch' : pitch,
            'yaw' : yaw,
        }.items(),
    )

    # resource path
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('tugbot_gazebo_my'),
                        'models'))

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    # add by nishi 2025.2.5
    ld.add_action(set_env_vars_resources)

    ld.add_action(spawn_turtlebot_cmd)

    return ld
