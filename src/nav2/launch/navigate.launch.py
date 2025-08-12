# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # 使用PathJoinSubstitution正确构建路径
    default_map_path = PathJoinSubstitution([
        FindPackageShare('nav2'), 'maps', 'map_my.yaml'
    ])
    
    default_params_path = PathJoinSubstitution([
        FindPackageShare('nav2'), 'config', 'nav2', 'navigation.yaml'
    ])
    
    default_rviz_path = PathJoinSubstitution([
        FindPackageShare('nav2'), 'rviz', 'navigation.rviz'
    ])
    
    # Nav2启动文件路径
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='params_file', 
            default_value=default_params_path,
            description='Navigation2 params file'
        ),

        DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sim_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),

        # 启动Nav2核心功能
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'params_file': LaunchConfiguration("params_file"),
                'use_sim_time': LaunchConfiguration("sim"),
                'autostart': 'true'
            }.items()
        ),

        # 日志输出
        LogInfo(msg=['Using map: ', LaunchConfiguration('map')]),
        LogInfo(msg=['Using params: ', LaunchConfiguration('params_file')]),

        # RViz2可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])