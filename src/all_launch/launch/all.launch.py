from dash import Output
import launch
import os
import launch.launch_description_sources
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Launch
    # gazebo 仿真
    gazebo_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('go2_config') + '/launch/gazebo_velodyne.launch.py'
        )
    )
    
    # lio-sam 算法
    lio_sam_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('lio_sam') + '/launch/lidar.launch.py'
        )
    )

    # point2laser 转换
    point2laser_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('pointcloud_to_laserscan') + '/launch/pointcloud_to_laserscan_launch.py'
        )
    )

    # 自定义 point2laser 转换
    sample_point2laser_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('pointcloud_to_laserscan') + '/launch/sample_pointcloud_to_laserscan_launch.py'
        )
    )

    # map_server 地图
    map_file = os.path.join(get_package_share_directory('all_launch'), 'maps', 'gazebo_map.yaml')
    map_server_launch = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file},
                    {'use_sim_time': True}]
    )

    lifecycle_manager_launch = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # Cartographer 2D 建图
    cartographer_2d_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            get_package_share_directory('gazebo_cartographer') + '/launch/cartographer.launch.py'
        )
    )

    # amcl 定位
    nav2_yaml = os.path.join(get_package_share_directory('all_launch'), 'config', 'nav2.yaml')
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    # 生命循环节点
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['amcl']}]
    )

    '''
    录制
        gazebo 仿真
        启动 lio-sam 算法
    '''
    start_action_group = launch.actions.GroupAction([
        # 启动仿真
        launch.actions.TimerAction(period=0.5, actions=[gazebo_launch]),
        # 启动 point2laser 转换
        # launch.actions.TimerAction(period=1.5, actions=[point2laser_launch]),
        # 启动自定义 point2laser 转换
        launch.actions.TimerAction(period=2.5, actions=[sample_point2laser_launch]),
        # 启动 lio-sam 算法
        launch.actions.TimerAction(period=10.5, actions=[lio_sam_launch]),
        # 启动 Cartographer 2D 建图
        # launch.actions.TimerAction(period=11.0, actions=[cartographer_2d_launch]),
        # 启动 map_server 地图
        # launch.actions.TimerAction(period=11.5, actions=[map_server_launch]),
        # launch.actions.TimerAction(period=12.5, actions=[lifecycle_manager_launch]),
        # 启动 amcl 定位
        # launch.actions.TimerAction(period=13.5, actions=[amcl_node]),
        # 启动生命循环节点
        # launch.actions.TimerAction(period=14.5, actions=[lifecycle_node]),
    ])


    return launch.LaunchDescription([
        start_action_group
    ])