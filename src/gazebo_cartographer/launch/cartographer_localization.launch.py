import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare(package='gazebo_cartographer').find('gazebo_cartographer')

    # 基础配置（不使用 .perform）
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    configuration_directory = LaunchConfiguration(
        'configuration_directory', default=os.path.join(pkg_share, 'config')
    )

    # 纯定位固定使用 localization 的 lua，并加载 pbstream
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='cartogrpher_localization.lua'
    )
    pbstream_filename = LaunchConfiguration(
        'pbstream_filename', default='/home/hzy/study/gazebo_ws/src/all_launch/maps/map.pbstream'
    )
    scan_topic = LaunchConfiguration('scan_topic', default='scan')

    # cartographer_node
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
            '-load_state_filename', pbstream_filename,
        ],
        remappings=[('scan', scan_topic)],
    )

    # occupancy grid node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
    )

    # 声明参数（写死默认值，保持可覆盖）
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'))
    ld.add_action(DeclareLaunchArgument('resolution', default_value='0.05', description='Occupancy grid resolution'))
    ld.add_action(DeclareLaunchArgument('publish_period_sec', default_value='1.0', description='OccupancyGrid publish period'))
    ld.add_action(DeclareLaunchArgument('configuration_directory', default_value=os.path.join(pkg_share, 'config'), description='Cartographer config directory'))
    ld.add_action(DeclareLaunchArgument('configuration_basename', default_value='cartogrpher_localization.lua', description='Cartographer lua filename'))
    ld.add_action(DeclareLaunchArgument('pbstream_filename', default_value='/home/hzy/study/gazebo_ws/src/all_launch/maps/map.pbstream', description='Path to .pbstream map for localization'))
    ld.add_action(DeclareLaunchArgument('scan_topic', default_value='scan', description='LaserScan topic'))

    # 添加节点
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    # ============== 集成 Nav2 导航（仅启动导航各Server，不启动AMCL/MapServer） ==============
    default_params_path = PathJoinSubstitution([
        FindPackageShare('nav2'), 'config', 'nav2', 'navigation.yaml'
    ])
    nav2_core_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
    ])

    # 声明 Nav2 相关可覆盖参数
    ld.add_action(DeclareLaunchArgument('params_file', default_value=default_params_path, description='Nav2 params file'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='true', description='Run RViz2'))

    # 启动 Nav2 核心（不包含 AMCL / MapServer），避免与 Cartographer 冲突
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_core_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items(),
        )
    )

    # 可选：启动 RViz2，使用 nav2 包内默认配置
    default_rviz_path = PathJoinSubstitution([
        FindPackageShare('nav2'), 'rviz', 'navigation.rviz'
    ])
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz_path],
            parameters=[{'use_sim_time': use_sim_time_param}],
        )
    )

    return ld