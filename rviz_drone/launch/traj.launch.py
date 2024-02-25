#!/usr/bin/env python3
"""
Map trajectories
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # traj_ns = LaunchConfiguration('traj_ns', default='traj_frame')
    
    scale_size_arg = DeclareLaunchArgument(
        'scale_size',
        default_value='0.05',
        description='size of trajectory marker'
    )
    
    life_time_arg = DeclareLaunchArgument(
        'life_time',
        default_value='2.0',
        description='life time of trajectory marker'
    )
    
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='map',
        description='parent frame of trajectory marker'
    )
    
    child_frame_arg = DeclareLaunchArgument(
        'child_frame',
        default_value='trajectory_frame',
        description='child frame of trajectory marker'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='30.0',
        description='rate of trajectory marker'
    )
    
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='trajectory',
        description='namespace of trajectory marker'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='',
        description='topic name of trajectory marker'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='30.0',
        description='rate of trajectory marker'
    )
    
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='trajectory',
        description='namespace of trajectory marker'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='trajectory',
        description='topic name of trajectory marker'
    )
    
    red_color_arg = DeclareLaunchArgument(
        'red_color',
        default_value='1.0',
        description='red color of trajectory marker'
    )
    
    green_color_arg = DeclareLaunchArgument(
        'green_color',
        default_value='0.0',
        description='green color of trajectory marker'
    )
    
    blue_color_arg = DeclareLaunchArgument(
        'blue_color',
        default_value='0.0',
        description='blue color of trajectory marker'
    )
    
    alpha_color_arg = DeclareLaunchArgument(
        'alpha_color',
        default_value='1.0',
        description='alpha color of trajectory marker'
    )
    
    traj_vis_node = Node(
        package='rviz_drone',
        executable='traj_vis.py',
        namespace=[LaunchConfiguration('ns')],
        remappings=[
            ('trajectory', LaunchConfiguration('topic_name'))
        ],
        parameters=[
            # {'scale_size': LaunchConfiguration('scale_size')},
            {'life_time': LaunchConfiguration('life_time')},
            {'parent_frame': LaunchConfiguration('parent_frame')},
            {'child_frame': LaunchConfiguration('child_frame')},
            {'rate': LaunchConfiguration('rate')},
            {'ns': LaunchConfiguration('ns')},
            {'topic_name': LaunchConfiguration('topic_name')},
            {'red_color': LaunchConfiguration('red_color')},
            {'green_color': LaunchConfiguration('green_color')},
            {'blue_color': LaunchConfiguration('blue_color')},
            {'alpha_color': LaunchConfiguration('alpha_color')}
        ]
    )
    
    ld = LaunchDescription([
        scale_size_arg,
        life_time_arg,
        parent_frame_arg,
        child_frame_arg,
        rate_arg,
        ns_arg,
        topic_name_arg,
        red_color_arg,
        green_color_arg,
        blue_color_arg,
        alpha_color_arg,
        traj_vis_node
    ])

    
    return ld
    
    