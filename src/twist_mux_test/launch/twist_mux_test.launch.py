#!/usr/bin/env python3


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('twist_mux_test'),
                                        'config', 'params.yaml')
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')

    declare_joy_vel = DeclareLaunchArgument('joy_vel', default_value='/test_twist_mux/cmd_vel')
    declare_joy_config = DeclareLaunchArgument('joy_config', default_value='ps3')
    declare_joy_dev = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    declare_config_filepath = DeclareLaunchArgument('config_filepath', default_value=[
        TextSubstitution(text=os.path.join(
            get_package_share_directory('teleop_twist_joy'), 'config', '')),
        joy_config, TextSubstitution(text='.config.yaml')])
    
    joy_node = Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])
        
    teleop_twist_joy_node = Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[config_file_path],
            remappings={('/cmd_vel', '/joy_cmd_vel')},
            )
    teleop_twist_keyboard_node = Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node', 
            remappings={('/cmd_vel', '/key_cmd_vel')},
            output='screen',
            prefix = 'xterm -e',
            )
    twist_mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('joy_vel'))},
            parameters=[config_file_path],
        )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_joy_vel)
    ld.add_action(declare_joy_config)
    ld.add_action(declare_joy_dev)
    ld.add_action(declare_config_filepath)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(teleop_twist_keyboard_node)
    ld.add_action(twist_mux_node)

    return ld