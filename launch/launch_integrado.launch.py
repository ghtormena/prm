#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ───────────────────── caminhos ─────────────────────
    prm_share   = get_package_share_directory('prm')
    nav2_share  = get_package_share_directory('nav2_bringup')
    slam_share  = get_package_share_directory('slam_toolbox')

    nav2_params  = PathJoinSubstitution([prm_share,  'config', 'nav2_params.yaml'])
    slam_params  = PathJoinSubstitution([prm_share,  'config', 'mapper_params_online_async.yaml'])
    world_launch = PathJoinSubstitution([prm_share,  'launch', 'inicia_simulacao.launch.py'])
    robot_launch = PathJoinSubstitution([prm_share,  'launch', 'carrega_robo.launch.py'])

    # ───────────────────── includes ─────────────────────
    inicia_simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        #launch_arguments={'world': 'empty_arena.sdf'}.items()
    )

    carrega_robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch)
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    # ─────────────── ordem de arranque ────────────────
    return LaunchDescription([
        TimerAction(period=2.0, actions=[inicia_simulacao]),
        TimerAction(period=5.0, actions=[carrega_robo]),
        TimerAction(period=10.0, actions=[nav2]),   
        TimerAction(period=15.0, actions=[slam_toolbox]),
    ])
