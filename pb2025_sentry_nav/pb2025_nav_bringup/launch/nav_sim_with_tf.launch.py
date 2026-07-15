# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""导航仿真一键启动: Nav2 + RViz + map→odom 静态 TF 桥接

替代手动开三个终端的繁琐操作，一条命令搞定仿真导航。
small_gicp 缺少 PCD 文件时，用静态 TF (map→odom) 桥接。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    slam = LaunchConfiguration("slam")
    use_composition = LaunchConfiguration("use_composition")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1"
    )
    declare_world_cmd = DeclareLaunchArgument(
        "world", default_value="rmuc_2025"
    )
    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False"
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="False"
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True"
    )

    # 导航系统 (包含点云转换、导航栈、手柄遥控、RViz)
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "rm_navigation_simulation_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "world": world,
            "slam": slam,
            "use_composition": use_composition,
            "use_rviz": use_rviz,
        }.items(),
    )

    # map→odom 静态 TF 桥接 (small_gicp PCD 缺失时的替代方案)
    static_tf_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_bridge",
        namespace=namespace,
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        remappings=[("tf_static", "tf_static")],
        output="screen",
    )

    # gimbal_yaw→gimbal_yaw_fake 初始静态 TF (零旋转)
    # fake_vel_transform 的 publishTransform() 由 wall-timer 驱动，第一个 TF 要等
    # 20ms 真实时间才发，sim time 可能已跑了好几秒。在此期间 global/local costmap
    # 和 bt_navigator 做 gimbal_yaw_fake→map 查询时会因 TF 链缺口而超时。
    # 用静态 TF 填上这个初始缺口 (旋转恒为零，与 fake_vel_transform 初始状态一致)。
    gimbal_tf_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gimbal_yaw_init_bridge",
        namespace=namespace,
        arguments=["0", "0", "0", "0", "0", "0", "gimbal_yaw", "gimbal_yaw_fake"],
        remappings=[("tf_static", "tf_static")],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(nav_cmd)
    ld.add_action(static_tf_cmd)
    ld.add_action(gimbal_tf_cmd)

    return ld
