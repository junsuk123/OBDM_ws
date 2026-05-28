#!/usr/bin/env python3
"""
Simple Iris launcher for OBDM - Gazebo + ArduCopter + ROS2
Simplified approach without robot_state_publisher issues
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Iris quadcopter in Gazebo."""
    
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_ardupilot_bringup = get_package_share_directory("ardupilot_gz_bringup")
    
    # Get repository root (this launch file location)
    launch_file_dir = Path(__file__).parent
    repo_root = launch_file_dir.parent.parent
    
    # Gazebo server
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_ardupilot_gazebo) / "worlds" / "iris_runway.sdf"}'
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_server")),
    )

    # Gazebo GUI
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_gui")),
    )

    # Robot description publisher (publishes SDF as ROS parameter)
    sdf_file = str(Path(pkg_ardupilot_gazebo) / "models" / "iris_with_standoffs" / "model.sdf")
    with open(sdf_file) as f:
        robot_desc_content = f.read()
    
    # Fix package:// URIs in SDF to absolute paths
    pkg_ardupilot_gazebo_str = str(Path(pkg_ardupilot_gazebo).parent)
    robot_desc_content = robot_desc_content.replace(
        "package://ardupilot_gazebo/",
        f"file://{pkg_ardupilot_gazebo_str}/ardupilot_gazebo/"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="iris",
        parameters=[{"robot_description": robot_desc_content}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    # ArduCopter SITL
    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_ardupilot_sitl) / "launch" / "sitl_dds_udp.launch.py")
        ),
        launch_arguments={
            "command": "arducopter",
            "wipe": "False",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "model": "json",
            "synthetic_clock": "True",
            "defaults": ",".join([
                str(Path(pkg_ardupilot_sitl) / "config" / "default_params" / "copter.parm"),
                str(Path(pkg_ardupilot_gazebo) / "config" / "gazebo-iris-gimbal.parm"),
                str(Path(pkg_ardupilot_sitl) / "config" / "default_params" / "dds_udp.parm"),
                str(Path(pkg_ardupilot_sitl) / "config" / "default_params" / "dds_use_ns.parm"),
            ]),
            "sim_address": "127.0.0.1",
            "micro_ros_agent_ns": "iris",
            "transport": "udp4",
            "port": "2019",
        }.items(),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    # Parameter bridge (connects Gazebo to ROS2)
    bridge_config = str(Path(pkg_ardupilot_bringup) / "config" / "iris_bridge.yaml")
    
    # Read bridge config and replace placeholders
    with open(bridge_config) as f:
        bridge_config_content = f.read()
    
    # Replace placeholders with actual values
    bridge_config_content = bridge_config_content.replace(
        "{{ world_name }}", "iris_runway"
    ).replace(
        "{{ robot_name }}", "iris"
    )
    
    # Write to temp file
    import tempfile
    temp_bridge_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False
    )
    temp_bridge_file.write(bridge_config_content)
    temp_bridge_file.close()
    bridge_config_file = temp_bridge_file.name

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace="iris",
        parameters=[{"config_file": bridge_config_file}],
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    # Gazebo model spawn (using parameter server, not topic)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        namespace="iris",
        arguments=[
            "-world", "",
            "-param", "robot_description",  # Use parameter, not topic
            "-name", "iris",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.2",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "1.5707963267948966",
        ],
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_gz_sim_server", default_value="true"),
        DeclareLaunchArgument("use_gz_sim_gui", default_value="true"),
        DeclareLaunchArgument("spawn_robot", default_value="true"),
        gz_sim_server,
        gz_sim_gui,
        robot_state_publisher,
        bridge,
        spawn_robot,
        sitl,
    ])
