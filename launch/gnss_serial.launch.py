#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "portname",
            default_value="/dev/gnss_device",
            description="Serial port name for the microcontroller",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Baudrate for serial communication (GNSS typically uses 115200)",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="10.0",
            description="Publishing rate in Hz (GNSS typically 1-10 Hz)",
        )
    )

    # Initialize Arguments
    portname = LaunchConfiguration("portname")
    baudrate = LaunchConfiguration("baudrate")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    # GNSS Hardware Node
    gnss_serial_node = Node(
        package="gnss_serial",
        executable="gnss_serial",
        name="gnss_serial_node",
        output="screen",
        parameters=[
            {
                "portname": portname,
                "baudrate": baudrate,
                "publish_rate_hz": publish_rate_hz,
            }
        ],
    )

    nodes = [
        gnss_serial_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
