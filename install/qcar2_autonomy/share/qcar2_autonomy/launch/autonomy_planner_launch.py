import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    yolo_detector= Node(
        package ='qcar2_autonomy',
        executable ='yolo_detector',
        name ='yolo_detector'
    )
        
    lane_detector = Node(
        package='qcar2_autonomy',
        executable='lane_detector', # Implementation based on your project goals
        name='lane_detector'
    )


    return LaunchDescription([
        yolo_detector,
        lane_detector
        ]
    )