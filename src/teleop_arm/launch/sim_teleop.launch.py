# teleop_arm/launch/sim_teleop.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_simulation_gazebo'),
                'launch',
                'ur_sim_control.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': 'ur5',
            'use_fake_hardware': 'true',
            'launch_rviz': 'false'
        }.items()
    )

    return LaunchDescription([
        ur_launch
    ])
