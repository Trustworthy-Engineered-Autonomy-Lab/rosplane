import os 
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the package directory
    rosplane_gcs_dir = get_package_share_directory('rosplane_gcs')

    rviz2_config_file = rosplane_gcs_dir + '/config/rosplane_gcs.rviz'
    rviz2_splash_file = rosplane_gcs_dir + '/resource/logo.png'

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x','0','--y','0','--z','0','--yaw','0','--pitch','0','--roll','3.1415926535','--frame-id','world','--child-frame-id','NED']
        ),
        Node(
            package='rosplane_gcs',
            executable='rviz_waypoint_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d',rviz2_config_file, '-s', rviz2_splash_file]
        )
    ])