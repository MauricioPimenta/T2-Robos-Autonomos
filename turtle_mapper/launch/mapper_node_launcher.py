from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    return LaunchDescription([
        # Launch the TurtleBot3 world in Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py'))
        ),
        
        # Node for dynamic mapping
        Node(
            package='your_package_name',
            executable='dynamic_mapper',
            name='dynamic_mapper',
            parameters=[{'map_resolution': 0.05}]
        ),
        
        # Launch RViz to visualize the map
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),

        # Node to visualize occupancy grid map in Gazebo
        Node(
            package='grid_map_ros',
            executable='grid_map_visualization',
            name='grid_map_visualization',
            parameters=[
                {'grid_map_topic': '/map'},
                {'visualization_type': 'occupancy_grid'}
            ]
        )
    ])
