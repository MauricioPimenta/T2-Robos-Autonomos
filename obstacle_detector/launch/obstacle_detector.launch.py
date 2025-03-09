from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import random

def generate_launch_description():
    # Generar coordenadas aleatorias para la posición inicial del robot
    x = random.uniform(-2.0, 2.0)
    y = random.uniform(-2.0, 2.0)
    yaw = random.uniform(0, 6.28)  # Ángulo aleatorio en radianes

    # Obtener la ruta del launch de turtlebot3_gazebo
    turtlebot3_gazebo_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_world.launch.py'
    )

    return LaunchDescription([
        # Incluir el launch de Gazebo correctamente
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_gazebo_launch_file),
            launch_arguments={
                'x_pose': str(x),
                'y_pose': str(y),
                'yaw': str(yaw)
            }.items()
        ),
        # Lanzar el nodo de detección de obstáculos
        Node(
            package='obstacle_detector',
            executable='obstacle_detector_node',
            output='screen',
        ),
    ])
