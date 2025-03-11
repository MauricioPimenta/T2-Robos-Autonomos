import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import numpy as np
import random
import math
import os
import cv2
import tf_transformations  # Certifique-se de ter instalado o pacote tf-transformations

class FastSLAMMapper(Node):
    def __init__(self):
        super().__init__('fastslam_mapper')
        
        # Declare and get map resolution parameter
        self.declare_parameter('map_resolution', 0.05)
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        
        # Particle filter setup
        self.num_particles = 100
        self.particles = np.array([[0.0, 0.0, 0.0] for _ in range(self.num_particles)])  # [x, y, yaw]
        self.weights = np.ones(self.num_particles) / self.num_particles
        
        # Occupancy grid map setup
        self.map_size = 200  # Start with a larger default size
        self.map_data = np.full((self.map_size, self.map_size), -1, dtype=np.int8)  # Unknown (-1)
        self.robot_position = [self.map_size // 2, self.map_size // 2]  # Start in center
        self.detected_obstacles = []  # List to store detected obstacles
        
        # Publishers and subscribers
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info('FastSLAM mapping node with localization and map-based obstacle detection has started.')

    

    def odom_callback(self, msg: Odometry):
        # Extraindo a posição (em metros)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convertendo para índices do mapa
        self.robot_position[0] = int(x / self.map_resolution) + self.map_size // 2
        self.robot_position[1] = int(y / self.map_resolution) + self.map_size // 2

        # Extraindo o quaternion da mensagem
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]

        # Convertendo o quaternion para ângulos de Euler (roll, pitch, yaw)
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(quaternion)
        self.robot_theta = yaw  # Armazena o yaw para uso nas transformações dos dados do sensor

        self.get_logger().info(f"Odom atualizado: pos=({x:.2f}, {y:.2f}), theta={self.robot_theta:.2f} rad")

        
    def scan_callback(self, msg: LaserScan):
        # Utilize a pose do robô baseada na odometria (com a orientação apropriada)
        # Suponha que você também armazene a orientação (theta) vinda da odometria
        robot_x = (self.robot_position[0] - self.map_size // 2) * self.map_resolution
        robot_y = (self.robot_position[1] - self.map_size // 2) * self.map_resolution
        robot_theta = self.robot_theta

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                obs_x = int(((robot_x + r * np.cos(angle + robot_theta)) / self.map_resolution)) + self.map_size // 2
                obs_y = int(((robot_y + r * np.sin(angle + robot_theta)) / self.map_resolution)) + self.map_size // 2
                
                self.expand_map(obs_x, obs_y)
                self.mark_free_cells(self.robot_position[0], self.robot_position[1], obs_x, obs_y)
                self.map_data[obs_y, obs_x] = 100
                self.get_logger().info(f"Obstacle marked at ({obs_x}, {obs_y})")
            angle += msg.angle_increment
        
        self.detect_obstacles_from_map()
        self.update_particle_weights(msg)



    def detect_obstacles_from_map(self):
        # Convert occupancy grid to binary image for contour detection
        binary_map = (self.map_data == 100).astype(np.uint8) * 255
        contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(cnt)
            center_x = x + w // 2
            center_y = y + h // 2

            if not self.is_duplicate_obstacle(center_x, center_y):
                shape = "Circular" if len(approx) > 6 else "Square"
                self.detected_obstacles.append((center_x, center_y, shape))
                self.save_obstacle_info()
                self.get_logger().info(f'Obstacle detected: ({center_x}, {center_y}) Shape: {shape}')

    def expand_map(self, x, y):
        rows, cols = self.map_data.shape
        if x >= cols or y >= rows or x < 0 or y < 0:
            new_size = max(rows, cols, x + 50, y + 50)  # Expand by a buffer
            new_map = np.full((new_size, new_size), -1, dtype=np.int8)
            offset = (new_size - self.map_size) // 2
            new_map[offset:offset+self.map_size, offset:offset+self.map_size] = self.map_data
            self.map_data = new_map
            self.map_size = new_size
            self.robot_position = [self.robot_position[0] + offset, self.robot_position[1] + offset]
            self.get_logger().info(f"Map expanded to size {new_size}x{new_size}")


    def mark_free_cells(self, rx, ry, obs_x, obs_y):
        num_points = max(abs(obs_x - int(rx)), abs(obs_y - int(ry)))
        for i in range(num_points):
            x = int(rx + i * (obs_x - rx) / num_points)
            y = int(ry + i * (obs_y - ry) / num_points)
            if 0 <= y < self.map_data.shape[0] and 0 <= x < self.map_data.shape[1]:
                if self.map_data[y, x] == -1:
                    self.map_data[y, x] = 0  # Mark as free space



    def is_duplicate_obstacle(self, x, y):
        for ox, oy, _ in self.detected_obstacles:
            if abs(ox - x) < 5 and abs(oy - y) < 5:
                return True
        return False

    def save_obstacle_info(self):
        with open("detected_obstacles.txt", "w") as f:
            for obs in self.detected_obstacles:
                f.write(f"{obs[0]}, {obs[1]}, {obs[2]}\n")

    def update_particle_weights(self, scan_msg):
        for i in range(self.num_particles):
            weight = 1.0
            angle = scan_msg.angle_min
            for r in scan_msg.ranges:
                if scan_msg.range_min <= r <= scan_msg.range_max:
                    expected_x = int((self.particles[i, 0] + r * np.cos(angle + self.particles[i, 2])) / self.map_resolution)
                    expected_y = int((self.particles[i, 1] + r * np.sin(angle + self.particles[i, 2])) / self.map_resolution)
                    if 0 <= expected_y < self.map_data.shape[0] and 0 <= expected_x < self.map_data.shape[1]:
                        if self.map_data[expected_y, expected_x] == 100:
                            weight *= 1.2  # Obstacle detected where expected
                        else:
                            weight *= 0.8  # No obstacle where expected
                angle += scan_msg.angle_increment
            self.weights[i] = weight
        
        self.weights += 1e-300  # Avoid zero weights
        self.weights /= np.sum(self.weights)  # Normalize
        self.resample_particles()

    def resample_particles(self):
        indices = np.random.choice(self.num_particles, self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_data.shape[1]
        map_msg.info.height = self.map_data.shape[0]
        map_msg.info.origin = Pose()
        map_msg.data = self.map_data.flatten().tolist()
        self.map_publisher.publish(map_msg)
        self.get_logger().info('Map published.')


def main(args=None):
    rclpy.init(args=args)
    node = FastSLAMMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()