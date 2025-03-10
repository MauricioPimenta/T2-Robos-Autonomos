import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np

class DynamicMapper(Node):
    def __init__(self):
        super().__init__('dynamic_mapper')
        
        # Declare and get map resolution parameter
        self.declare_parameter('map_resolution', 0.05)  # Default to 0.05 meters per cell
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        
        # Occupancy grid map setup
        self.map_data = np.full((100, 100), -1, dtype=np.int8)  # Initial unknown map (dynamic resizing later)
        self.origin = [0.0, 0.0]  # Robot starts at the origin
        
        # Publishers and subscribers
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Dynamic mapping node has started.')

    def scan_callback(self, msg: LaserScan):
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                # Calculate coordinates of the obstacle
                obs_x = int((r * np.cos(angle) - self.origin[0]) / self.map_resolution)
                obs_y = int((r * np.sin(angle) - self.origin[1]) / self.map_resolution)
                self.expand_map(obs_x, obs_y)
                
                # Mark free cells (between robot and obstacle)
                self.mark_free_cells(obs_x, obs_y)
                
                # Mark the obstacle cell as occupied
                self.map_data[obs_y, obs_x] = 100
            angle += msg.angle_increment

    def expand_map(self, x, y):
        rows, cols = self.map_data.shape
        if x >= cols or y >= rows or x < 0 or y < 0:
            new_rows = max(rows, y + 1)
            new_cols = max(cols, x + 1)
            new_map = np.full((new_rows, new_cols), -1, dtype=np.int8)
            new_map[:rows, :cols] = self.map_data
            self.map_data = new_map

    def mark_free_cells(self, obs_x, obs_y):
        robot_x, robot_y = int(self.origin[0] / self.map_resolution), int(self.origin[1] / self.map_resolution)
        num_points = max(abs(obs_x - robot_x), abs(obs_y - robot_y))
        for i in range(num_points):
            x = int(robot_x + i * (obs_x - robot_x) / num_points)
            y = int(robot_y + i * (obs_y - robot_y) / num_points)
            if 0 <= y < self.map_data.shape[0] and 0 <= x < self.map_data.shape[1]:
                if self.map_data[y, x] == -1:  # Only update unknown cells
                    self.map_data[y, x] = 0  # Free space

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_data.shape[1]
        map_msg.info.height = self.map_data.shape[0]
        map_msg.info.origin = Pose()  # Default origin at (0,0)
        map_msg.data = self.map_data.flatten().tolist()
        self.map_publisher.publish(map_msg)
        self.get_logger().info('Map published.')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Shutting down...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
