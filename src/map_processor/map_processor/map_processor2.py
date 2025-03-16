from typing import List
import rclpy

import cv2
import numpy as np
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from turtlebot3_mapper.utils import occupancy_grid_to_numpy

class MapProcessorNode(Node):
    def parameter_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        for param in parameters:
            if param.name == "threshold":
                self.threshold = param.value
                self.get_logger().info("Updated parameter threshold=%.2f" % self.threshold)
            elif param.name == "connectivity":
                self.connectivity = param.value
                self.get_logger().info("Updated parameter connectivity=%.2f" % self.connectivity)
            elif param.name == "min_area":
                self.min_area = param.value
                self.get_logger().info("Updated parameter min_area=%.2f" % self.min_area)
            elif param.name == "max_area":
                self.max_area = param.value
                self.get_logger().info("Updated parameter max_area=%.2f" % self.max_area)
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def __init__(self, node_name: str = 'map_processor2'):
        super().__init__(node_name=node_name)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("threshold", 150, ParameterDescriptor(description="Threshold to binary image")),
                ("connectivity", 4, ParameterDescriptor(description="4-way or 8-way connectivity")),
                ("min_area", 30, ParameterDescriptor(description="Min area of each connected component")),
                ("max_area", 1000, ParameterDescriptor(description="Max area of each connected component")),
            ],
        )
        self.threshold = float(self.get_parameter("threshold").value)
        self.connectivity = float(self.get_parameter("connectivity").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.max_area = float(self.get_parameter("max_area").value)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self._map_subscriber = self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/custom_map",
            callback=self._map_callback,
            qos_profile=10,
        )
        self._detections_publisher = self.create_publisher(
            msg_type=BoundingBox2DArray,
            topic="/detections",
            qos_profile=10,
        )
        self._result_publisher = self.create_publisher(
            msg_type=Image,
            topic="/rendered",
            qos_profile=10,
        )
        self._bridge = CvBridge()
        self.get_logger().info(f"Init {node_name}")

    def _map_callback(self, message: OccupancyGrid):
        array = occupancy_grid_to_numpy(message)
        input = array.astype("float32")
        input = 255 * (input - np.min(input)) / (np.max(input) - np.min(input))
        input[input <= self.threshold] = 0
        input[input > self.threshold] = 255
        input = input.astype('uint8')
        
        blurred = cv2.GaussianBlur(input, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = BoundingBox2DArray()
        detections.header = message.header
        output = cv2.cvtColor(input, cv2.COLOR_GRAY2BGR)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)
            
            if self.min_area < area < self.max_area:
                x, y, w, h = cv2.boundingRect(cnt)
                center = (int(x + w / 2), int(y + h / 2))
                
                bbox = BoundingBox2D()
                bbox.center.position.x = float(center[0])
                bbox.center.position.y = float(center[1])
                bbox.size_x = float(w)
                bbox.size_y = float(h)
                
                if len(approx) > 8:
                    object_type = "sphere"
                    color = (0, 0, 255)
                else:
                    object_type = "box"
                    color = (0, 255, 0)
                
                detections.boxes.append(bbox)
                cv2.rectangle(output, (x, y), (x + w, y + h), color, 1)
                cv2.circle(output, center, 3, (255, 0, 0), -1)
                
                self.get_logger().info(f"Detected {object_type} at {center}")
                
        self._detections_publisher.publish(detections)
        image_ros = self._bridge.cv2_to_imgmsg(np.flip(output, axis=0))
        image_ros.header = message.header
        self._result_publisher.publish(image_ros)

def main(args=None):
    rclpy.init(args=args)
    node = MapProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
