import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge

class MapProcessorNode(Node):
    def __init__(self):
        super().__init__('map_processor')
        self.subscription = self.create_subscription(
            Image,
            '/map_image',
            self.map_callback,
            10)
        self.publisher = self.create_publisher(String, '/detected_objects', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Map Processor Node has started.')
    
    def map_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        detected_objects = self.detect_objects(cv_image)
        self.publisher.publish(String(data=str(detected_objects)))
    
    def detect_objects(self, image):
        objects = []
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)
            if area > 100:
                x, y, w, h = cv2.boundingRect(cnt)
                center = (int(x + w / 2), int(y + h / 2))
                
                if len(approx) > 8:
                    objects.append({'type': 'sphere', 'position': center})
                else:
                    objects.append({'type': 'box', 'position': center})
        
        return objects

def main(args=None):
    rclpy.init(args=args)
    node = MapProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
