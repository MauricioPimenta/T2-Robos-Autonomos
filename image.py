import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/map_image', 10)
        self.timer = self.create_timer(1.0, self.publish_image)
        self.bridge = CvBridge()

    def publish_image(self):
        img = cv2.imread('/root/workspace/mapateste.pgm', cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error('Erro ao carregar a imagem!')
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
        self.publisher.publish(msg)
        self.get_logger().info('Imagem publicada!')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
