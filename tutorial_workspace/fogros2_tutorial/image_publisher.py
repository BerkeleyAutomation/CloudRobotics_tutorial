import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'compressed_image', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # Generate a random image (480x640 with 3 channels)
        random_image = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)

        # Encode the numpy array to PNG
        retval, buffer = cv2.imencode('.png', random_image)
        if retval:
            # Create a CompressedImage message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.format = "png"
            compressed_image_msg.data = np.array(buffer).tobytes()

            # Publish the compressed image
            self.publisher_.publish(compressed_image_msg)
            self.get_logger().info('Publishing compressed image')

def main(args=None):
    rclpy.init(args=args)
    compressed_image_publisher = CompressedImagePublisher()
    rclpy.spin(compressed_image_publisher)
    compressed_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
