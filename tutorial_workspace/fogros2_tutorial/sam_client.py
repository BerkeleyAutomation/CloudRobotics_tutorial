import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import os


class SegmentAnythingClient(Node):
    def __init__(self):
        super().__init__('sam_subscriber')
        self.publisher = self.create_publisher(CompressedImage, '/image', 10)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image/segmented',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(2.0, self.publish_image)  # Adjust the timer as needed

    def publish_image(self):
        # check if image exists 
        if not os.path.exists('/home/ubuntu/image.jpg'):
            # download 
            os.system('wget -O /home/ubuntu/image.jpg https://letsenhance.io/static/8f5e523ee6b2479e26ecc91b9c25261e/1015f/MainAfter.jpg')
        
        self.get_logger().info('Publishing image')
        # Load an image from disk
        image = cv2.imread('/home/ubuntu/image.jpg')
        # Compress the image
        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info('Received segmentation mask')
        # Convert CompressedImage data to an image array
        mask_array = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        # You might have multiple masks in a single message, handle accordingly
        # Here we assume one mask per message for simplicity
        masks = [mask_array]
        
def main(args=None):
    rclpy.init(args=args)
    sam_subscriber = SegmentAnythingClient()
    rclpy.spin(sam_subscriber)
    sam_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
