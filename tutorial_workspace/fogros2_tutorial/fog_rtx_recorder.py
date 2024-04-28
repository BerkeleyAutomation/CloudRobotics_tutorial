import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import fog_x 

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'compressed_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.dataset = fog_x.Dataset(
                name="demo_ds",
                path="~/test_dataset", # can be AWS S3, Google Bucket! 
            )  
        

    def listener_callback(self, msg):
        self.episode = self.dataset.new_episode()
        self.get_logger().info('Receiving compressed image')

        # Convert the data from the CompressedImage message to a numpy array
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)

        # Decode the image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.episode.add(feature = "image", value = image)
        self.episode.close()
 

def main(args=None):
    rclpy.init(args=args)
    compressed_image_subscriber = CompressedImageSubscriber()
    rclpy.spin(compressed_image_subscriber)
    compressed_image_subscriber.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
