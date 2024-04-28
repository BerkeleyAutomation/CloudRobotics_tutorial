import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from matplotlib import pyplot as plt

def show_anns(masks):
    if len(masks) == 0:
        return
    ax = plt.gca()
    ax.set_autoscale_on(False)
    img = np.ones((masks[0].shape[0], masks[0].shape[1], 4))
    img[:, :, 3] = 0
    for mask in masks:
        m = np.all(mask == [1, 1, 1], axis=-1)
        color_mask = np.concatenate([np.random.random(3), [0.35]])
        img[m] = color_mask
    ax.imshow(img)
    plt.show()

class SAMSubscriberNode(Node):
    def __init__(self):
        super().__init__('sam_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'segmentation_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received segmentation mask')
        # Convert CompressedImage data to an image array
        mask_array = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        # You might have multiple masks in a single message, handle accordingly
        # Here we assume one mask per message for simplicity
        masks = [mask_array]
        show_anns(masks)

def main(args=None):
    rclpy.init(args=args)
    sam_subscriber = SAMSubscriberNode()
    rclpy.spin(sam_subscriber)
    sam_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
