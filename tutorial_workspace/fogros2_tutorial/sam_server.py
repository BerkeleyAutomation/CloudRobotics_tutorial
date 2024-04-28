import random
import socket
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry
from sensor_msgs.msg import CompressedImage
from matplotlib import pyplot as plt

def show_anns(anns):
    if len(anns) == 0:
        return
    sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
    ax = plt.gca()
    ax.set_autoscale_on(False)

    img = np.ones((sorted_anns[0]['segmentation'].shape[0], sorted_anns[0]['segmentation'].shape[1], 4))
    img[:,:,3] = 0
    for ann in sorted_anns:
        m = ann['segmentation']
        color_mask = np.concatenate([np.random.random(3), [1]])
        img[m] = color_mask
    plt.savefig("/home/ubuntu/mask.png")
    return img 


class SegmentAnythingServer(Node):
    def __init__(self):
        super().__init__("sam_publisher")
        # generate random name
        self.host_name = socket.gethostname() + str(random.randint(0, 1000))
        self.get_logger().info(f'I am {self.host_name}. Starting publisher on /image/segmented.')

        self.publisher_ = self.create_publisher(CompressedImage, '/image/segmented', 2)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image',
            self.listener_callback,
            2)

        # Load the SAM model
        self.get_logger().info('Loading SAM model...')
        model_path = "/home/ubuntu/models/sam_vit_h_4b8939.pth" 
        self.sam = sam_model_registry["default"](checkpoint=model_path)
        self.sam.to(device="cuda")  # Ensure CUDA is available, otherwise specify 'cpu'
        self.sam_mask_generator = SamAutomaticMaskGenerator(self.sam)
        self.get_logger().info('SAM model loaded.')

    def listener_callback(self, msg):
        self.get_logger().info('Received image')
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        self.get_logger().info('Generating masks...')
        masks_output = self.sam_mask_generator.generate(image)
        img = show_anns(masks_output)
        msg = CompressedImage()
        msg.format = "png"
        img = np.clip(img * 255, 0, 255).astype(np.uint8)
        msg.data = cv2.imencode('.png', img)[1].tobytes()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sam_publisher_node = SegmentAnythingServer()
    rclpy.spin(sam_publisher_node)
    
    sam_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
