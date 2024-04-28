import random
import socket
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry
from sensor_msgs.msg import CompressedImage

class SamPublisherNode(Node):
    def __init__(self):
        super().__init__("sam_publisher")
        # generate random name
        self.host_name = socket.gethostname() + str(random.randint(0, 1000))
        self.get_logger().info(f'I am {self.host_name}. Starting publisher on /segmentation_data.')

        self.publisher_ = self.create_publisher(CompressedImage, 'segmentation_data', 10)
        self.timer = self.create_timer(2.0, self.publish_masks)  # Adjust the timer as needed

        # Load the SAM model
        model_path = "/home/simeonoa/sam_ws/models/sam_vit_h_4b8939.pth" # Replace with path to your model
        self.sam = sam_model_registry["default"](checkpoint=model_path)
        self.sam.to(device="cuda")  # Ensure CUDA is available, otherwise specify 'cpu'
        self.sam_mask_generator = SamAutomaticMaskGenerator(self.sam)

    def publish_masks(self):
        # For demonstration, we use a dummy image. Replace with actual image acquisition logic
        dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.get_logger().info('Generating masks...')
        masks_output = self.sam_mask_generator.generate(dummy_image)

        for m in masks_output:
            msg = CompressedImage()
            msg.format = "png"
            msg.data = np.array(cv2.imencode('.png', m['segmentation'].astype(np.uint8))[1]).tobytes()
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published mask with stability score {m["stability_score"]}')

def main(args=None):
    rclpy.init(args=args)
    sam_publisher_node = SamPublisherNode()
    rclpy.spin(sam_publisher_node)
    
    sam_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
