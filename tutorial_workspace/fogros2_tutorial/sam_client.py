import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import os
from cloudgripper_msgs.srv import GetCameraImage  # Adjust this to your actual service definition
from cv_bridge import CvBridge

class SegmentAnythingClient(Node):
    def __init__(self):
        super().__init__('sam_subscriber')
        self.publisher = self.create_publisher(CompressedImage, '/image', 2)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image/segmented',
            self.listener_callback,
            2)
        
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(10.0, self.publish_image)  # Adjust the timer as needed

        self.client = self.create_client(GetCameraImage, '/get_camera_image')
        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Service not available, waiting again...')

    def request_and_publish_image(self, camera_type):
        req = GetCameraImage.Request()
        req.camera_type = camera_type
        future = self.client.call_async(req)
        future.add_done_callback(lambda future: self.handle_response(future, camera_type))

    def handle_response(self, future, camera_type):
        try:
            response = future.result()
            if response.success:
                # Convert the received ROS image message to an OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(response.image, desired_encoding='bgr8')
                # Convert the OpenCV image back to a ROS image message for publishing
                # ros_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                # Publish the image on the appropriate topic
                if camera_type == 'base':
                    msg = CompressedImage()
                    msg.format = 'jpeg'
                    msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
                    self.publisher.publish(msg)
                    self.get_logger().info('Base camera image published.')
                elif camera_type == 'top':
                    msg = CompressedImage()
                    msg.format = 'jpeg'
                    msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
                    self.publisher.publish(msg)
                    self.get_logger().info('Top camera image published.')
            else:
                self.get_logger().info(f'Failed to receive image from {camera_type} camera: ' + response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def publish_image(self):
        self.request_and_publish_image('base')
        # check if image exists 
        # if not os.path.exists('/home/ubuntu/image.jpg'):
        #     # download 
        #     os.system('wget -O /home/ubuntu/image.jpg https://github.com/BerkeleyAutomation/fogros-realtime-examples/blob/main/images/happy-bear.png?raw=true')
        
        # self.get_logger().info('Publishing image')
        # # Load an image from disk
        # image = cv2.imread('/home/ubuntu/image.jpg')
        # # Compress the image
        msg = CompressedImage()
        msg.format = 'jpeg'
        # msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        # self.publisher.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info('Received segmented image!')
        # Convert CompressedImage data to an image array

def main(args=None):
    rclpy.init(args=args)
    sam_subscriber = SegmentAnythingClient()
    rclpy.spin(sam_subscriber)
    sam_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
