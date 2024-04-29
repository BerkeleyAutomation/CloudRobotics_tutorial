import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import fog_x 
import functools

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')

        self.dataset_name = self.declare_parameter('robot_name', 'PARAMETER NOT SET').get_parameter_value().string_value
        if self.dataset_name == 'PARAMETER NOT SET':
            raise ValueError('Please set the dataset_name parameter')
        self.declare_parameter("max_idle_time", 10)
        self.max_idle_time = self.get_parameter("max_idle_time").value

        self.dataset = fog_x.Dataset(
                name=self.dataset_name,
                path="s3://cloud-robotics-workshop", # can be AWS S3, Google Bucket! 
            )  
        
        image_topics = [
            "/image/base",
            "/image/top",
            "/image/segmented",
        ]
        for topic_name in image_topics:
            self.create_subscription(
                CompressedImage,
                topic_name,
                self.create_dynamic_image_callback(topic_name),
                10
            )
        
        # this detects the activeness of commands 
        # timeout and create a new episode if no command is received for 10 seconds
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.episode_timeout_timer = self.create_timer(1, self.episode_timeout_callback)
        self.episode = None 
        self.time_since_last_cmd_vel = 0
        
    def cmd_vel_callback(self, msg):
        if self.episode is None:
            self.episode = self.dataset.new_episode()
            self.get_logger().info(f"Created new episode")
        self.time_since_last_cmd_vel = 0
    
    def episode_timeout_callback(self):
        self.time_since_last_cmd_vel += 1
        if self.time_since_last_cmd_vel > self.max_idle_time and self.episode is not None:
            self.episode.close()
            self.get_logger().info(f"Closed episode")
            self.episode = None

    def create_dynamic_image_callback(self, topic_name):
        def image_callback(self, msg):
            if self.episode is None:
                return 
            self.get_logger().info(f"Received image on {topic_name}")
            # Convert the data from the CompressedImage message to a numpy array
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            # Decode the image
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.episode.add(feature = topic_name, value = image)
    
        return functools.partial(image_callback, self)

    def destroy_node(self):
        self.episode.close()
        super().destroy_node()
 

def main(args=None):
    rclpy.init(args=args)
    compressed_image_subscriber = CompressedImageSubscriber()
    rclpy.spin(compressed_image_subscriber)
    compressed_image_subscriber.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
