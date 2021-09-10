import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os

class MinimalPublisher(Node):
      def __init__(self):
         super().__init__('minimal_publisher')
         self.publisher_ = self.create_publisher(Image, 'image', 10)
         timer_period = 2#1/10 # seconds
         self.timer = self.create_timer(timer_period, self.timer_callback)
         self._counter = 0
         self._dir = "/home/kratochvila/Desktop/mereni3/image"#"/home/kratochvila/Desktop/lidar/mereni2/image"
         self._files = [file for file in os.listdir(self._dir) if file.split(".")[-1] == "jpg"]
         self._files.sort()
         #self.cv_image = cv2.imread('test.jpeg') ### an RGB image 
         self.bridge = CvBridge()

      def timer_callback(self):
          if self._counter > len(self._files)-1:
              self._counter = 0
          self.cv_image = cv2.imread(os.path.join(self._dir,self._files[self._counter]))

          msg = self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8")
          msg.header.frame_id = "map"
          msg.header.stamp.sec = self._counter
          self.publisher_.publish(msg)
          self.get_logger().info('Publishing an image from file {}'.format(self._files[self._counter]))
          self._counter += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()