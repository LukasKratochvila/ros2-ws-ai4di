import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os

class ImgPublisher(Node):
      def __init__(self):
         super().__init__('img_publisher')

         self.declare_parameter("input_dir", "/home/kratochvila/Desktop/mereni3/image")
         self.declare_parameter("output_topic", "/image")
         self.declare_parameter("frame_id", "image")
         self.declare_parameter("frequency", 2.0)
         self.declare_parameter("start_counter", 0)
         self.declare_parameter("file_ext", "jpg")
         self.declare_parameter("debug", False)
         
         self.freq = self.get_parameter("frequency")._value
         self.timer = self.create_timer(1/self.freq, self.timer_callback)
         
         self.dir = self.get_parameter("input_dir")._value
         self.file_ext = self.get_parameter("file_ext")._value
         self.counter = self.get_parameter("start_counter")._value

         self.publisher = self.create_publisher(Image, self.get_parameter("output_topic")._value, 10)
         self.frame_id = self.get_parameter("frame_id")._value

         self.debug = self.get_parameter("debug")._value
         
         if not os.path.isdir(self.dir):
            self.get_logger().error("Input dir: {} is not directory or doesn't exist!".format(self.dir))
         else:
            self.files = [file for file in os.listdir(self.dir) if file.split(".")[-1] == "jpg"]
            self.files.sort()
            if self.debug:
               self.get_logger().info("Read directory {} with {} files.".format(self.dir, len(self.files)))

         self.bridge = CvBridge()
         self.get_logger().info("Pcl_pub_py has started.")

      def timer_callback(self):
          if self.counter > len(self.files)-1:
            self.counter = 0
          self.cv_image = cv2.imread(os.path.join(self.dir,self.files[self.counter]))

          msg = self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8")
          msg.header.frame_id = self.frame_id
          msg.header.stamp.sec = self.counter
          self.publisher.publish(msg)
          if self.debug:
            self.get_logger().info('Publishing an image from file {}'.format(self.files[self.counter]))
          self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    img_publisher = ImgPublisher()
    rclpy.spin(img_publisher)
    img_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()