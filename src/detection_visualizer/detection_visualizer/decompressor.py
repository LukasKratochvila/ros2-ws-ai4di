#!/usr/bin/env python3
#from _future_ import print_function
# https://github.com/opencv/opencv/issues/18945
import sys, time
import cv2
import numpy as np
#import roslib
#import rospy
import rclpy
from rclpy.node import Node
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
#from std_msgs.msg import String, Header
#from std_msgs.msg import Float32MultiArray
#from transforms3d.euler import euler2mat, mat2euler
#from transforms3d.axangles import axangle2mat

class Decompressor(Node):
   # Try to tune the queue_size value. Maybe increase it up to the frequency.	
   def __init__(self):
      super().__init__('decompressor')
      '''Initialize ros publisher, ros subscriber'''
      
      self.declare_parameter("input_topic", '/image/compressed')
      self.declare_parameter("output_topic", '~/raw')
      self.declare_parameter("debug", True)

      self.input_topic = self.get_parameter("input_topic")._value
      self.output_topic = self.get_parameter("output_topic")._value
      self.debug = self.get_parameter("debug")._value
      
      # topic where we publish
      self.image_pub  = self.create_publisher(Image, self.output_topic, 10)
      self.bridge = CvBridge()
      # subscribed Topic
      self.subscriber = self.create_subscription(CompressedImage, self.input_topic, self.callback, 10)
      self.get_logger().info("Decompressor has started.")
		
   def callback(self, ros_data):
      if self.debug:
         self.get_logger().info("Recieved compressedImage.")

      image_np = self.bridge.compressed_imgmsg_to_cv2(ros_data,"bgr8")

      out_msg = self.bridge.cv2_to_imgmsg(image_np,"bgr8")
      out_msg.header = ros_data.header
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_np,"rgb8"))
      self.image_pub.publish(out_msg)	
      if self.debug:
            self.get_logger().info("Publishing Image")	
		
#class passthrough_class:
#   def __init__(self):
#      self.info_pub  = rospy.Publisher("/output/camera_info", CameraInfo,queue_size=1)
#      self.info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo,self.callback,queue_size=1)
#   def callback(self,data):
#      self.info_pub.publish(data)

def main(): #args
   '''Initializes and cleanup ros node'''
   rclpy.init()
   rclpy.spin(Decompressor())
   rclpy.shutdown()
   #rospy.init_node('Decompressor', anonymous=True)
   #ic = Decompressor()
   #pt = passthrough_class()
   #try:
   #   rospy.spin()
   #except KeyboardInterrupt:
   #   print("Shutting down ROS Image converter module")
   #   #cv2.destroyAllWindows()

if __name__ == '__main__':
   main() #sys.argv
