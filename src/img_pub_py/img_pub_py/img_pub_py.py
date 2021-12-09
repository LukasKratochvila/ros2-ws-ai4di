# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

 
class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')

    self.declare_parameter("output_topic", "/image")
    self.declare_parameter("frame_id", "image")
    self.declare_parameter("frequency", 2.0)
    self.declare_parameter("encoding", "bgr8")
    self.declare_parameter("debug", False)
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, self.get_parameter("output_topic")._value, 10)
      
    # We will publish a message every 0.1 seconds
    self.frame_id = self.get_parameter("frame_id")._value
    self.freq = self.get_parameter("frequency")._value
    self.encoding = self.get_parameter("encoding")._value
    self.debug = self.get_parameter("debug")._value
      
    # Create the timer
    self.timer = self.create_timer(1/self.freq, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    self.get_logger().info("Image_publisher has started.")
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
          
    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      msg = self.br.cv2_to_imgmsg(frame,self.encoding)
      msg.header.frame_id = self.frame_id
      self.publisher_.publish(msg)
 
    # Display the message on the console
    if self.debug:
      self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()