import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection2D, Detection3D, ObjectHypothesisWithPose
#from visualization_msgs.msg import MarkerArray, Marker

class DetectionMatcherTester(Node):
   def __init__(self):
      super().__init__('detection_matcher_tester')
      self.declare_parameter("OutputProject2D_topic", "/projection")
      self.declare_parameter("OutputDetect2D_topic", "/detector_node/detections")
      self.declare_parameter("OutputDetect3D_topic", "/detections")
      self.declare_parameter("frequency", 2.0)
      self.declare_parameter("start_counter", 0)
      self.declare_parameter("frame_id", "test")
      self.declare_parameter("debug", True)     

      self.Project2D_topic = self.get_parameter("OutputProject2D_topic")._value
      self.Detect2D_topic = self.get_parameter("OutputDetect2D_topic")._value
      self.Detect3D_topic = self.get_parameter("OutputDetect3D_topic")._value
      self.freq = self.get_parameter("frequency")._value
      self.counter = self.get_parameter("start_counter")._value   
      self.frame_id = self.get_parameter("frame_id")._value
      self.debug = self.get_parameter("debug")._value
      
      self.timer = self.create_timer(1/self.freq, self.timer_callback)
      
      self.output_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, depth=1)
      
      self.project2DPub_ = self.create_publisher(Detection2DArray, self.Project2D_topic, self.output_qos)
      self.detect2DPub_ = self.create_publisher(Detection2DArray, self.Detect2D_topic, self.output_qos)
      self.detect3DPub_ = self.create_publisher(Detection3DArray, self.Detect3D_topic, self.output_qos)
      
      self.get_logger().info("TestDetectionMatcher now running, Publishing on coresponding detections in " + self.Project2D_topic + ", " + self.Detect2D_topic + " and " + self.Detect3D_topic + " topics.")
    
   def timer_callback(self):
      pro2Dmsg = Detection2DArray()
      det2Dmsg = Detection2DArray()
      det3Dmsg = Detection3DArray()
      
      project_det, detect = Detection2D(), Detection2D()
      
      project_det.bbox.center.x = 0.
      project_det.bbox.size_x = 1.
      project_det.bbox.center.y = 0.
      project_det.bbox.size_y = 10.
      detect.bbox.center.x = 1.
      detect.bbox.size_x = 10.
      detect.bbox.center.y = 1.
      detect.bbox.size_y = 10.
      
      hyp = ObjectHypothesisWithPose()
      hyp.id = str(self.counter)
      detect.results.append(hyp)
      
      det3d = Detection3D()
      
      pro2Dmsg.header.frame_id = self.frame_id
      pro2Dmsg.header.stamp = self.get_clock().now().to_msg()
      det2Dmsg.header = det3Dmsg.header = pro2Dmsg.header
      
      pro2Dmsg.detections.append(project_det)
      det2Dmsg.detections.append(detect)
      det3Dmsg.detections.append(det3d)
      
      self.project2DPub_.publish(pro2Dmsg)
      self.detect2DPub_.publish(det2Dmsg)
      self.detect3DPub_.publish(det3Dmsg)
      if self.debug:
         self.get_logger().info('Publishing {} msgs.'.format(self.counter))
      self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    tester = DetectionMatcherTester()
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
