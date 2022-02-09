import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

import message_filters

from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D
#from visualization_msgs.msg import MarkerArray

class DetectionMatcher(Node):
   def __init__(self):
      super().__init__('detection_matcher')
      self.declare_parameter("Project2D_topic", "/projection")
      self.declare_parameter("Detect2D_topic", "/detector_node/detections")
      self.declare_parameter("Detect3D_topic", "/detections")
      self.declare_parameter("Output3D_topic", "/detections3D")
      self.declare_parameter("queue_size", 10)
      self.declare_parameter("time_toll", 10)
      self.declare_parameter("tresh", 0.5)
      self.declare_parameter("debug", True)     

      self.Project2D_topic = self.get_parameter("Project2D_topic")._value
      self.Detect2D_topic = self.get_parameter("Detect2D_topic")._value
      self.Detect3D_topic = self.get_parameter("Detect3D_topic")._value
      self.Output3D_topic = self.get_parameter("Output3D_topic")._value
      self.queue_size = self.get_parameter("queue_size")._value
      self.time_toll = self.get_parameter("time_toll")._value
      self.tresh = self.get_parameter("tresh")._value
      self.debug = self.get_parameter("debug")._value
      
      self.project2DSub_ = message_filters.Subscriber(self, Detection2DArray, self.Project2D_topic)
      self.detect2DSub_ = message_filters.Subscriber(self, Detection2DArray, self.Detect2D_topic)
      self.detect3DSub_ = message_filters.Subscriber(self, Detection3DArray, self.Detect3D_topic)
      
      self.sync_ = message_filters.ApproximateTimeSynchronizer((self.project2DSub_, self.detect2DSub_, self.detect3DSub_), self.queue_size, self.time_toll)
      self.sync_.registerCallback(self.syncCallback)
      if (False):
         self.detect3DSub_.registerCallback(self.sync3DCallback)
         
      self.output_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, depth=1)
      self.detect3DPub_ = self.create_publisher(Detection3DArray, self.Output3D_topic, self.output_qos)
      
      self.get_logger().info("DetectionMatcher now running, looking for coresponding detections in " + self.Project2D_topic + ", " + self.Detect2D_topic + " and " + self.Detect3D_topic + " topics.")
    
   def IoU(self, bbox1center_x, bbox1size_x, bbox1center_y, bbox1size_y, bbox2center_x, bbox2size_x, bbox2center_y, bbox2size_y, tresh):
      area1 = (bbox1size_x * bbox1size_y);
      area2 = bbox2size_x * bbox2size_y;
        
      distx = min(bbox1size_x/2 + bbox1center_x, bbox2size_x/2 + bbox2center_x) - max(bbox1size_x/2 - bbox1center_x, bbox2size_x/2 - bbox2center_x);
      disty = min(bbox1size_y/2 + bbox1center_y, bbox2size_y/2 + bbox2center_y) - max(bbox1size_y/2 - bbox1center_y, bbox2size_y/2 - bbox2center_y);
        
      areaI = distx * disty;
        
      IoverU = areaI / (area1 + area2 - areaI);
       
      if(IoverU < tresh):
         return 0
      else:
         return IoverU
    
   def syncCallback(self, project2Dmsg, detect2Dmsg, detect3Dmsg):
   # When projected 3D clusters and 2D YOLO detections come concurrently
      if len(project2Dmsg.detections) != len(detect3Dmsg.detections):
         self.get_logger().error("Different detection size of {} and {}.".format(self.Project2D_topic, self.Detect3D_topic))
         return
      if self.debug:
         self.get_logger().info("Processing three topics.")

      # IoU metric is applied across all YOLO and projections, individual maximums are selected
      for i, project_det in zip(range(len(project2Dmsg.detections)), project2Dmsg.detections):
         candidates = list()
         for detect in detect2Dmsg.detections:
            candidates.append(self.IoU(project_det.bbox.center.x,
                                       project_det.bbox.size_x,
                                       project_det.bbox.center.y,
                                       project_det.bbox.size_y,
                                       detect.bbox.center.x,
                                       detect.bbox.size_x,
                                       detect.bbox.center.y,
                                       detect.bbox.size_y,
                                       self.tresh))
         maxIoU = candidates.index(max(candidates))
         for res in detect2Dmsg.detections[maxIoU].results:
            detect3Dmsg.detections[i].results.append(res)
    
      # Only the classified boxes get republished as vision_msgs::msg::Detection3DArray
      self.detect3DPub_.publish(detect3Dmsg)
      if self.debug:
         self.get_logger().info("Published detection3darray message.")
    
   def sync3DCallback(self, msg):
   # When 3D detections come and are to be forwarded downstream (converts from MarkerArray to Detection3DArray)
      if self.debug: 
         self.get_logger().info("Processing one topic.")
      # Republishes 3D clustering detections as vision_msgs::msg::Detection3DArray
      self.detect3DPub_.publish(msg)
      if self.debug: 
         self.get_logger().info("Published detection3darray message.")

def main(args=None):
    rclpy.init(args=args)
    matcher = DetectionMatcher()
    rclpy.spin(matcher)
    matcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
