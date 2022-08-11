import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

import message_filters

from vision_msgs.msg import Detection2DArray, Detection3DArray

class DetectionMatcher(Node):
   def __init__(self):
      super().__init__('detection_matcher')
      self.declare_parameter("Project2D_topic", "/cluster_det")
      self.declare_parameter("Detect2D_topic", "/detector_node/detections")
      self.declare_parameter("Detect3D_topic", "/detections_det")
      self.declare_parameter("Output3D_topic", "/detections3D")
      self.declare_parameter("queue_size", 10)
      self.declare_parameter("time_toll", 10)
      self.declare_parameter("tresh", 0.9)
      self.declare_parameter("only_match", True)  
      self.declare_parameter("debug", True)     

      self.Project2D_topic = self.get_parameter("Project2D_topic")._value
      self.Detect2D_topic = self.get_parameter("Detect2D_topic")._value
      self.Detect3D_topic = self.get_parameter("Detect3D_topic")._value
      self.Output3D_topic = self.get_parameter("Output3D_topic")._value
      self.queue_size = self.get_parameter("queue_size")._value
      self.time_toll = self.get_parameter("time_toll")._value
      self.tresh = self.get_parameter("tresh")._value
      self.only_match = self.get_parameter("only_match")._value
      self.debug = self.get_parameter("debug")._value
      
      self.project2DSub_ = message_filters.Subscriber(self, Detection2DArray, self.Project2D_topic)
      self.detect2DSub_ = message_filters.Subscriber(self, Detection2DArray, self.Detect2D_topic)
      self.detect3DSub_ = message_filters.Subscriber(self, Detection3DArray, self.Detect3D_topic)
      
      self.sync_ = message_filters.ApproximateTimeSynchronizer((self.project2DSub_, self.detect2DSub_, self.detect3DSub_), self.queue_size, self.time_toll)
      self.sync_.registerCallback(self.syncCallback)
         
      self.output_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, depth=1)
      self.detect3DPub_ = self.create_publisher(Detection3DArray, self.Output3D_topic, self.output_qos)
      
      self.get_logger().info("DetectionMatcher now running, looking for coresponding detections in " + self.Project2D_topic + ", " + self.Detect2D_topic + " and " + self.Detect3D_topic + " topics.")
    
   def IoU(self, bbox1, bbox2, tresh):
      area1 = (bbox1.size_x * bbox1.size_y)
      area2 = (bbox2.size_x * bbox2.size_y)
        
      distx = max(0,(min((bbox1.center.x + bbox1.size_x/2.0), (bbox2.center.x + bbox2.size_x/2.0)) - max((bbox1.center.x - bbox1.size_x/2.0), (bbox2.center.x - bbox2.size_x/2.0))))
      disty = max(0,(min((bbox1.center.y + bbox1.size_y/2.0), (bbox2.center.y + bbox2.size_y/2.0)) - max((bbox1.center.y - bbox1.size_y/2.0), (bbox2.center.y - bbox2.size_y/2.0))))
        
      areaI = distx * disty
        
      IoverU = areaI / (area1 + area2 - areaI)
       
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
         
      outmsg = Detection3DArray()
      outmsg.header = detect3Dmsg.header
      # create list of ids for matching
      ids=list()
      for det in detect3Dmsg.detections:
         ids.append(det.tracking_id)

      # IoU metric is applied across all YOLO and projections, individual maximums are selected
      for project_det in project2Dmsg.detections:
         candidates = list()
         for detect in detect2Dmsg.detections:
            candidates.append(self.IoU(project_det.bbox, detect.bbox, self.tresh))
         if self.debug:
            self.get_logger().info("IOU:{}".format(candidates))
         if len(candidates) == 0:
            if self.debug:
               self.get_logger().info("Projection #{} No candidate -> continue.".format(project_det.tracking_id))
            continue
         if max(candidates) == 0:
            if self.debug:
               self.get_logger().info("Projection #{} All candidates has zero IOU -> continue.".format(project_det.tracking_id))
            continue
         maxIoU_index = candidates.index(max(candidates))
         det3d_index = ids.index(project_det.tracking_id)

         # Remove Unknown hypothesis
         det = detect3Dmsg.detections[det3d_index]
         for res in detect3Dmsg.detections[det3d_index].results:
            if res.id == "Unknown":
               detect3Dmsg.detections[det3d_index].results.remove(res)
         # Rewrite label to 3D detection
         for res in detect2Dmsg.detections[maxIoU_index].results:
            det.results.append(res)
         # Add distance information
         for res in project_det.results:
            if res.id == "Distance":
               det.results.append(res)

         outmsg.detections.append(det)
    
      # Only the classified boxes get republished as vision_msgs::msg::Detection3DArray
      if self.only_match:
         self.detect3DPub_.publish(outmsg)
      else:
         self.detect3DPub_.publish(detect3Dmsg)
      if self.debug:
         if self.only_match:
            self.get_logger().info("Published {} detection3darray message.".format(len(outmsg.detections)))
         else:
            self.get_logger().info("Published {} detection3darray message.".format(len(detect3Dmsg.detections)))

def main(args=None):
    rclpy.init(args=args)
    matcher = DetectionMatcher()
    rclpy.spin(matcher)
    matcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
