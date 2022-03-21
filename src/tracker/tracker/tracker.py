import rclpy
import cv2
import os
import numpy as np

from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D

from .deep_sort.nn_matching import NearestNeighborDistanceMetric 
from .deep_sort.tracker import Tracker3D
from .deep_sort.detection import Detection3D as Det3D

class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker_3d')

        self.declare_parameter("input_det_topic", "/detections_det") # detections_det
        self.declare_parameter("output_det_topic", "/kalman_det")
        
        self.declare_parameter("debug", False)

        self._input_det_topic = self.get_parameter("input_det_topic")._value
        self._output_det_topic = self.get_parameter("output_det_topic")._value
        
        self.debug = self.get_parameter("debug")._value
        
        self.detect3DSub_ = self.create_subscription(Detection3DArray, self._input_det_topic, self._on_detections, 10)

        self.detectVizPub_ = self.create_publisher(Detection3DArray, self._output_det_topic, 10)
        
        self.metric = NearestNeighborDistanceMetric("cosine", 0.2, None)
        self.tracker = Tracker3D(self.metric,0.9,5)

        self.get_logger().info('Tracker_3D node has started.')

    def _on_detections(self, detections_msg):
        detections=[]
        for det in detections_msg.detections:
            detections.append(Det3D((int(det.bbox.center.position.x - det.bbox.size.x/2),
                                           int(det.bbox.center.position.y - det.bbox.size.y/2),
                                           int(det.bbox.center.position.z - det.bbox.size.z/2),
                                           det.bbox.size.x,
                                           det.bbox.size.y,
                                           det.bbox.size.z),
                                           0, (det.bbox.center.position.x,
                                               det.bbox.center.position.y,
                                               det.bbox.center.position.z))) # -1,-1,-1 works
        if self.debug:
            self.get_logger().info('Tracker_3D tracks # before: {}.'.format(len(self.tracker.tracks)))
        if self.debug:
            self.get_logger().info('Detection_msg #: {}.'.format(len(detections_msg.detections)))
        self.tracker.predict()
        self.tracker.update(detections)
        if self.debug:
            self.get_logger().info('Tracker_3D tracks # after: {}.'.format(len(self.tracker.tracks)))
        arr = Detection3DArray()
        arr.header=detections_msg.header
        for track in self.tracker.tracks:
            if not track.is_confirmed():
                continue
            det = Detection3D()
            det.bbox.center.position.x=track.mean[0]
            det.bbox.center.position.y=track.mean[1]
            det.bbox.center.position.z=track.mean[2]
            det.bbox.size.x=track.mean[3]
            det.bbox.size.y=track.mean[4]
            det.bbox.size.z=track.mean[5]
            det.tracking_id=str(track.track_id)
            arr.detections.append(det)
        #for i, det in zip(range(len(detections)),detections):
        #    detections_msg.detections[i].bbox.center.position.x=det.tlwh[0]
        #    detections_msg.detections[i].bbox.center.position.y=det.tlwh[1]
        #    detections_msg.detections[i].bbox.center.position.z=det.tlwh[2]
        #    detections_msg.detections[i].bbox.size.x=det.tlwh[3]
        #    detections_msg.detections[i].bbox.size.y=det.tlwh[4]
        #    detections_msg.detections[i].bbox.size.z=det.tlwh[5]
        if self.debug:
            self.get_logger().info('Tracker_3D publishing.')
        self.detectVizPub_.publish(arr)

def main(args=None):
    rclpy.init(args=args)

    tracker_node = TrackerNode()

    rclpy.spin(tracker_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
