import rclpy
import cv2
import os
import math
import numpy as np

from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D, Detection2DArray, Detection2D, ObjectHypothesisWithPose

from .deep_sort.nn_matching import NearestNeighborDistanceMetric 
from .deep_sort.tracker import Tracker3D, Tracker
from .deep_sort.detection import Detection3D as Det3D
from .deep_sort.detection import Detection

class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker_3d')

        self.declare_parameter("input_det_topic", "/detections_det") # detections_det
        self.declare_parameter("output_det_topic", "/kalman_det")
        self.declare_parameter("dim", 3)
        self.declare_parameter("model_path", None)
        self.declare_parameter("metric", "euclidean")
        self.declare_parameter("matching_threshold", 0.2)
        self.declare_parameter("budget", 10)
        self.declare_parameter("max_iou_distance", 1.0)
        self.declare_parameter("max_age", 30)
        self.declare_parameter("n_init", 1)

        self.declare_parameter("debug", False)

        self._input_det_topic = self.get_parameter("input_det_topic")._value
        self._output_det_topic = self.get_parameter("output_det_topic")._value
        self._dim = self.get_parameter("dim")._value
        self._model_path = self.get_parameter("model_path")._value

        self._metric = self.get_parameter("metric")._value
        self._matching_threshold = self.get_parameter("matching_threshold")._value
        self._budget = self.get_parameter("budget")._value
        self._max_iou_distance = self.get_parameter("max_iou_distance")._value
        self._max_age = self.get_parameter("max_age")._value
        self._n_init = self.get_parameter("n_init")._value

        
        self.debug = self.get_parameter("debug")._value
        
        if self._dim == 2:
             self.detect2DSub_ = self.create_subscription(Detection2DArray, self._input_det_topic, self._on_detections2D, 10)
             self.detect2DPub_ = self.create_publisher(Detection2DArray, self._output_det_topic, 10)
             if not self._model_path == None:
                 if not os.path.exists(self._model_path):
                     self.get_logger().error('Tracker node Path parameter: File {} does not exist. - Stopping'.format(self._model_path))
                     return
                 from .deep_sort.generate_detections import create_box_encoder
                 self._encoder = create_box_encoder(self._model_path, input_name="images",
                       output_name="features", batch_size=32)
        elif self._dim == 3:
             self.detect3DSub_ = self.create_subscription(Detection3DArray, self._input_det_topic, self._on_detections3D, 10)
             self.detect3DPub_ = self.create_publisher(Detection3DArray, self._output_det_topic, 10)
        else:
            self.get_logger().error('Tracker node got wrong dimension parameter - Stopping')
            return
        if self._metric not in ["cosine", "euclidean"]:
            self.get_logger().error('Tracker node got wrong metric parameter - Stopping')
            return 
        self.metric = NearestNeighborDistanceMetric(self._metric, self._matching_threshold, self._budget)
        if self._dim == 2:
            self.tracker = Tracker(self.metric, self._max_iou_distance, self._max_age, self._n_init)
        else:
            self.tracker = Tracker3D(self.metric, self._max_iou_distance, self._max_age, self._n_init)

        self.get_logger().info('Tracker node has started.')

    def _on_detections2D(self, detections_msg):
        detections=[]
        if not self._model_path == None and len(detections_msg.detections) != 0:
            bgr_image = np.array(detections_msg.detections[0].source_img.data).reshape([detections_msg.detections[0].source_img.height, detections_msg.detections[0].source_img.width, 3]) # first detection has the source image
            #self.get_logger().info('Image shape {}.'.format(bgr_image.shape))
            dets=np.array([[int(det.bbox.center.x - det.bbox.size_x/2),int(det.bbox.center.y - det.bbox.size_y/2),det.bbox.size_x, det.bbox.size_y] for det in detections_msg.detections])
            confs=[det.results[0].score if len(det.results) > 0 else 0 for det in detections_msg.detections]
            cls=[det.results[0].id if len(det.results) > 0 else 0 for det in detections_msg.detections]
            features=self._encoder(bgr_image.copy(), dets.copy())
        for i, det in zip(range(len(detections_msg.detections)),detections_msg.detections):
            if not self._model_path == None:
                detections.append(Detection((dets[i,0],dets[i,1],dets[i,2],dets[i,3]), confs[i], cls[i], features[i]))
            else:
                detections.append(Detection((det.bbox.center.x - det.bbox.size_x/2,
                                             det.bbox.center.y - det.bbox.size_y/2,
                                             det.bbox.size_x, det.bbox.size_y), det.results[0].score if len(det.results) > 0 else 0, det.results[0].id if len(det.results) > 0 else None,(i,i)))
        if self.debug:
            self.get_logger().info('Tracker_2D tracks # before: {}.'.format(len(self.tracker.tracks)))
        if self.debug:
            self.get_logger().info('Detection2D_msg #: {}.'.format(len(detections_msg.detections)))
        self.tracker.predict()
        self.tracker.update(detections)
        if self.debug:
            self.get_logger().info('Tracker_2D tracks # after: {}.'.format(len(self.tracker.tracks)))
        arr = Detection2DArray()
        arr.header=detections_msg.header
        for track in self.tracker.tracks:
            if not track.is_confirmed():
                continue
            det = Detection2D()
            det.bbox.center.x=track.mean[0]
            det.bbox.center.y=track.mean[1]
            det.bbox.size_x=track.mean[2]*track.mean[3]
            det.bbox.size_y=track.mean[3]
            if track.cls != None:
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = track.cls
                hypothesis.score = track.confidence
                det.results.append(hypothesis)
            det.tracking_id=str(track.track_id)
            arr.detections.append(det)
        if self.debug:
            self.get_logger().info('Tracker_2D publishing.')
        self.detect2DPub_.publish(arr)
        
    def _on_detections3D(self, detections_msg):
        detections=[]
        for i, det in zip(range(len(detections_msg.detections)),detections_msg.detections):
            detections.append(Det3D((det.bbox.center.position.x - det.bbox.size.x/2,
                                           det.bbox.center.position.y - det.bbox.size.y/2,
                                           det.bbox.center.position.z - det.bbox.size.z/2,
                                           det.bbox.size.x,
                                           det.bbox.size.y,
                                           det.bbox.size.z),
                                           det.results[0].score if len(det.results) > 0 else 0, 
                                           det.results[0].id if len(det.results) > 0 else None, 
                                           math.acos(det.bbox.center.orientation.x), # assume rotation only in xy plane
                                           None))

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
            det.bbox.center.orientation.x=math.cos(track.angle) if track.angle != None else 0.
            det.bbox.center.orientation.y=math.sin(track.angle) if track.angle != None else 0.
            det.bbox.center.orientation.z=0.
            det.bbox.center.orientation.w=0.
            det.bbox.size.x=track.mean[3]
            det.bbox.size.y=track.mean[4]
            det.bbox.size.z=track.mean[5]
            if track.cls != None:
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = track.cls
                hypothesis.score = track.confidence
                det.results.append(hypothesis)
            det.tracking_id=str(track.track_id)
            arr.detections.append(det)
        if self.debug:
            self.get_logger().info('Tracker_3D publishing.')
        self.detect3DPub_.publish(arr)

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
