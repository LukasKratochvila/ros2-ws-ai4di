# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

import cv2
import cv_bridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class DetectionVisualizerNode(Node):

    def __init__(self):
        super().__init__('detection_visualizer')

        self.declare_parameter("output_topic", '~/dbg_images')
        self.declare_parameter("input_img_topic", '/image')
        self.declare_parameter("input_det2d_topic", '/detector_node/detections')
        self.declare_parameter("time_toll", 0.01)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("start_counter", 0)
        self.declare_parameter("encoding", "bgr8")
        self.declare_parameter("min_size_x", 50)
        self.declare_parameter("min_size_y", 50)
        self.declare_parameter("debug", False)

        self._counter = self.get_parameter("start_counter")._value
        self.debug = self.get_parameter("debug")._value

        self._bridge = cv_bridge.CvBridge()
        self.encoding = self.get_parameter("encoding")._value

        self.min_size_x = self.get_parameter("min_size_x")._value
        self.min_size_y = self.get_parameter("min_size_y")._value

        output_image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1)

        self._image_pub = self.create_publisher(Image, self.get_parameter("output_topic")._value, output_image_qos)

        self._image_sub = message_filters.Subscriber(self, Image, self.get_parameter("input_img_topic")._value)
        self._detections_sub = message_filters.Subscriber(self, Detection2DArray, self.get_parameter("input_det2d_topic")._value)

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self._image_sub, self._detections_sub), self.get_parameter("queue_size")._value, self.get_parameter("time_toll")._value)
        self._synchronizer.registerCallback(self.on_detections)
        
        self.get_logger().info("Detection_visualizer have started.")

    def on_detections(self, image_msg, detections_msg):
        cv_image = self._bridge.imgmsg_to_cv2(image_msg)

        # Draw boxes on image
        for detection in detections_msg.detections:
            max_class = None
            max_score = 0.0
            dis = 0.0
            for hypothesis in detection.results:
                if hypothesis.id == "Distance":
                    dis = -hypothesis.score
                if hypothesis.score > max_score:
                    max_score = hypothesis.score
                    max_class = hypothesis.id
                elif hypothesis.score == 0.0:
                    max_class = 'Unknown'
            if max_class is None:
                #self.get_logger().err("Failed to find class with highest score")
                #continue
                max_class='No class'

            cx = detection.bbox.center.x
            cy = detection.bbox.center.y
            sx = detection.bbox.size_x
            sy = detection.bbox.size_y
            
            if (abs(sx) < self.min_size_x or abs(sy) < self.min_size_y):
                continue

            min_pt = (round(cx - sx / 2.0), round(cy - sy / 2.0))
            max_pt = (round(cx + sx / 2.0), round(cy + sy / 2.0))
            color = (0, 255, 0)
            thickness = 1
            cv2.rectangle(cv_image, min_pt, max_pt, color, thickness)

            label = '{} {:.2f}'.format(max_class, max_score)
            pos = (min_pt[0], max_pt[1])
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, label, pos, font, 0.75, color, 1, cv2.LINE_AA)
            label = '{} {:.1f}m'.format("Distance", dis)
            pos = (min_pt[0], max_pt[1]+25)
            cv2.putText(cv_image, label, pos, font, 0.75, color, 1, cv2.LINE_AA)
            
        msg=self._bridge.cv2_to_imgmsg(cv_image, encoding=self.encoding)
        msg.header=image_msg.header

        self._image_pub.publish(msg)
        if self.debug:
            self.get_logger().info("Publishing image #{0}".format(self._counter))
        self._counter+=1


def main():
    rclpy.init()
    rclpy.spin(DetectionVisualizerNode())
    rclpy.shutdown()
