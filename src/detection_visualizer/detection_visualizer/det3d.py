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
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

class Det3dVisualizerNode(Node):

    def __init__(self):
        super().__init__('detection_visualizer_3d')

        self.declare_parameter("input_det3d_topic", '/detections3D')
        self.declare_parameter("output_topic", '~/dbg_markers')
        self.declare_parameter("debug", True)

        self.input_det3d_topic = self.get_parameter("input_det3d_topic")._value
        self.output_topic = self.get_parameter("output_topic")._value
        self.debug = self.get_parameter("debug")._value

        self.output_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=QoSReliabilityPolicy.RELIABLE,
                                     depth=1)
        self.detect3DSub_ = self.create_subscription(Detection3DArray, self.input_det3d_topic, self.on_detections, 10)

        self.detectVizPub_ = self.create_publisher(MarkerArray, self.output_topic, self.output_qos)
        
        self.get_logger().info("Detection_visualizer_3d has started.")

    def on_detections(self, detections_msg):
    
        if self.debug:
            self.get_logger().info("Recieved 3D detections.")
        detVizmsg = MarkerArray()
        counter = 0
        
        for detection in detections_msg.detections:
            marker = Marker()
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
            marker.header = detections_msg.header
            marker.id = counter
            counter += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose = detection.bbox.center
            
            marker.scale = detection.bbox.size

            marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.6, 0.7, 0.7, 0.7
            
            detVizmsg.markers.append(marker);

            text = Marker()
            text.header = detections_msg.header
            text.id = counter
            counter += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            
            text.text = "ID: {}\nClass: {}\nProb: {:3f}".format(detection.tracking_id, max_class, max_score)
            
            text.pose = detection.bbox.center
            
            text.scale.x, text.scale.y, text.scale.z = 0.5, 0.5, 0.5
            
            text.color.a, text.color.r, text.color.g, text.color.b = 0.6, 0.7, 0.7, 0.7
            
            detVizmsg.markers.append(text)

        self.detectVizPub_.publish(detVizmsg)
        if self.debug:
            self.get_logger().info("Publishing MarkerArray")
 
def main():
    rclpy.init()
    rclpy.spin(Det3dVisualizerNode())
    rclpy.shutdown()
