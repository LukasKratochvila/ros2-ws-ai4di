from posixpath import split

from numpy.core.fromnumeric import reshape
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import pcl
from ros2_numpy.point_cloud2 import array_to_pointcloud2
import numpy as np
import os

class PclPublisher(Node):
      def __init__(self):
         super().__init__('pcl_publisher')
         self._publisher = self.create_publisher(PointCloud2, 'livox/lidar', 10)
         self._timer_period = 1/10  # seconds
         self._timer = self.create_timer(self._timer_period, self._timer_callback)

         self._dir = "/home/kratochvila/Desktop/mereni3/pcl"#"/home/kratochvila/Desktop/lidar/mereni2/pcl"#"/home/kratochvila/Data/recording/lidar_center/"
         self._files = [file for file in os.listdir(self._dir) if file.split(".")[-1] == "pcd"]
         self._files.sort()
         self._counter = 0
         
         self.get_logger().info('Pcl_publisher node have started.')
         

      def _timer_callback(self):
          if self._counter > len(self._files)-1:
              self._counter = 0
          
          cloud = pcl.load(os.path.join(self._dir,self._files[self._counter]), format="pcd")
          cloud_arr = np.rec.fromarrays(np.transpose(cloud.to_array()),dtype=[('x', np.float32),('y', np.float32),('z', np.float32)])
          msg = array_to_pointcloud2(cloud_arr,self.get_clock().now().to_msg(),"map")
          self._publisher.publish(msg)
          self.get_logger().info('Publishing a point cloud from file {}'.format(self._files[self._counter]))
          self._counter += 1

def main(args=None):
    rclpy.init(args=args)
    pcl_publisher = PclPublisher()
    rclpy.spin(pcl_publisher)
    pcl_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()