import rclpy
import cv2
import os
from rclpy.node import Node
import message_filters

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2

from ros2_numpy.point_cloud2 import pointcloud2_to_array
import numpy as np
import pcl


class ImagePclSaver(Node):

    def __init__(self):
        super().__init__('image_pcl_saver')
        
        self._synchronized = False
        if self._synchronized:
            self._image_subscription = self.create_subscription(
                Image,'image', self._image_listener_callback, 10)
            self._pcl_subscription = self.create_subscription(
                PointCloud2,'livox/lidar', self._pcl_listener_callback, 10)
            self._frequency = 0.01 # time
            self._synchronizer = message_filters.ApproximateTimeSynchronizer(
                (self._image_subscription, self._pcl_subscription), 1, self._frequency)
            self._synchronizer.registerCallback(self._on_detections)
        else:
            self._image_subscription = self.create_subscription(
                Image,'/detection_visualizer/dbg_images', self._image_listener_callback, 10) #"image"
            self._pcl_subscription = self.create_subscription(
                PointCloud2,'livox/lidar', self._pcl_listener_callback, 10)
            
        self._image_subscription  # prevent unused variable warning
        self._pcl_subscription

        self._bridge = CvBridge()

        self._image_counter = self._pcl_counter = 0
        self._destFolder = "./"
        self._image_subdir = "image_det/" #"image"
        self._pcl_subdir = "pcl/"
        self._imgExtention = "jpg"
        self._pclExtention = "pcd"

        # check if foldersfolder
        if not os.path.exists(self._destFolder + self._image_subdir):
            os.makedirs(self._destFolder + self._image_subdir)
        if not os.path.exists(self._destFolder + self._pcl_subdir):
            os.makedirs(self._destFolder + self._pcl_subdir)

        self.get_logger().info('Image_pcl_saver node have started.')

    def _image_listener_callback(self, msg):

        pathStream = "{0}{1}frame{2:06}.{3}".format(self._destFolder,self._image_subdir,msg.header.stamp.sec,self._imgExtention) #self._image_counter
        ret = cv2.imwrite(pathStream, self._bridge.imgmsg_to_cv2(msg))
        if ret == True:
            self.get_logger().info("I saved image #{} as {}".format(self._image_counter, pathStream))
        else:
            self.get_logger().error("Image cannot be saved on {}".format(pathStream))
        self._image_counter += 1
    
    def _pcl_listener_callback(self, msg):
        pathStream = "{0}{1}scan_{2}.{3}".format(self._destFolder,self._pcl_subdir,self._pcl_counter,self._pclExtention)
        
        arr = pointcloud2_to_array(msg,squeeze=True)
        if len(arr.dtype) == 6:
            arr=np.array(arr.tolist())[:,0:3]
        
        cloud = pcl.PointCloud(arr.astype(np.float32))
        pcl.save(cloud, pathStream, format="pcd", binary=False)
        self.get_logger().info("I saved point cloud #{} as {}".format(self._pcl_counter, pathStream))
        self._pcl_counter += 1

    def _on_detections(self, image_msg, detections_msg):
        return

def main(args=None):
    rclpy.init(args=args)

    image_pcl_saver = ImagePclSaver()

    rclpy.spin(image_pcl_saver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_pcl_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()