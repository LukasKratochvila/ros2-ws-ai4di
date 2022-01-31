#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgcodecs.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

#include <cmath>

class Projection : public rclcpp::Node
{
  public:
    Projection() : Node("projection")
    {
      this->declare_parameter("input_topic", "/detections");
      this->declare_parameter("output_topic", "/cluster_det");
      this->declare_parameter("output_pcl_topic", "~/pcl");
      this->declare_parameter("output_img_topic", "~/img");
      this->declare_parameter("output_frame", "image");

      this->declare_parameter("debug", false);
      
      output_frame = this->get_parameter("output_frame").as_string();
      debug = this->get_parameter("debug").as_bool();

      det_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      this->get_parameter("input_topic").as_string(), 10, std::bind(&Projection::topic_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(this->get_parameter("output_topic").as_string(), 10);
      if (debug){
        pcl_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(this->get_parameter("output_pcl_topic").as_string(), 10);
        img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(this->get_parameter("output_img_topic").as_string(), 10);
      }

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
      std::stringstream ss;
      if (debug)
        ss << std::endl << "K = " << std::endl << K  << std::endl << "DistCoefs = " << std::endl << D << std::endl;
      ss << "Projection has started.";

      RCLCPP_INFO(this->get_logger(), ss.str());
    }

  private:
    void topic_callback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr detections) const
    {
      std::string fromFrameRel = detections->markers.at(0).header.frame_id;
      std::string toFrameRel = output_frame;
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel, tf2::TimePointZero);
        //RCLCPP_INFO(get_logger(), "Transform %s to %s is: RX: %0.4f, RY: %0.4f, RZ: %0.4f TX: %0.4f, TY: %0.4f, TZ: %0.4f", toFrameRel.c_str(), fromFrameRel.c_str(), transformStamped.transform.rotation.x,transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
      }
      catch (tf2::TransformException & ex) {
        RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      }
      std::stringstream ss;
      if (debug){
        ss << detections->markers.size()/2 << std::endl;
        RCLCPP_INFO(get_logger(),ss.str());
        ss.clear();
      }
      
      //tf2::Transform trans(transformStamped.transform.rotation);
      // Transform to image frame
      
      visualization_msgs::msg::MarkerArray trans_det = visualization_msgs::msg::MarkerArray();
      geometry_msgs::msg::PoseStamped marker_pose;
      visualization_msgs::msg::Marker marker;
      for(size_t i = 0; i < detections->markers.size(); i++) { 
        marker = detections->markers.at(i);
        marker_pose.header = marker.header;
        marker_pose.pose = marker.pose;
        //marker_pose = tf_buffer_->transform(marker_pose,"image");
        tf2::doTransform(marker_pose,marker_pose,transformStamped);
        marker.pose = marker_pose.pose;
        trans_det.markers.push_back(marker);
      }
      if (debug)
        pcl_publisher_->publish(trans_det);
      
      cv::Mat rVec = (cv::Mat_<double>(3,1) << 0, 0, 0);
      cv::Mat tVec = (cv::Mat_<double>(3,1) << 0, 0, 0);

      std::vector<cv::Point3d> objectPoints;
      std::vector<cv::String> objectStrings;
      for(size_t i = 0; i < trans_det.markers.size(); i++) { 
        auto pt = trans_det.markers.at(i);
        if (pt.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING){
          objectStrings.push_back(cv::String(pt.text));
          continue;
        }
        objectPoints.push_back(cv::Point3d(-pt.pose.position.y, -pt.pose.position.z, pt.pose.position.x));
        objectPoints.push_back(cv::Point3d(-(pt.pose.position.y - pt.scale.y/2), -(pt.pose.position.z - pt.scale.z/2), (pt.pose.position.x - pt.scale.x/2)));
        //objectPoints.push_back(cv::Point3d(-(pt.pose.position.y + pt.scale.y/2), -(pt.pose.position.z - pt.scale.z/2), (pt.pose.position.x - pt.scale.x/2)));
        objectPoints.push_back(cv::Point3d(-(pt.pose.position.y + pt.scale.y/2), -(pt.pose.position.z + pt.scale.z/2), (pt.pose.position.x - pt.scale.x/2)));
        //objectPoints.push_back(cv::Point3d(-(pt.pose.position.y - pt.scale.y/2), -(pt.pose.position.z + pt.scale.z/2), (pt.pose.position.x - pt.scale.x/2)));
      }
      std::vector<cv::Point2d> projectedPoints;
      cv::projectPoints(objectPoints, rVec, tVec, K, D, projectedPoints);

      if (debug){
        cv::Mat img(480, 640, CV_8UC3, CV_RGB(255, 255, 255));
        for(size_t i = 0; i < objectStrings.size(); i++){
          cv::circle(img, cv::Point2d(projectedPoints.at(3*i).x,projectedPoints.at(3*i).y), 5, CV_RGB(255,0,0));
          cv::putText(img, objectStrings.at(i),cv::Point2d(projectedPoints.at(3*i).x,projectedPoints.at(3*i).y), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0));
          cv::rectangle(img, cv::Rect2d(cv::Point2d(projectedPoints.at(3*i+1).x,projectedPoints.at(3*i+1).y),cv::Point2d(projectedPoints.at(3*i+2).x,projectedPoints.at(3*i+2).y)),CV_RGB(0,0,255));
        }
        ss << projectedPoints;
        RCLCPP_INFO(get_logger(),ss.str());
        ss.clear();

        cv_bridge::CvImage out_msg;
        out_msg.image = img;
        out_msg.header = trans_det.markers.at(0).header;
        out_msg.encoding = "bgr8";
        out_msg.header.frame_id = output_frame;
        //out_msg.header.stamp = detections->markers.at(0).header.stamp;

        img_publisher_->publish(*out_msg.toImageMsg());
      }

      vision_msgs::msg::Detection2DArray::UniquePtr detect(new vision_msgs::msg::Detection2DArray);
      detect->header=trans_det.markers.at(0).header;
      detect->detections.reserve(objectStrings.size());
      for (size_t i = 0; i < objectStrings.size(); ++i) {
        detect->detections.emplace_back();
        auto & detection_ros = detect->detections.back();

        // Copy probabilities of each class
        //for (int cls = 0; cls < detection.classes; ++cls) {
        //  if (detection.prob[cls] > 0.0f) {
        //    detection_ros.results.emplace_back();
        //    auto & hypothesis = detection_ros.results.back();
        //    hypothesis.id = impl_->class_names_.at(cls);
        //    hypothesis.score = detection.prob[cls];
        //  }
        //}
        detection_ros.results.emplace_back();
        auto & hypothesis = detection_ros.results.back();
        hypothesis.id = "Unknown";
        hypothesis.score = 0;
        detection_ros.results.emplace_back();
        auto & dis = detection_ros.results.back();
        dis.id = "Distance";
        dis.score = -sqrt(pow(trans_det.markers.at(2*i).pose.position.x,2)+pow(trans_det.markers.at(2*i).pose.position.y,2)+pow(trans_det.markers.at(2*i).pose.position.z,2));
        
        // Copy bounding box, darknet uses center of bounding box too
        detection_ros.bbox.center.x = (projectedPoints.at(3*i+2).x+projectedPoints.at(3*i+1).x)/2;
        detection_ros.bbox.center.y = (projectedPoints.at(3*i+2).y+projectedPoints.at(3*i+1).y)/2;
        detection_ros.bbox.size_x = projectedPoints.at(3*i+2).x-projectedPoints.at(3*i+1).x;
        detection_ros.bbox.size_y = projectedPoints.at(3*i+2).y-projectedPoints.at(3*i+1).y;
      }
      publisher_->publish(*detect);
    }
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr det_subscription_{nullptr};
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr publisher_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_{nullptr}; 
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pcl_publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};

    cv::Mat K = (cv::Mat_<double>(3,3) << 531.7944, 0, 296.3159, 0, 530.179, 255.4285, 0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(5,1) << -0.4512, 0.7176, -0.0014, 0.0017, -1.2896); // (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]])

    std::string output_frame;
    bool debug;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Projection>());
  rclcpp::shutdown();
  return 0;
}
