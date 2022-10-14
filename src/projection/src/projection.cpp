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

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <cv_bridge/cv_bridge.h>

#include <cmath>

class Projection : public rclcpp::Node
{
  public:
    Projection() : Node("projection")
    {
      this->declare_parameter("input_topic", "/detections");
      this->declare_parameter("output_topic", "/cluster_det");
      this->declare_parameter("dbg_img_topic", "~/img");
      this->declare_parameter("output_frame", "image");

      this->declare_parameter("K",  std::vector<double>({531.7944, 0, 296.3159, 0, 530.179, 255.4285, 0, 0, 1}));
      this->declare_parameter("D", std::vector<double>({-0.4512, 0.7176, -0.0014, 0.0017, -1.2896}));

      this->declare_parameter("debug", false);
      
      
      output_frame = this->get_parameter("output_frame").as_string();
      debug = this->get_parameter("debug").as_bool();

      det_subscription_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      this->get_parameter("input_topic").as_string(), 10, std::bind(&Projection::topic_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(this->get_parameter("output_topic").as_string(), 10);
      if (debug){
        img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(this->get_parameter("dbg_img_topic").as_string(), 10);
      }

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
      std::vector<double> pom_K = this->get_parameter("K").as_double_array();
      std::vector<double> pom_D = this->get_parameter("D").as_double_array();
      memcpy(K.data, pom_K.data(), pom_K.size()*sizeof(double));
      memcpy(D.data, pom_D.data(), pom_D.size()*sizeof(double));

      std::stringstream ss;
      if (debug)
      {
        ss << std::endl << "K = " << std::endl << K  << std::endl << "DistCoefs = " << std::endl << D << std::endl;
        RCLCPP_INFO(this->get_logger(), ss.str());
        ss.clear();
      }
      ss << "Projection has started.";
      RCLCPP_INFO(this->get_logger(), ss.str());
      ss.clear();
    }

  private:
    void topic_callback(const vision_msgs::msg::Detection3DArray::ConstSharedPtr detections) const
    {
      std::stringstream ss;
      if (detections->detections.size() == 0)
      {
        ss << "Got 0 detections." << std::endl;
        RCLCPP_WARN(get_logger(),ss.str());
        ss.clear();
        return;
      } 
      std::string fromFrameRel = detections->detections.at(0).header.frame_id;
      std::string toFrameRel = output_frame;
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        }
      catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      }
      if (debug){
        ss << "Got detections number:" << detections->detections.size() << std::endl;
        RCLCPP_INFO(get_logger(),ss.str());
        ss.clear();
      }
      
      // Transform to image frame
      vision_msgs::msg::Detection3DArray trans_det = vision_msgs::msg::Detection3DArray();
      trans_det.header=detections->header;
      vision_msgs::msg::Detection3D det;
      geometry_msgs::msg::PoseStamped pose;
      for(size_t i = 0; i < detections->detections.size(); i++) { 
        det = detections->detections.at(i);
        pose.header=det.header;
        pose.pose=det.bbox.center;
        tf2::doTransform(pose,pose,transformStamped);
        det.bbox.center=pose.pose;
        trans_det.detections.push_back(det);
      }
      if (debug){
        ss << "Transformed detections number:" << trans_det.detections.size() << std::endl;
        RCLCPP_INFO(get_logger(),ss.str());
        ss.clear();
      }
      
      cv::Mat rVec = (cv::Mat_<double>(3,1) << 0, 0, 0);
      cv::Mat tVec = (cv::Mat_<double>(3,1) << 0, 0, 0);

      std::vector<cv::Point3f> objectPoints;
      std::vector<cv::String> objectStrings;
      std::vector<cv::Point2f> projectedPoints;
      std::vector<cv::Rect> rectangles;
      std::vector<cv::Point2f> centers;
      for(size_t i = 0; i < trans_det.detections.size(); i++) { 
        auto pt = trans_det.detections.at(i);

        objectStrings.push_back(pt.tracking_id);
        // x pointing from image to forward so we move it to z coordinate
        // and take oposite value from y and z
        objectPoints.push_back(cv::Point3f(-pt.bbox.center.position.y, -pt.bbox.center.position.z, pt.bbox.center.position.x));
        objectPoints.push_back(cv::Point3f(-(pt.bbox.center.position.y - pt.bbox.size.y/2), -(pt.bbox.center.position.z - pt.bbox.size.z/2), (pt.bbox.center.position.x - pt.bbox.size.x/2)));
        objectPoints.push_back(cv::Point3f(-(pt.bbox.center.position.y - pt.bbox.size.y/2), -(pt.bbox.center.position.z - pt.bbox.size.z/2), (pt.bbox.center.position.x + pt.bbox.size.x/2)));
        objectPoints.push_back(cv::Point3f(-(pt.bbox.center.position.y + pt.bbox.size.y/2), -(pt.bbox.center.position.z - pt.bbox.size.z/2), (pt.bbox.center.position.x + pt.bbox.size.x/2)));
        objectPoints.push_back(cv::Point3f(-(pt.bbox.center.position.y - pt.bbox.size.y/2), -(pt.bbox.center.position.z + pt.bbox.size.z/2), (pt.bbox.center.position.x + pt.bbox.size.x/2)));
        objectPoints.push_back(cv::Point3f(-(pt.bbox.center.position.y + pt.bbox.size.y/2), -(pt.bbox.center.position.z + pt.bbox.size.z/2), (pt.bbox.center.position.x + pt.bbox.size.x/2)));
        objectPoints.push_back(cv::Point3f(-(pt.bbox.center.position.y + pt.bbox.size.y/2), -(pt.bbox.center.position.z + pt.bbox.size.z/2), (pt.bbox.center.position.x - pt.bbox.size.x/2)));
        objectPoints.push_back(cv::Point3f(-(pt.bbox.center.position.y - pt.bbox.size.y/2), -(pt.bbox.center.position.z + pt.bbox.size.z/2), (pt.bbox.center.position.x - pt.bbox.size.x/2)));
        objectPoints.push_back(cv::Point3f(-(pt.bbox.center.position.y + pt.bbox.size.y/2), -(pt.bbox.center.position.z - pt.bbox.size.z/2), (pt.bbox.center.position.x - pt.bbox.size.x/2)));
        cv::projectPoints(objectPoints, rVec, tVec, K, D, projectedPoints);
        centers.push_back(projectedPoints.at(0));
        rectangles.push_back(cv::boundingRect(projectedPoints));
        objectPoints.clear();
        projectedPoints.clear();
      }

      if (debug){
        cv::Mat img(480, 640, CV_8UC3, CV_RGB(255, 255, 255));
        for(size_t i = 0; i < objectStrings.size(); i++){
          cv::circle(img, centers.at(i), 5, CV_RGB(255,0,0));
          cv::putText(img, objectStrings.at(i), centers.at(i), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0));
          cv::rectangle(img, rectangles.at(i), CV_RGB(0,0,255));
        }

        cv_bridge::CvImage out_msg;
        out_msg.image = img;
        out_msg.header = trans_det.detections.at(0).header;
        out_msg.encoding = "bgr8";
        out_msg.header.frame_id = output_frame;
        //out_msg.header.stamp = detections->markers.at(0).header.stamp;

        img_publisher_->publish(*out_msg.toImageMsg());
      }

      vision_msgs::msg::Detection2DArray::UniquePtr detect(new vision_msgs::msg::Detection2DArray);
      detect->header=trans_det.detections.at(0).header;
      detect->detections.reserve(objectStrings.size());
      for (size_t i = 0; i < objectStrings.size(); ++i) {
        detect->detections.emplace_back();
        auto & detection_ros = detect->detections.back();
        detection_ros.header = detect->header;
        for (size_t j = 0; j < detect->detections.at(i).results.size(); ++j) {
          detection_ros.results.push_back(detect->detections.at(i).results.at(j));
        }        
        // Copy bounding box, darknet uses center of bounding box too
        detection_ros.bbox.center.x = static_cast<double>(centers.at(i).x);
        detection_ros.bbox.center.y = static_cast<double>(centers.at(i).y);
        detection_ros.bbox.size_x = static_cast<double>(rectangles.at(i).width);
        detection_ros.bbox.size_y = static_cast<double>(rectangles.at(i).height);
        detection_ros.is_tracking = true;
        detection_ros.tracking_id = objectStrings.at(i);
      }
      if (detect->detections.size() == 0)
      {
        ss << "Projection has 0 detections." << std::endl;
        RCLCPP_WARN(get_logger(),ss.str());
        ss.clear();
        return;
      } 
      publisher_->publish(*detect);
      if (debug){
        ss << "Published detections number:" << detect->detections.size();
        RCLCPP_INFO(get_logger(),ss.str());
        ss.clear();
      }
    }
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det_subscription_{nullptr};
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr publisher_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_{nullptr}; 
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};

    cv::Mat K = cv::Mat(3, 3, CV_64F); 
    cv::Mat D = cv::Mat(5, 1, CV_64F); // (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]])

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

