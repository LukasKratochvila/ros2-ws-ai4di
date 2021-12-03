#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "pcl_conversions/pcl_conversions.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>




#include <pcl/filters/voxel_grid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/impl/search.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class Clustering : public rclcpp::Node
{
  public:
    Clustering() : Node("Clustering")
    {
      this->declare_parameter("filter_limits_min", 2.0);
      this->declare_parameter("filter_limits_max", 10.0);
      this->declare_parameter("filter_field", "z");
      this->declare_parameter("mean_k", 50);
      this->declare_parameter("standard_dev_thresh", 1.0);
      this->declare_parameter("output_topic", "topic");

      filter_limits_min_ = this->get_parameter("filter_limits_min").as_double();
      filter_limits_max_ = this->get_parameter("filter_limits_max").as_double();
      filter_field_ = this->get_parameter("filter_field").as_string();

      mean_k_ = this->get_parameter("mean_k").as_int();
      standard_dev_thresh_ = this->get_parameter("standard_dev_thresh").as_double();

      output_topic_ = this->get_parameter("output_topic").as_string();
      
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud", 10, std::bind(&Clustering::topic_callback, this, std::placeholders::_1)); //livox/lidar
      marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detections", 10);

      RCLCPP_INFO(this->get_logger(), "Clustering node has started.");
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg,*cloud);

      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName(filter_field_);
      pass.setFilterLimits(-2,-0.1);
      pass.setFilterLimitsNegative (true);
      pass.filter(*cloud_filtered);

      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName(filter_field_);
      pass.setFilterLimits(filter_limits_min_,filter_limits_max_);
      pass.setFilterLimitsNegative (true);
      pass.filter(*cloud_filtered);

      std::stringstream log;
      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_filtered->makeShared());
      
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(0.20); // 20cm
      ec.setMinClusterSize (20);
      ec.setMaxClusterSize (10000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_filtered->makeShared());
      ec.extract (cluster_indices);
      //log << "PointCloud representing: " << cluster_indices.size () << " Clusters." << std::endl;

      std::vector<pcl::PointXYZ> min_list;
      std::vector<pcl::PointXYZ> max_list;

      for (auto& it : cluster_indices)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ minPoint = {NAN, NAN, NAN};
        pcl::PointXYZ maxPoint = {NAN, NAN, NAN};

        for (auto pit = it.indices.begin()+1 ; pit < it.indices.end() ; pit++) { // dont know wky, but sometimes point at index [0] is {0,0,0}
            const auto& point = cloud_filtered->makeShared()->points[*pit];

            if (*pit != it.indices[3]) {
                if(minPoint.x > point.x) {minPoint.x = point.x;}
                if(minPoint.y > point.y) {minPoint.y = point.y;}
                if(minPoint.z > point.z) {minPoint.z = point.z;}
                if(maxPoint.x < point.x) {maxPoint.x = point.x;}
                if(maxPoint.y < point.y) {maxPoint.y = point.y;}
                if(maxPoint.z < point.z) {maxPoint.z = point.z;}
            } else {
                minPoint.x = point.x;minPoint.y = point.y;minPoint.z = point.z;
                maxPoint.x = point.x;maxPoint.y = point.y;maxPoint.z = point.z;
            }
        }
        min_list.emplace_back(minPoint);
        max_list.emplace_back(maxPoint);
      }

      visualization_msgs::msg::MarkerArray output;

      auto timestamp = builtin_interfaces::msg::Time();
      auto frame = msg->header.frame_id;

      size_t cnt = 0;

      for( size_t i = 0; i < min_list.size(); i++) {

          double dx = max_list.at(i).x - min_list.at(i).x;
          double dy = max_list.at(i).y - min_list.at(i).y;
          double dz = max_list.at(i).z - min_list.at(i).z;

          if (dx == 0 || dy == 0 || dz == 0)
            continue;

          double cx = (max_list.at(i).x + min_list.at(i).x) / 2;
          double cy = (max_list.at(i).y + min_list.at(i).y) / 2;
          double cz = (max_list.at(i).z + min_list.at(i).z) / 2;

          // Bounding Box
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = frame;
          marker.header.stamp = timestamp;
          marker.id = cnt++;
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.action = visualization_msgs::msg::Marker::ADD;

          marker.pose.position.x = cx;
          marker.pose.position.y = cy;
          marker.pose.position.z = cz;

          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          marker.scale.x = dx;
          marker.scale.y = dy;
          marker.scale.z = dz;

          marker.color.a = 0.6;
          marker.color.r = 0.7;
          marker.color.g = 0.7;
          marker.color.b = 0.7;

          output.markers.push_back(marker);

          visualization_msgs::msg::Marker text;
          text.header.frame_id = frame;
          text.header.stamp = timestamp;
          text.id = cnt++;
          text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          text.action = visualization_msgs::msg::Marker::ADD;

          std::stringstream sstream;
          sstream << "ID: " << i;// << std::endl;
                  //<< "TTL: " << detection->getTTL() << std::endl;
          text.text = sstream.str();

          text.pose.position.x = max_list.at(i).x;
          text.pose.position.y = max_list.at(i).y;
          text.pose.position.z = max_list.at(i).z;

          text.scale.x = 1;
          text.scale.y = 1;
          text.scale.z = 1;

          text.color.a = 0.6;
          text.color.r = 0.7;
          text.color.g = 0.7;
          text.color.b = 0.7;

          output.markers.push_back(text);
        }
        marker_publisher_->publish(output);

        //log << "Publishing: " << min_list.size() << " detections.";
        //RCLCPP_INFO(this->get_logger(), log.str());
      
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    double filter_limits_min_;
    double filter_limits_max_;
    std::string filter_field_;

    int mean_k_;
    double standard_dev_thresh_;

    std::string output_topic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clustering>());
  rclcpp::shutdown();
  return 0;
}
