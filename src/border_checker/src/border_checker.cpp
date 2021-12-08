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

class BorderChecker : public rclcpp::Node
{
  public:
    BorderChecker() : Node("border_checker")
    {
      this->declare_parameter("input_topic", "/cloud");
      this->declare_parameter("output_topic", "topic");
      this->declare_parameter("mean_k", 50);
      this->declare_parameter("standard_dev_thresh", 1.0);
      this->declare_parameter("first_filter_limits_min", -2.0);
      this->declare_parameter("first_filter_limits_max", -0.1);
      this->declare_parameter("first_filter_field", "z");
      this->declare_parameter("second_filter_limits_min", 2.0);
      this->declare_parameter("second_filter_limits_max", 10.0);
      this->declare_parameter("second_filter_field", "z");
      
      this->declare_parameter("debug", false);
      
      mean_k = this->get_parameter("mean_k").as_int();
      standard_dev_thresh = this->get_parameter("standard_dev_thresh").as_double();
      first_filter_field = this->get_parameter("first_filter_field").as_string();
      first_filter_limits_min = this->get_parameter("first_filter_limits_min").as_double();
      first_filter_limits_max = this->get_parameter("first_filter_limits_max").as_double();
      second_filter_field = this->get_parameter("second_filter_field").as_string();
      second_filter_limits_min = this->get_parameter("second_filter_limits_min").as_double();
      second_filter_limits_max = this->get_parameter("second_filter_limits_max").as_double();  

      debug = this->get_parameter("debug").as_bool();
      
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->get_parameter("input_topic").as_string(), 10, std::bind(&BorderChecker::topic_callback, this, std::placeholders::_1)); //livox/lidar
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("output_topic").as_string(), 10);

      RCLCPP_INFO(this->get_logger(), "Border_checker has started.");
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg,*cloud);

      // Remove noise
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(this->mean_k);
      sor.setStddevMulThresh(this->standard_dev_thresh);
      sor.filter(*cloud);

      // Floor filtration
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName(first_filter_field);
      pass.setFilterLimits(first_filter_limits_min, first_filter_limits_max);
      pass.setFilterLimitsNegative (true);
      pass.filter(*cloud);

      // Ceiling filtration
      pass.setFilterFieldName(second_filter_field);
      pass.setFilterLimits(second_filter_limits_min, second_filter_limits_max);
      pass.setFilterLimitsNegative (true);
      pass.filter(*cloud);

      pcl::toROSMsg(*cloud, *msg);
      publisher_->publish(*msg);
      if (debug)
        RCLCPP_INFO(this->get_logger(), "Publishing: filtered cloud.");
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    double first_filter_limits_min, first_filter_limits_max, second_filter_limits_min, second_filter_limits_max;
    std::string first_filter_field, second_filter_field;

    int mean_k;
    double standard_dev_thresh;

    bool debug;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BorderChecker>());
  rclcpp::shutdown();
  return 0;
}
