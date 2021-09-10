#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "pcl_conversions/pcl_conversions.h"

//#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

class BorderChecker : public rclcpp::Node
{
  public:
    BorderChecker()
    : Node("border_checker")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/lidar", 10, std::bind(&BorderChecker::topic_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", 10);

      RCLCPP_INFO(this->get_logger(), "Border_checker has started.");
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg,*cloud);

      // Create the filtering object
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(0.1, 1.1);
      //pass.setFilterLimitsNegative (true);
      pass.filter(*cloud_filtered);
      /*std::cerr << *cloud_filtered << std::endl;
      for (const auto& point: *cloud)
      std::cerr << "    " << point.x << " "
                          << point.y << " "
                          << point.z << std::endl;

      
      pcl::ExtractIndices<PointType> filter;
      filter.setInputCloud(cloud_in);
      filter.setIndices(indices_in);
      filter.filter(*cloud_filtered);
      */
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(cloud_filtered);
      sor.setMeanK(50);
      sor.setStddevMulThresh(1.0);
      sor.filter(*cloud_filtered);

      pcl::toROSMsg(*cloud_filtered, *msg);
      publisher_->publish(*msg);
      RCLCPP_INFO(this->get_logger(), "Publishing: filtered cloud.");
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BorderChecker>());
  rclcpp::shutdown();
  return 0;
}